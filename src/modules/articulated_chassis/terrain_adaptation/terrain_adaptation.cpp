#include "terrain_adaptation.hpp"
#include <px4_platform_common/log.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/px4_config.h>
#include <drivers/drv_hrt.h>

using namespace time_literals;

TerrainAdaptation::TerrainAdaptation() :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

bool TerrainAdaptation::init()
{
    // Update parameters
    updateParams();

    // Start the work queue task
    ScheduleOnInterval(50_ms); // Run at 20 Hz

    PX4_INFO("Terrain Adaptation initialized - running at 20 Hz");

    return true;
}

void TerrainAdaptation::set_classifier_method(int method)
{
    if (method >= 0 && method <= 2) {
        _param_classifier_method.set(method);
        _param_classifier_method.commit();
        PX4_INFO("Classifier method set to %d", method);
    } else {
        PX4_WARN("Invalid classifier method: %d. Must be 0, 1, or 2", method);
    }
}

void TerrainAdaptation::reset_terrain_memory()
{
    // Reset terrain state
    _terrain_state.current_surface = SurfaceClassifier::UNKNOWN;
    _terrain_state.current_roughness = 0.0f;
    _terrain_state.adaptation_confidence = 0.5f;
    _terrain_state.adaptation_active = false;
    _terrain_state.last_classification_time = 0;

    // Reset vibration analyzer
    _vibration_analyzer._buffer_index = 0;

    PX4_INFO("Terrain memory and calibration reset");
}

void TerrainAdaptation::Run()
{
    if (!_param_enable.get()) {
        // Publish inactive status and exit
        terrain_adaptation_s adaptation{};
        adaptation.timestamp = hrt_absolute_time();
        adaptation.terrain_adaptation_active = false;
        _terrain_adaptation_pub.publish(adaptation);
        return;
    }

    while (!should_exit()) {
        // Update terrain classification
        updateTerrainClassification();

        // Update slope analysis
        updateSlopeAnalysis();

        // Compute adaptive strategy
        terrain_adaptation_s adaptation{};
        computeAdaptiveStrategy(adaptation);

        // Publish terrain adaptation status
        _terrain_adaptation_pub.publish(adaptation);

        // Apply terrain-specific adaptations
        if (adaptation.terrain_adaptation_active) {
            applyTerrainAdaptation(adaptation);
        }

        // Monitor performance for learning
        monitorPerformance();

        px4_usleep(50000); // Run at 20Hz
    }
}

void TerrainAdaptation::updateTerrainClassification()
{
    SurfaceClassifier::SurfaceFeatures features{};

    // Get vibration data from accelerometer
    sensor_accel_s accel;
    if (_sensor_accel_sub.copy(&accel)) {
        float accel_magnitude = sqrtf(accel.x * accel.x + accel.y * accel.y + accel.z * accel.z);
        _vibration_analyzer.addSample(accel_magnitude);

        features.vibration_amplitude = _vibration_analyzer.computeVibrationAmplitude();
        features.vibration_frequency = _vibration_analyzer.computeDominantFrequency();
    }

    // Get slip characteristics
    slip_estimation_s slip;
    if (_slip_estimation_sub.copy(&slip)) {
        features.friction_coefficient = slip.friction_coefficient;
        features.slip_characteristics = (fabsf(slip.slip_ratio_front) +
                                       fabsf(slip.slip_ratio_rear)) / 2.0f;

        // Use surface roughness from slip estimator if available
        if (slip.surface_roughness > 0.0f) {
            features.vibration_amplitude = math::max(features.vibration_amplitude, slip.surface_roughness);
        }
    }

    // Environmental estimation
    _environmental_sensing.update(features.friction_coefficient,
                                 features.slip_characteristics,
                                 features.vibration_amplitude);

    EnvironmentalSensing::EnvironmentalData env_data = _environmental_sensing.getData();
    features.temperature = env_data.ambient_temperature;
    features.moisture_indicator = env_data.humidity_estimate;

    // Classify surface
    SurfaceClassifier::SurfaceType new_surface = _surface_classifier.classify(features);

    // Update terrain state if classification changed
    if (new_surface != _terrain_state.current_surface) {
        _terrain_state.current_surface = new_surface;
        _terrain_state.current_roughness = _surface_classifier.getSurfaceRoughness(new_surface);
        _terrain_state.last_classification_time = hrt_absolute_time();

        const char* surface_names[] = {"Asphalt", "Gravel", "Mud", "Sand", "Snow", "Ice", "Unknown"};
        PX4_INFO("Surface classified as: %s (roughness: %.2f)",
                 surface_names[new_surface], (double)_terrain_state.current_roughness);
    }
}

void TerrainAdaptation::updateSlopeAnalysis()
{
    vehicle_attitude_s attitude;
    vehicle_local_position_s position;

    if (_vehicle_attitude_sub.copy(&attitude) && _vehicle_local_position_sub.copy(&position)) {
        matrix::Eulerf euler(matrix::Quatf(attitude.q));

        _slope_analyzer.update(euler.theta(), euler.phi(), euler.psi(), -position.z);
        _terrain_state.current_slope = _slope_analyzer.getCurrentSlope();
    }
}

void TerrainAdaptation::computeAdaptiveStrategy(terrain_adaptation_s &output)
{
    output.timestamp = hrt_absolute_time();
    output.terrain_type = static_cast<uint8_t>(_terrain_state.current_surface);
    output.surface_roughness = _terrain_state.current_roughness;
    output.slope_angle_rad = _terrain_state.current_slope.grade_angle_rad;
    output.banking_angle_rad = _terrain_state.current_slope.cross_slope_rad;

    // Get current load information
    float load_factor = 1.0f; // Default
    load_sensing_s load;
    if (_load_sensing_sub.copy(&load)) {
        load_factor = (load.payload_mass_kg + 2000.0f) / 4000.0f; // Normalize to [0.5, 1.5]
    }

    // Get current speed
    float current_speed = 0.0f;
    vehicle_local_position_s pos;
    if (_vehicle_local_position_sub.copy(&pos)) {
        current_speed = sqrtf(pos.vx * pos.vx + pos.vy * pos.vy);
    }

    // Compute control strategy
    AdaptiveController::ControlStrategy strategy = _adaptive_controller.computeStrategy(
        _terrain_state.current_surface,
        _terrain_state.current_slope,
        load_factor,
        current_speed
    );

    // Fill output message
    output.friction_coefficient = _surface_classifier.getExpectedFriction(_terrain_state.current_surface);
    output.adapted_max_speed = strategy.max_speed_ms;
    output.adapted_acceleration = strategy.max_speed_ms * 0.5f; // Estimate acceleration limit
    output.adapted_traction_mode = strategy.traction_control_gain;

    // Add penetration resistance based on terrain type
    switch (_terrain_state.current_surface) {
        case SurfaceClassifier::ASPHALT:
            output.penetration_resistance = 0.1f;
            break;
        case SurfaceClassifier::GRAVEL:
            output.penetration_resistance = 0.3f;
            break;
        case SurfaceClassifier::MUD:
            output.penetration_resistance = 0.8f;
            break;
        case SurfaceClassifier::SAND:
            output.penetration_resistance = 0.6f;
            break;
        case SurfaceClassifier::SNOW:
            output.penetration_resistance = 0.4f;
            break;
        case SurfaceClassifier::ICE:
            output.penetration_resistance = 0.1f;
            break;
        default:
            output.penetration_resistance = 0.5f;
            break;
    }

    // Calculate overall confidence based on terrain detection
    float slope_stability = 1.0f - fabsf(_terrain_state.current_slope.grade_angle_rad) / 0.5f; // Max 28.6 degrees
    float surface_stability = output.friction_coefficient;
    float load_stability = 2.0f - load_factor; // Higher load = lower stability

    output.terrain_confidence = math::constrain(
        slope_stability * surface_stability * load_stability, 0.0f, 1.0f);

    // Determine if adaptation should be active
    output.terrain_adaptation_active = (output.terrain_confidence < _param_stability_margin.get()) ||
                              (output.surface_roughness > _param_roughness_threshold.get()) ||
                              (fabsf(output.slope_angle_rad) > _param_slope_threshold.get());

    // Set learning mode status
    output.learning_mode_active = (_terrain_state.current_surface != SurfaceClassifier::UNKNOWN);
    output.learning_confidence = _performance_monitor.getAdaptationConfidence();
    output.terrain_transitions = 0; // TODO: implement transition counting

    // Set terrain features (simplified)
    for (int i = 0; i < 8; i++) {
        output.terrain_features[i] = 0.0f; // TODO: implement feature extraction
    }

    // Basic hazard detection
    output.hazard_detected = false;
    output.hazard_type = 0; // none
    output.hazard_severity = 0.0f;

    // Check for slope hazards
    if (fabsf(output.slope_angle_rad) > 0.4f) { // > 23 degrees
        output.hazard_detected = true;
        output.hazard_type = 1; // slope
        output.hazard_severity = fabsf(output.slope_angle_rad) / 0.5f; // normalized
    }

    // Check for low friction hazards
    if (output.friction_coefficient < 0.3f) {
        output.hazard_detected = true;
        output.hazard_type = 3; // ice/low friction
        output.hazard_severity = math::max(output.hazard_severity, (0.3f - output.friction_coefficient) / 0.3f);
    }

    _terrain_state.adaptation_active = output.terrain_adaptation_active;
    _terrain_state.adaptation_confidence = _performance_monitor.getAdaptationConfidence();

    // Store strategy for application
    if (output.terrain_adaptation_active) {
        publishAdaptationCommands(strategy);
    }
}

void TerrainAdaptation::applyTerrainAdaptation(const terrain_adaptation_s &adaptation)
{
    // Apply speed recommendations through modified wheel speed setpoints
    wheel_speeds_setpoint_s current_setpoint;
    if (_wheel_speeds_setpoint_sub.copy(&current_setpoint)) {

        // Reduce speed if recommended speed is lower than current
        float speed_magnitude = sqrtf(current_setpoint.front_wheel_speed_rad_s * current_setpoint.front_wheel_speed_rad_s +
                                     current_setpoint.rear_wheel_speed_rad_s * current_setpoint.rear_wheel_speed_rad_s);

        if (speed_magnitude > adaptation.adapted_max_speed) {
            float reduction_factor = adaptation.adapted_max_speed / speed_magnitude;

            wheel_speeds_setpoint_s adapted_setpoint = current_setpoint;
            adapted_setpoint.front_wheel_speed_rad_s *= reduction_factor;
            adapted_setpoint.rear_wheel_speed_rad_s *= reduction_factor;
            adapted_setpoint.timestamp = hrt_absolute_time();

            _wheel_speeds_setpoint_pub.publish(adapted_setpoint);

            PX4_DEBUG("Terrain adaptation: reducing speed by factor %.2f", (double)reduction_factor);
        }
    }
}

void TerrainAdaptation::publishAdaptationCommands(const AdaptiveController::ControlStrategy &strategy)
{
    // Publish traction control adjustments
    traction_control_s tc_cmd{};
    tc_cmd.timestamp = hrt_absolute_time();
    tc_cmd.traction_control_active = true;
    tc_cmd.torque_distribution = strategy.torque_distribution;

    // Adjust for terrain-specific conditions
    slip_estimation_s slip;
    if (_slip_estimation_sub.copy(&slip)) {
        // More aggressive traction control on slippery surfaces
        if (_terrain_state.current_surface == SurfaceClassifier::ICE ||
            _terrain_state.current_surface == SurfaceClassifier::SNOW) {
            tc_cmd.torque_distribution *= 0.7f; // Reduce torque demand
        }

        tc_cmd.slip_detected = slip.slip_detected ||
                              (fabsf(slip.slip_ratio_front) > 0.1f * strategy.traction_control_gain) ||
                              (fabsf(slip.slip_ratio_rear) > 0.1f * strategy.traction_control_gain);
    }

    _traction_control_pub.publish(tc_cmd);
}

void TerrainAdaptation::monitorPerformance()
{
    // Simple performance monitoring based on slip and stability
    PerformanceMonitor::PerformanceMetrics metrics{};

    slip_estimation_s slip;
    if (_slip_estimation_sub.copy(&slip)) {
        // Traction efficiency: lower slip = higher efficiency
        float avg_slip = (fabsf(slip.slip_ratio_front) + fabsf(slip.slip_ratio_rear)) / 2.0f;
        metrics.traction_efficiency = 1.0f - math::constrain(avg_slip, 0.0f, 1.0f);

        // Stability score based on lateral slip and overall vehicle dynamics
        float lateral_slip = (fabsf(slip.slip_angle_front_rad) + fabsf(slip.slip_angle_rear_rad)) / 2.0f;
        metrics.stability_score = 1.0f - math::constrain(lateral_slip, 0.0f, 0.5f) / 0.5f;
    }

    // Energy efficiency (simplified - based on speed vs terrain difficulty)
    vehicle_local_position_s pos;
    if (_vehicle_local_position_sub.copy(&pos)) {
        float current_speed = sqrtf(pos.vx * pos.vx + pos.vy * pos.vy);
        float terrain_difficulty = 1.0f + _terrain_state.current_roughness +
                                  fabsf(_terrain_state.current_slope.grade_angle_rad);
        metrics.energy_efficiency = current_speed / terrain_difficulty;
    }

    // Operator comfort (based on vibration levels)
    metrics.operator_comfort = 1.0f - math::constrain(_vibration_analyzer.computeVibrationAmplitude(), 0.0f, 10.0f) / 10.0f;

    // Task completion rate (simplified - assume successful if stable)
    metrics.task_completion_rate = metrics.stability_score;

    // Record performance for learning
    AdaptiveController::ControlStrategy current_strategy{};
    current_strategy.max_speed_ms = 10.0f; // Default values
    current_strategy.torque_distribution = 0.0f;
    current_strategy.traction_control_gain = 1.0f;

    _performance_monitor.recordPerformance(metrics, _terrain_state.current_surface, current_strategy);
}

// Surface Classifier Implementation
TerrainAdaptation::SurfaceClassifier::SurfaceClassifier()
{
    initializeSurfaceModels();
}

void TerrainAdaptation::SurfaceClassifier::initializeSurfaceModels()
{
    // Initialize surface models with typical characteristics

    // Asphalt
    _surface_models[ASPHALT] = {
        .friction_range = {0.8f, 1.2f},
        .roughness_level = 0.1f,
        .vibration_signature = {1.0f, 20.0f, 0.5f},
        .confidence = 1.0f
    };

    // Gravel
    _surface_models[GRAVEL] = {
        .friction_range = {0.6f, 0.9f},
        .roughness_level = 0.4f,
        .vibration_signature = {3.0f, 15.0f, 1.5f},
        .confidence = 1.0f
    };

    // Mud
    _surface_models[MUD] = {
        .friction_range = {0.3f, 0.6f},
        .roughness_level = 0.3f,
        .vibration_signature = {2.0f, 10.0f, 1.0f},
        .confidence = 1.0f
    };

    // Sand
    _surface_models[SAND] = {
        .friction_range = {0.4f, 0.7f},
        .roughness_level = 0.5f,
        .vibration_signature = {2.5f, 12.0f, 1.2f},
        .confidence = 1.0f
    };

    // Snow
    _surface_models[SNOW] = {
        .friction_range = {0.2f, 0.5f},
        .roughness_level = 0.2f,
        .vibration_signature = {1.5f, 8.0f, 0.8f},
        .confidence = 1.0f
    };

    // Ice
    _surface_models[ICE] = {
        .friction_range = {0.1f, 0.3f},
        .roughness_level = 0.1f,
        .vibration_signature = {0.5f, 25.0f, 0.3f},
        .confidence = 1.0f
    };

    // Unknown
    _surface_models[UNKNOWN] = {
        .friction_range = {0.5f, 0.8f},
        .roughness_level = 0.3f,
        .vibration_signature = {2.0f, 15.0f, 1.0f},
        .confidence = 0.0f
    };
}

TerrainAdaptation::SurfaceClassifier::SurfaceType
TerrainAdaptation::SurfaceClassifier::classify(const SurfaceFeatures &features)
{
    float min_distance = INFINITY;
    SurfaceType best_match = UNKNOWN;

    // Find best matching surface type
    for (int i = 0; i < 6; i++) { // Don't include UNKNOWN in classification
        float distance = calculateFeatureDistance(features, static_cast<SurfaceType>(i));
        if (distance < min_distance) {
            min_distance = distance;
            best_match = static_cast<SurfaceType>(i);
        }
    }

    // Require minimum confidence for classification
    if (min_distance > 5.0f) { // Threshold for classification confidence
        return UNKNOWN;
    }

    return best_match;
}

float TerrainAdaptation::SurfaceClassifier::calculateFeatureDistance(
    const SurfaceFeatures &features, SurfaceType type)
{
    const SurfaceModel &model = _surface_models[type];
    float distance = 0.0f;

    // Friction distance
    float friction_center = (model.friction_range[0] + model.friction_range[1]) / 2.0f;
    distance += fabsf(features.friction_coefficient - friction_center) * 2.0f; // Weight friction heavily

    // Vibration amplitude distance
    distance += fabsf(features.vibration_amplitude - model.vibration_signature[0]);

    // Vibration frequency distance
    distance += fabsf(features.vibration_frequency - model.vibration_signature[1]) * 0.1f;

    // Temperature effect (for snow/ice detection)
    if (type == SNOW || type == ICE) {
        if (features.temperature > 5.0f) { // Above freezing
            distance += 10.0f; // Heavy penalty
        }
    }

    return distance;
}

float TerrainAdaptation::SurfaceClassifier::getSurfaceRoughness(SurfaceType type)
{
    if (type < 7) {
        return _surface_models[type].roughness_level;
    }
    return 0.3f; // Default
}

float TerrainAdaptation::SurfaceClassifier::getExpectedFriction(SurfaceType type)
{
    if (type < 7) {
        const SurfaceModel &model = _surface_models[type];
        return (model.friction_range[0] + model.friction_range[1]) / 2.0f;
    }
    return 0.6f; // Default
}

// Vibration Analyzer Implementation
void TerrainAdaptation::VibrationAnalyzer::addSample(float accel_magnitude)
{
    _accel_buffer[_buffer_index] = accel_magnitude;
    _buffer_index = (_buffer_index + 1) % FFT_SIZE;
}

float TerrainAdaptation::VibrationAnalyzer::computeVibrationAmplitude()
{
    // Simple RMS calculation
    float sum_squares = 0.0f;
    for (int i = 0; i < FFT_SIZE; i++) {
        sum_squares += _accel_buffer[i] * _accel_buffer[i];
    }
    return sqrtf(sum_squares / FFT_SIZE);
}

float TerrainAdaptation::VibrationAnalyzer::computeDominantFrequency()
{
    // Simplified frequency analysis - find peak in autocorrelation
    float max_correlation = 0.0f;
    int best_lag = 1;

    for (int lag = 1; lag < FFT_SIZE / 4; lag++) {
        float correlation = 0.0f;
        for (int i = 0; i < FFT_SIZE - lag; i++) {
            correlation += _accel_buffer[i] * _accel_buffer[i + lag];
        }

        if (correlation > max_correlation) {
            max_correlation = correlation;
            best_lag = lag;
        }
    }

    // Convert lag to frequency (assuming 100Hz sampling)
    return 100.0f / best_lag;
}

// Adaptive Controller Implementation
TerrainAdaptation::AdaptiveController::AdaptiveController()
{
    initializeStrategies();
}

void TerrainAdaptation::AdaptiveController::initializeStrategies()
{
    // Initialize control strategies for each terrain type

    // Asphalt - high performance
    _terrain_strategies[SurfaceClassifier::ASPHALT] = {
        .max_speed_ms = 15.0f,
        .torque_distribution = 0.0f,  // Balanced
        .traction_control_gain = 1.0f,
        .stability_threshold = 0.3f,
        .differential_lock = false
    };

    // Gravel - moderate caution
    _terrain_strategies[SurfaceClassifier::GRAVEL] = {
        .max_speed_ms = 10.0f,
        .torque_distribution = -0.2f, // Slight rear bias
        .traction_control_gain = 1.2f,
        .stability_threshold = 0.4f,
        .differential_lock = false
    };

    // Mud - careful operation
    _terrain_strategies[SurfaceClassifier::MUD] = {
        .max_speed_ms = 6.0f,
        .torque_distribution = -0.3f, // Rear bias for traction
        .traction_control_gain = 1.5f,
        .stability_threshold = 0.5f,
        .differential_lock = true
    };

    // Sand - similar to mud but different torque
    _terrain_strategies[SurfaceClassifier::SAND] = {
        .max_speed_ms = 8.0f,
        .torque_distribution = 0.1f,  // Slight front bias
        .traction_control_gain = 1.3f,
        .stability_threshold = 0.4f,
        .differential_lock = true
    };

    // Snow - very cautious
    _terrain_strategies[SurfaceClassifier::SNOW] = {
        .max_speed_ms = 4.0f,
        .torque_distribution = 0.0f,  // Balanced for stability
        .traction_control_gain = 2.0f,
        .stability_threshold = 0.6f,
        .differential_lock = false
    };

    // Ice - extremely cautious
    _terrain_strategies[SurfaceClassifier::ICE] = {
        .max_speed_ms = 2.0f,
        .torque_distribution = 0.0f,  // Balanced
        .traction_control_gain = 3.0f,
        .stability_threshold = 0.8f,
        .differential_lock = false
    };

    // Unknown - conservative default
    _terrain_strategies[SurfaceClassifier::UNKNOWN] = {
        .max_speed_ms = 8.0f,
        .torque_distribution = 0.0f,
        .traction_control_gain = 1.5f,
        .stability_threshold = 0.5f,
        .differential_lock = false
    };
}

TerrainAdaptation::AdaptiveController::ControlStrategy
TerrainAdaptation::AdaptiveController::computeStrategy(
    SurfaceClassifier::SurfaceType surface,
    const SlopeAnalyzer::SlopeInfo &slope,
    float load_factor,
    float current_speed)
{
    // Get base strategy for surface type
    ControlStrategy strategy = _terrain_strategies[surface];

    // Blend with slope and load considerations
    strategy = blendStrategies(strategy, slope, load_factor);

    // Apply adaptive learning adjustments
    strategy.max_speed_ms *= _adaptive_params.speed_reduction_factor;
    strategy.traction_control_gain *= _adaptive_params.tc_sensitivity;
    strategy.stability_threshold += _adaptive_params.stability_margin;

    return strategy;
}

TerrainAdaptation::AdaptiveController::ControlStrategy
TerrainAdaptation::AdaptiveController::blendStrategies(
    const ControlStrategy &base,
    const SlopeAnalyzer::SlopeInfo &slope,
    float load_factor)
{
    ControlStrategy adapted = base;

    // Adjust for slope
    float slope_factor = 1.0f - fabsf(slope.grade_angle_rad) / 0.5f; // Reduce performance on slopes
    adapted.max_speed_ms *= math::constrain(slope_factor, 0.3f, 1.0f);

    // Adjust torque distribution for uphill/downhill
    if (slope.uphill) {
        adapted.torque_distribution -= 0.1f; // More rear bias when climbing
    } else {
        adapted.torque_distribution += 0.1f; // More front bias when descending
    }

    // Adjust for load
    float load_speed_factor = 2.0f - load_factor; // Higher load = lower speed
    adapted.max_speed_ms *= math::constrain(load_speed_factor, 0.5f, 1.2f);

    // Constrain values
    adapted.torque_distribution = math::constrain(adapted.torque_distribution, -1.0f, 1.0f);
    adapted.max_speed_ms = math::max(adapted.max_speed_ms, 1.0f); // Minimum speed

    return adapted;
}

// Slope Analyzer Implementation
TerrainAdaptation::SlopeAnalyzer::SlopeAnalyzer()
{
    // Initialize history arrays
    for (int i = 0; i < SLOPE_HISTORY_SIZE; i++) {
        _pitch_history[i] = 0.0f;
        _roll_history[i] = 0.0f;
        _altitude_history[i] = 0.0f;
    }
}

void TerrainAdaptation::SlopeAnalyzer::update(float pitch, float roll, float heading, float altitude)
{
    // Add to history
    addToHistory(pitch, roll, altitude);

    // Update current slope info
    _current_slope.grade_angle_rad = pitch;
    _current_slope.cross_slope_rad = roll;
    _current_slope.slope_direction_rad = heading; // Simplified
    _current_slope.slope_change_rate = calculateSlopeChangeRate();
    _current_slope.uphill = (pitch > 0.05f); // 2.9 degrees threshold
}

void TerrainAdaptation::SlopeAnalyzer::addToHistory(float pitch, float roll, float altitude)
{
    _pitch_history[_history_index] = pitch;
    _roll_history[_history_index] = roll;
    _altitude_history[_history_index] = altitude;
    _history_index = (_history_index + 1) % SLOPE_HISTORY_SIZE;
}

float TerrainAdaptation::SlopeAnalyzer::calculateSlopeChangeRate()
{
    if (_history_index < 2) return 0.0f;

    // Calculate rate of change in pitch over recent history
    int prev_index = (_history_index - 2 + SLOPE_HISTORY_SIZE) % SLOPE_HISTORY_SIZE;
    float pitch_change = _pitch_history[_history_index - 1] - _pitch_history[prev_index];

    return pitch_change / 0.1f; // Assuming 0.1s between samples
}

float TerrainAdaptation::SlopeAnalyzer::predictStabilityRisk(float vehicle_speed, float cog_height)
{
    // Simple stability risk calculation
    float slope_risk = fabsf(_current_slope.grade_angle_rad) / 0.5f; // Normalize to 28.6 degrees
    float speed_risk = vehicle_speed / 10.0f; // Normalize to 10 m/s
    float cog_risk = (cog_height - 1.0f) / 1.0f; // Normalize to 1m above standard

    return math::constrain(slope_risk + speed_risk * 0.5f + cog_risk * 0.3f, 0.0f, 1.0f);
}

// Environmental Sensing Implementation
TerrainAdaptation::EnvironmentalSensing::EnvironmentalSensing()
{
    _env_data.ambient_temperature = 20.0f; // Default 20°C
    _env_data.humidity_estimate = 0.5f;
    _env_data.precipitation_indicator = 0.0f;
    _env_data.visibility_factor = 1.0f;
}

void TerrainAdaptation::EnvironmentalSensing::update(float friction_estimate,
                                                    float slip_variance,
                                                    float vibration_level)
{
    estimateWeatherConditions(friction_estimate, slip_variance);

    // Update precipitation indicator based on slip patterns
    if (slip_variance > 0.1f && friction_estimate < 0.6f) {
        _env_data.precipitation_indicator = math::min(_env_data.precipitation_indicator + 0.1f, 1.0f);
    } else {
        _env_data.precipitation_indicator = math::max(_env_data.precipitation_indicator - 0.05f, 0.0f);
    }
}

void TerrainAdaptation::EnvironmentalSensing::estimateWeatherConditions(
    float friction, float slip_variance)
{
    // Simple weather estimation based on traction conditions
    if (friction < 0.3f && slip_variance > 0.2f) {
        // Likely wet or icy conditions
        _env_data.ambient_temperature = math::max(_env_data.ambient_temperature - 1.0f, -10.0f);
        _env_data.humidity_estimate = math::min(_env_data.humidity_estimate + 0.1f, 1.0f);
    } else if (friction > 0.8f && slip_variance < 0.05f) {
        // Likely dry conditions
        _env_data.humidity_estimate = math::max(_env_data.humidity_estimate - 0.05f, 0.0f);
    }
}

// Performance Monitor Implementation
TerrainAdaptation::PerformanceMonitor::PerformanceMonitor()
{
    for (int i = 0; i < PERFORMANCE_HISTORY_SIZE; i++) {
        _performance_history[i] = {};
    }
}

void TerrainAdaptation::PerformanceMonitor::recordPerformance(
    const PerformanceMetrics &metrics,
    SurfaceClassifier::SurfaceType surface,
    const AdaptiveController::ControlStrategy &strategy)
{
    int index = _history_count % PERFORMANCE_HISTORY_SIZE;

    _performance_history[index].metrics = metrics;
    _performance_history[index].surface = surface;
    _performance_history[index].strategy = strategy;
    _performance_history[index].timestamp = hrt_absolute_time();

    if (_history_count < PERFORMANCE_HISTORY_SIZE) {
        _history_count++;
    }

    // Update adaptation confidence based on recent performance
    float recent_score = calculateOverallScore(metrics);
    _adaptation_confidence = 0.9f * _adaptation_confidence + 0.1f * recent_score;
}

float TerrainAdaptation::PerformanceMonitor::calculateOverallScore(
    const PerformanceMetrics &metrics)
{
    // Weighted combination of performance metrics
    return 0.3f * metrics.traction_efficiency +
           0.3f * metrics.stability_score +
           0.2f * metrics.energy_efficiency +
           0.1f * metrics.operator_comfort +
           0.1f * metrics.task_completion_rate;
}

void TerrainAdaptation::PerformanceMonitor::evaluateAdaptationEffectiveness()
{
    if (_history_count < 10) return; // Need minimum history

    // Compare recent performance with historical average
    float recent_avg = 0.0f;
    float historical_avg = 0.0f;

    int recent_count = math::min(_history_count, 10);
    int historical_count = math::min(_history_count, 50);

    // Calculate recent average (last 10 records)
    for (int i = 0; i < recent_count; i++) {
        int index = (_history_count - 1 - i + PERFORMANCE_HISTORY_SIZE) % PERFORMANCE_HISTORY_SIZE;
        recent_avg += calculateOverallScore(_performance_history[index].metrics);
    }
    recent_avg /= recent_count;

    // Calculate historical average
    for (int i = 0; i < historical_count; i++) {
        int index = (_history_count - 1 - i + PERFORMANCE_HISTORY_SIZE) % PERFORMANCE_HISTORY_SIZE;
        historical_avg += calculateOverallScore(_performance_history[index].metrics);
    }
    historical_avg /= historical_count;

    // Update adaptation confidence based on improvement
    if (recent_avg > historical_avg) {
        _adaptation_confidence = math::min(_adaptation_confidence + 0.05f, 1.0f);
    } else {
        _adaptation_confidence = math::max(_adaptation_confidence - 0.02f, 0.0f);
    }
}
