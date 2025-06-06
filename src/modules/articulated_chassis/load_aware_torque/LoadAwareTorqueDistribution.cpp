#include "LoadAwareTorqueDistribution.hpp"
#include <px    // Main control loop
    while (!should_exit()) {

        // Check for parameter updates
        if (_parameter_update_sub.updated()) {
            parameter_update_s param_update;
            _parameter_update_sub.copy(&param_update);
            updateParams();
        }

        // Update load state from sensors
        update_load_state();

        // Update articulation state from wheel loader feedback
        update_articulation_state();

        // Calculate static weight distributionmon/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_local_position.h>
#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>

LoadAwareTorqueDistribution::LoadAwareTorqueDistribution() :
    ModuleBase(MODULE_NAME, px4::wq_configurations::hp_default)
{
    // Initialize module status
    _status = {};
    _status.module_id = MODULE_ID_LOAD_AWARE_TORQUE;
    _status.timestamp = hrt_absolute_time();

    // Initialize torque output
    _torque_output = {};
}

int LoadAwareTorqueDistribution::task_spawn(int argc, char *argv[])
{
    _task_id = px4_task_spawn_cmd("load_aware_torque",
                                  SCHED_DEFAULT,
                                  SCHED_PRIORITY_FAST_DRIVER,
                                  2560,
                                  (px4_main_t)&run_trampoline,
                                  (char *const *)argv);

    if (_task_id < 0) {
        _task_id = -1;
        return -errno;
    }

    return 0;
}

bool LoadAwareTorqueDistribution::init()
{
    PX4_INFO("Load-Aware Torque Distribution initialized");
    PX4_INFO("Control rate: %.1f Hz", (double)CONTROL_RATE_HZ);
    PX4_INFO("Torque ratio limits: %.1f - %.1f",
             (double)_min_front_ratio.get(), (double)_max_front_ratio.get());

    return true;
}

void LoadAwareTorqueDistribution::run()
{
    if (!init()) {
        PX4_ERR("Load-aware torque distribution initialization failed");
        return;
    }

    // Main control loop
    while (!should_exit()) {

        // Check for parameter updates
        if (_parameter_update_sub.updated()) {
            parameter_update_s param_update;
            _parameter_update_sub.copy(&param_update);
            updateParams();
        }

        // Update load state from sensors
        update_load_state();

        // Update articulation state
        update_articulation_state();

        // Calculate static weight distribution
        calculate_static_weight_distribution();

        // Calculate dynamic weight transfer
        calculate_dynamic_weight_transfer();

        // Calculate base torque distribution
        calculate_base_torque_distribution();

        // Apply dynamic adjustments
        apply_dynamic_adjustments();

        // Apply stability corrections
        apply_stability_corrections();

        // Apply terrain adaptations
        apply_terrain_adaptations();

        // Validate and apply safety limits
        validate_torque_distribution();

        // Optimize for efficiency/traction
        if (_efficiency_optimize.get()) {
            optimize_for_efficiency();
        } else {
            optimize_for_traction();
        }

        // Publish torque commands
        publish_torque_commands();

        // Update performance metrics
        update_performance_metrics();

        // Sleep until next control cycle
        px4_usleep(CONTROL_INTERVAL_US);
    }
}

void LoadAwareTorqueDistribution::update_load_state()
{
    // Get load sensing data
    if (_load_sensing_sub.updated()) {
        load_sensing_s load_data;
        _load_sensing_sub.copy(&load_data);

        _load_state.payload_mass_kg = load_data.payload_mass_kg;
        _load_state.total_mass_kg = EMPTY_MASS_KG + _load_state.payload_mass_kg;
        _load_state.cog_x_m = load_data.cog_position_x_m + _cog_x_offset.get();
        _load_state.cog_z_m = load_data.cog_position_z_m + _cog_z_offset.get();
        _load_state.front_axle_load_kg = load_data.front_axle_load_kg;
        _load_state.rear_axle_load_kg = load_data.rear_axle_load_kg;
        _load_state.dynamic_load_transfer = load_data.dynamic_load_transfer;
        _load_state.load_state = load_data.load_state;
        _load_state.load_stable = load_data.load_stable;
        _load_state.load_data_valid = true;
        _load_state.last_update_time = hrt_absolute_time();
    }

    // Get vehicle dynamics
    if (_vehicle_attitude_sub.updated()) {
        vehicle_attitude_s attitude;
        _vehicle_attitude_sub.copy(&attitude);

        // Convert quaternion to Euler angles
        matrix::Eulerf euler(matrix::Quatf(attitude.q));
        _dynamics_state.pitch_angle_rad = euler.phi();
        _dynamics_state.roll_angle_rad = euler.theta();
        _dynamics_state.yaw_rate_rad_s = attitude.rollspeed; // Using as yaw rate
        _dynamics_state.dynamics_valid = true;
    }

    // Get position for acceleration calculation
    if (_vehicle_local_position_sub.updated()) {
        vehicle_local_position_s position;
        _vehicle_local_position_sub.copy(&position);

        _dynamics_state.vehicle_speed_ms = sqrtf(position.vx * position.vx + position.vy * position.vy);
        _dynamics_state.longitudinal_accel_ms2 = position.ax;
        _dynamics_state.lateral_accel_ms2 = position.ay;
    }

    // Get traction information
    if (_slip_estimation_sub.updated()) {
        slip_estimation_s slip_data;
        _slip_estimation_sub.copy(&slip_data);

        _traction_state.slip_front = slip_data.longitudinal_slip_front;
        _traction_state.slip_rear = slip_data.longitudinal_slip_rear;
        _traction_state.friction_coefficient = slip_data.surface_friction_estimate;
    }

    // Get terrain information
    if (_terrain_adaptation_sub.updated()) {
        terrain_adaptation_s terrain;
        _terrain_adaptation_sub.copy(&terrain);

        _traction_state.surface_roughness = terrain.surface_roughness;
        _traction_state.surface_type = terrain.surface_type;
        _dynamics_state.slope_angle_rad = terrain.slope_angle_rad;
    }

    // Get traction control status
    if (_traction_control_sub.updated()) {
        traction_control_s traction;
        _traction_control_sub.copy(&traction);

        _traction_state.traction_control_active = traction.traction_control_active;
        _traction_state.traction_limit_factor = math::constrain(
            fabsf(traction.torque_distribution), 0.0f, 1.0f);
    }
}

void LoadAwareTorqueDistribution::update_articulation_state()
{
    // Check for updates from wheel loader status topic
    if (_wheel_loader_status_sub.updated()) {
        wheel_loader_status_s status;
        _wheel_loader_status_sub.copy(&status);

        // Update articulation angle in dynamics state
        _dynamics_state.articulation_angle_rad = status.articulation_angle_feedback;

        // Validate dynamics data
        _dynamics_state.dynamics_valid = true;

        PX4_DEBUG("Updated articulation angle: %.2f rad (%.1f degrees)",
                 (double)_dynamics_state.articulation_angle_rad,
                 (double)math::degrees(_dynamics_state.articulation_angle_rad));
    }
}

void LoadAwareTorqueDistribution::calculate_static_weight_distribution()
{
    if (!_load_state.load_data_valid) {
        return;
    }

    // Calculate weight distribution based on center of gravity
    // For articulated wheel loader: COG_x > 0 means COG shifted forward from vehicle center
    // This increases front axle load (typical when bucket is loaded)
    float wheelbase = WHEELBASE_M;

    // For articulated vehicle, calculate effective wheelbase adjustment based on articulation angle
    // During articulation, the effective wheelbase changes slightly
    float articulation_adjustment = 1.0f;
    if (fabsf(_dynamics_state.articulation_angle_rad) > 0.05f) {
        // Adjust effective wheelbase - becomes slightly shorter as articulation increases
        articulation_adjustment = cosf(_dynamics_state.articulation_angle_rad);
        wheelbase = wheelbase * articulation_adjustment;
    }

    // Moment arms measured from rear axle (correct physics approach)
    // When COG shifts forward (+X), front moment arm decreases, rear moment arm increases
    float rear_moment_arm = wheelbase - _load_state.cog_x_m;  // Distance from rear axle to COG
    float front_moment_arm = _load_state.cog_x_m;             // Distance from front axle to COG

    // Moment balance: Total_Weight × COG_distance_from_rear = Front_Weight × wheelbase
    // Front_Weight = (Total_Weight × rear_moment_arm) / wheelbase
    float front_weight_ratio = rear_moment_arm / wheelbase;
    float rear_weight_ratio = front_moment_arm / wheelbase;

    // Validate moment balance (should sum to 1.0)
    float ratio_sum = front_weight_ratio + rear_weight_ratio;
    if (fabsf(ratio_sum - 1.0f) > 0.01f) {
        // Normalize if calculation error
        front_weight_ratio /= ratio_sum;
        rear_weight_ratio /= ratio_sum;
    }

    // Clamp to physical limits for articulated wheel loader
    // Front axle can carry 20-75% of load depending on bucket state
    front_weight_ratio = math::constrain(front_weight_ratio, 0.2f, 0.75f);
    rear_weight_ratio = 1.0f - front_weight_ratio;

    // For optimal traction, torque distribution should match weight distribution
    // Heavier axle gets proportionally more torque
    _torque_distribution.base_distribution = front_weight_ratio;

    // Store calculated axle loads
    float calculated_rear_load = _load_state.total_mass_kg * rear_weight_ratio;
    float calculated_front_load = _load_state.total_mass_kg * front_weight_ratio;

    // Validate against measured loads if available
    if (_load_state.front_axle_load_kg > 0 && _load_state.rear_axle_load_kg > 0) {
        float measured_total = _load_state.front_axle_load_kg + _load_state.rear_axle_load_kg;
        float measured_front_ratio = _load_state.front_axle_load_kg / measured_total;

        // Blend calculated and measured values
        _torque_distribution.base_distribution = 0.7f * _torque_distribution.base_distribution +
                                                0.3f * measured_front_ratio;
    }
}

void LoadAwareTorqueDistribution::calculate_dynamic_weight_transfer()
{
    if (!_dynamics_state.dynamics_valid) {
        return;
    }

    float dynamic_adjustment = 0.0f;

    // Get time delta for dynamic calculations
    uint64_t current_time = hrt_absolute_time();
    float dt = 0.02f; // Default to 50Hz if first run

    if (_load_state.last_update_time > 0) {
        dt = (current_time - _load_state.last_update_time) / 1e6f;
    }
    _load_state.last_update_time = current_time;

    // Longitudinal weight transfer due to acceleration/braking
    if (fabsf(_dynamics_state.longitudinal_accel_ms2) > 0.5f) {
        // Physics: During acceleration, inertial force acts backward on COG
        // This creates additional load on rear axle, reduces front axle load
        // Weight transfer = (mass × accel × COG_height) / wheelbase
        float accel_factor = _dynamics_state.longitudinal_accel_ms2 / 9.81f; // Normalize to g-force
        float height_factor = _load_state.cog_z_m / WHEELBASE_M; // Height influence

        // Positive acceleration (forward) reduces front weight, increases rear weight
        // Negative value means reducing front torque ratio during acceleration
        dynamic_adjustment = -accel_factor * height_factor * _dynamic_gain.get();
        dynamic_adjustment = math::constrain(dynamic_adjustment, -MAX_WEIGHT_TRANSFER_RATIO, MAX_WEIGHT_TRANSFER_RATIO);
    }

    // Slope influence on weight distribution
    if (fabsf(_dynamics_state.slope_angle_rad) > 0.087f) { // > 5 degrees
        // On uphill slope: weight transfers to rear axle (component of gravity)
        // On downhill slope: weight transfers to front axle
        float slope_factor = sinf(_dynamics_state.slope_angle_rad);
        // Positive slope (uphill) reduces front weight ratio
        dynamic_adjustment -= slope_factor * 0.2f; // More rear bias on uphill
    }

    _torque_distribution.dynamic_adjustment = dynamic_adjustment;
}

void LoadAwareTorqueDistribution::apply_dynamic_adjustments()
{
    // Apply dynamic adjustment to base distribution
    float adjusted_distribution = _torque_distribution.base_distribution +
                                 _torque_distribution.dynamic_adjustment;

    // Apply rate limiting to prevent sudden changes
    uint64_t current_time = hrt_absolute_time();
    float dt = 0.02f; // Default control period

    if (_filter_state.last_filter_time > 0) {
        dt = (current_time - _filter_state.last_filter_time) / 1e6f;
        float max_change = _distribution_rate_limit.get() * dt;

        float distribution_change = adjusted_distribution - _filter_state.previous_distribution;
        if (fabsf(distribution_change) > max_change) {
            adjusted_distribution = _filter_state.previous_distribution +
                                   copysignf(max_change, distribution_change);
        }
    }

    _filter_state.previous_distribution = adjusted_distribution;
    _filter_state.last_filter_time = current_time;

    // Apply low-pass filtering
    float filter_alpha = 1.0f - expf(-dt / _distribution_filter_tc.get());
    _filter_state.filtered_distribution = filter_alpha * adjusted_distribution +
                                         (1.0f - filter_alpha) * _filter_state.filtered_distribution;
}

void LoadAwareTorqueDistribution::apply_stability_corrections()
{
    float stability_correction = 0.0f;

    // Stability correction based on slip difference
    float slip_difference = _traction_state.slip_rear - _traction_state.slip_front;
    if (fabsf(slip_difference) > 0.05f) {
        // If rear slips more, shift torque to front
        // Use proportional control with saturation
        stability_correction = slip_difference * _stability_gain.get();
        stability_correction = math::constrain(stability_correction, -0.3f, 0.3f);
        _performance.stability_interventions++;
    }

    // Lateral stability correction - during high lateral acceleration
    if (fabsf(_dynamics_state.lateral_accel_ms2) > 2.0f) {
        // For articulated wheel loader: reduce front torque during aggressive steering
        // to prevent front axle from breaking traction and losing steering control
        float lateral_factor = (_dynamics_state.lateral_accel_ms2 / 9.81f) * 0.05f; // Scale by g-force
        stability_correction -= lateral_factor;
    }

    // Articulation angle stability correction - key addition for articulated vehicles
    if (fabsf(_dynamics_state.articulation_angle_rad) > 0.17f) {  // > 10 degrees
        // Articulation impacts weight distribution and stability
        // Large articulation angles shift more load to the outer wheels
        // Calculate correction factor based on articulation angle
        float articulation_factor = sinf(_dynamics_state.articulation_angle_rad) * 0.2f;

        // Adjust torque distribution to maintain stability during articulation
        // Positive articulation angle (turning right) shifts weight to left wheels
        // For stability, we typically want more torque on the axle with better weight
        stability_correction += articulation_factor * _stability_gain.get();

        // Limit the articulation correction
        stability_correction = math::constrain(stability_correction, -0.3f, 0.3f);
        PX4_DEBUG("Applied articulation correction: %.3f", (double)articulation_factor);
    }

    // Yaw stability - for articulated vehicles with front steering
    if (fabsf(_dynamics_state.yaw_rate_rad_s) > 0.3f) {
        // During rapid turns, maintain steering authority by ensuring front wheels have adequate traction
        // but prevent over-torquing which can cause understeer
        float yaw_factor = (_dynamics_state.yaw_rate_rad_s / 1.0f) * 0.03f; // Normalize to typical max yaw rate

        // For articulated wheel loader: slight front bias helps maintain steering control
        if (fabsf(_dynamics_state.yaw_rate_rad_s) > 0.5f) {
            stability_correction += 0.05f; // Small front bias for steering authority
        }
    }

    // Prevent excessive correction
    stability_correction = math::constrain(stability_correction, -0.4f, 0.4f);

    _torque_distribution.stability_correction = stability_correction;
}

void LoadAwareTorqueDistribution::apply_terrain_adaptations()
{
    float terrain_adjustment = 0.0f;

    // Terrain-specific adjustments for articulated wheel loader
    // Note: Positive values increase front torque ratio, negative values increase rear ratio
    switch (_traction_state.surface_type) {
        case 0: // Asphalt
            terrain_adjustment = 0.0f; // No adjustment needed - use weight-based distribution
            break;
        case 1: // Gravel
            terrain_adjustment = 0.05f; // Slightly more front bias for steering control
            break;
        case 2: // Mud
            // In mud, rear wheels provide pushing power while front steers
            // But for loaded wheel loader, front axle is heavier and provides better traction
            terrain_adjustment = 0.0f; // Let weight distribution dominate
            break;
        case 3: // Sand
            // Sand requires balanced approach - avoid spinning either axle
            terrain_adjustment = -0.05f; // Slight rear bias to prevent front wheel digging
            break;
        case 4: // Snow
            terrain_adjustment = 0.1f; // Front bias for steering control and flotation
            break;
        case 5: // Ice
            terrain_adjustment = 0.15f; // Maximum front bias for steering authority
            break;
        default:
            terrain_adjustment = 0.0f;
    }

    // Adjust based on surface roughness - rough terrain benefits from weight-matched distribution
    // Reduce terrain bias on very rough surfaces to let weight distribution dominate
    float roughness_factor = math::constrain(_traction_state.surface_roughness, 0.0f, 1.0f);
    terrain_adjustment *= (1.0f - roughness_factor * 0.5f);

    // Apply terrain gain parameter
    terrain_adjustment *= _terrain_gain.get();

    _torque_distribution.terrain_adaptation = terrain_adjustment;
}

void LoadAwareTorqueDistribution::validate_torque_distribution()
{
    // Combine all adjustments
    float final_distribution = _filter_state.filtered_distribution +
                              _torque_distribution.stability_correction +
                              _torque_distribution.terrain_adaptation;

    // Apply hard limits
    final_distribution = math::constrain(final_distribution,
                                        _min_front_ratio.get(),
                                        _max_front_ratio.get());

    // Safety checks
    _safety_state.torque_distribution_valid = true;
    _safety_state.stability_ok = true;

    // Check for unreasonable values
    if (!_load_state.load_stable) {
        final_distribution = 0.5f; // Equal distribution if load unstable
        _safety_state.torque_distribution_valid = false;
    }

    // Emergency distribution if critical failure
    if (!_safety_state.torque_distribution_valid) {
        final_distribution = 0.5f;
        _safety_state.emergency_distribution_active = true;
    } else {
        _safety_state.emergency_distribution_active = false;
    }

    _torque_distribution.final_distribution = final_distribution;
    _torque_distribution.front_torque_ratio = final_distribution;
    _torque_distribution.rear_torque_ratio = 1.0f - final_distribution;
}

void LoadAwareTorqueDistribution::optimize_for_efficiency()
{
    // Efficiency optimization favors equal distribution to minimize losses
    float efficiency_target = 0.5f;

    // Adjust towards efficient distribution
    float efficiency_weight = _efficiency_weight.get();
    _torque_distribution.final_distribution =
        (1.0f - efficiency_weight) * _torque_distribution.final_distribution +
        efficiency_weight * efficiency_target;

    // Update efficiency factor
    float distribution_error = fabsf(_torque_distribution.final_distribution - 0.5f);
    _performance.efficiency_factor = 1.0f - distribution_error * 0.1f;
}

void LoadAwareTorqueDistribution::optimize_for_traction()
{
    // Traction optimization based on slip feedback
    float traction_weight = _traction_weight.get();

    // Bias towards axle with better traction
    if (_traction_state.slip_front < _traction_state.slip_rear) {
        // Front has better traction
        _torque_distribution.final_distribution += traction_weight * 0.05f;
    } else if (_traction_state.slip_rear < _traction_state.slip_front) {
        // Rear has better traction
        _torque_distribution.final_distribution -= traction_weight * 0.05f;
    }

    // Calculate traction utilization
    float avg_slip = (_traction_state.slip_front + _traction_state.slip_rear) / 2.0f;
    _performance.traction_utilization = math::constrain(1.0f - avg_slip * 10.0f, 0.0f, 1.0f);
}

void LoadAwareTorqueDistribution::publish_torque_commands()
{
    uint64_t timestamp = hrt_absolute_time();

    // Get total torque request
    float total_torque_request = 0.0f;
    if (_wheel_speeds_sub.updated()) {
        wheel_speeds_setpoint_s speeds;
        _wheel_speeds_sub.copy(&speeds);
        total_torque_request = speeds.max_torque_nm;
    }
    _torque_distribution.total_torque_request_nm = total_torque_request;

    // Calculate individual axle torques based on distribution ratios
    _torque_distribution.front_torque_nm =
        _torque_distribution.total_torque_request_nm * _torque_distribution.front_torque_ratio;
    _torque_distribution.rear_torque_nm =
        _torque_distribution.total_torque_request_nm * _torque_distribution.rear_torque_ratio;

    // Publish load-aware torque output
    _torque_output.timestamp = timestamp;
    _torque_output.base_torque_distribution = _torque_distribution.base_distribution;
    _torque_output.dynamic_torque_adjustment = _torque_distribution.dynamic_adjustment;
    _torque_output.stability_torque_correction = _torque_distribution.stability_correction;
    _torque_output.optimal_front_torque_nm = _torque_distribution.front_torque_nm;
    _torque_output.optimal_rear_torque_nm = _torque_distribution.rear_torque_nm;
    _torque_output.weight_distribution_ratio = _torque_distribution.front_torque_ratio;
    _torque_output.stability_margin = _performance.stability_margin;
    _torque_output.articulation_angle_rad = _dynamics_state.articulation_angle_rad; // Add articulation angle
    _torque_output.redistribution_active =
        fabsf(_torque_distribution.final_distribution - 0.5f) > 0.05f;

    _load_aware_torque_pub.publish(_torque_output);

    // Publish updated wheel speed setpoints with proper torque distribution
    // CRITICAL FIX: Send actual calculated torques to wheel controllers
    wheel_speeds_setpoint_s wheel_speeds{};
    wheel_speeds.timestamp = timestamp;
    wheel_speeds.front_wheel_speed_rpm = 0.0f; // Will be set by speed controller
    wheel_speeds.rear_wheel_speed_rpm = 0.0f;  // Will be set by speed controller

    // FIXED: Send distributed torques instead of total torque
    wheel_speeds.front_max_torque_nm = _torque_distribution.front_torque_nm;
    wheel_speeds.rear_max_torque_nm = _torque_distribution.rear_torque_nm;
    wheel_speeds.max_torque_nm = _torque_distribution.total_torque_request_nm; // Keep total for reference
    wheel_speeds.control_mode = 1; // Torque control mode
    wheel_speeds.torque_distribution_active = true; // Flag that distribution is active

    _wheel_speeds_pub.publish(wheel_speeds);

    // Update module status
    _status.timestamp = timestamp;
    _status.module_id = MODULE_ID_LOAD_AWARE_TORQUE;
    _status.health_status = _safety_state.torque_distribution_valid ?
                           MODULE_HEALTH_OK : MODULE_HEALTH_WARNING;
    _status.operational_status = _safety_state.emergency_distribution_active ?
                                MODULE_OP_EMERGENCY : MODULE_OP_NORMAL;

    _status.data[0] = _torque_distribution.final_distribution;
    _status.data[1] = _torque_distribution.front_torque_nm;
    _status.data[2] = _torque_distribution.rear_torque_nm;
    _status.data[3] = _performance.efficiency_factor;
    _status.data[4] = _performance.traction_utilization;
    _status.data[5] = _performance.stability_margin;

    _module_status_pub.publish(_status);
}

void LoadAwareTorqueDistribution::update_performance_metrics()
{
    // Update performance metrics
    _performance.weight_distribution_error =
        fabsf(_torque_distribution.final_distribution - _torque_distribution.base_distribution);

    _performance.avg_front_slip = 0.95f * _performance.avg_front_slip +
                                 0.05f * _traction_state.slip_front;
    _performance.avg_rear_slip = 0.95f * _performance.avg_rear_slip +
                                0.05f * _traction_state.slip_rear;

    // Calculate stability margin
    float max_slip = fmaxf(fabsf(_traction_state.slip_front), fabsf(_traction_state.slip_rear));
    _performance.stability_margin = math::constrain(1.0f - max_slip * 5.0f, 0.0f, 1.0f);

    // Count distribution changes
    static float last_distribution = 0.5f;
    if (fabsf(_torque_distribution.final_distribution - last_distribution) > 0.01f) {
        _performance.distribution_changes++;
        last_distribution = _torque_distribution.final_distribution;
    }

    // Calculate power distribution efficiency
    float distribution_deviation = fabsf(_torque_distribution.final_distribution - 0.5f);
    _performance.power_distribution_efficiency = 1.0f - distribution_deviation * 0.2f;
}

int LoadAwareTorqueDistribution::print_status()
{
    PX4_INFO("Load-Aware Torque Distribution Status:");
    PX4_INFO("  Payload Mass: %.0f kg", (double)_load_state.payload_mass_kg);
    PX4_INFO("  Total Mass: %.0f kg", (double)_load_state.total_mass_kg);
    PX4_INFO("  COG Position: X=%.2f m, Z=%.2f m",
             (double)_load_state.cog_x_m, (double)_load_state.cog_z_m);
    PX4_INFO("  Articulation Angle: %.2f rad (%.1f degrees)",
             (double)_dynamics_state.articulation_angle_rad,
             (double)math::degrees(_dynamics_state.articulation_angle_rad));
    PX4_INFO("  Weight Distribution:");
    PX4_INFO("    Front Axle: %.0f kg", (double)_load_state.front_axle_load_kg);
    PX4_INFO("    Rear Axle: %.0f kg", (double)_load_state.rear_axle_load_kg);
    PX4_INFO("  Torque Distribution:");
    PX4_INFO("    Base Distribution: %.3f", (double)_torque_distribution.base_distribution);
    PX4_INFO("    Dynamic Adjustment: %.3f", (double)_torque_distribution.dynamic_adjustment);
    PX4_INFO("    Stability Correction: %.3f", (double)_torque_distribution.stability_correction);
    PX4_INFO("    Terrain Adaptation: %.3f", (double)_torque_distribution.terrain_adaptation);
    PX4_INFO("    Final Distribution: %.3f", (double)_torque_distribution.final_distribution);
    PX4_INFO("  Torque Commands:");
    PX4_INFO("    Front Torque: %.1f Nm", (double)_torque_distribution.front_torque_nm);
    PX4_INFO("    Rear Torque: %.1f Nm", (double)_torque_distribution.rear_torque_nm);
    PX4_INFO("  Performance:");
    PX4_INFO("    Efficiency Factor: %.3f", (double)_performance.efficiency_factor);
    PX4_INFO("    Traction Utilization: %.3f", (double)_performance.traction_utilization);
    PX4_INFO("    Stability Margin: %.3f", (double)_performance.stability_margin);
    PX4_INFO("    Distribution Changes: %u", _performance.distribution_changes);
    PX4_INFO("    Stability Interventions: %u", _performance.stability_interventions);
    PX4_INFO("  Safety:");
    PX4_INFO("    Distribution Valid: %s", _safety_state.torque_distribution_valid ? "YES" : "NO");
    PX4_INFO("    Load Stable: %s", _load_state.load_stable ? "YES" : "NO");
    PX4_INFO("    Emergency Mode: %s", _safety_state.emergency_distribution_active ? "ACTIVE" : "INACTIVE");

    return 0;
}

int LoadAwareTorqueDistribution::custom_command(int argc, char *argv[])
{
    if (!is_running()) {
        PX4_ERR("Module not running");
        return -1;
    }

    if (!strcmp(argv[0], "status")) {
        return get_instance()->print_status();
    }

    return print_usage("unknown command");
}

int LoadAwareTorqueDistribution::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Load-Aware Torque Distribution for articulated wheel loader.

Dynamically distributes torque between front and rear axles based on payload mass,
center of gravity, dynamic weight transfer, and terrain conditions.

### Examples
CLI usage example:
$ load_aware_torque start
$ load_aware_torque status
$ load_aware_torque stop
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("load_aware_torque", "controller");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print status information");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

extern "C" __EXPORT int load_aware_torque_main(int argc, char *argv[])
{
    return LoadAwareTorqueDistribution::main(argc, argv);
}
