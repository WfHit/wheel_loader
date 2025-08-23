#include "SlipEstimatorModule.hpp"
#include <px4_platform_common/log.h>
#include <mathlib/mathlib.h>

namespace traction_control {

SlipEstimatorModule::SlipEstimatorModule()
    : ParameterizedTractionModule("SlipEstimator", "2.0.0")
{
}

bool SlipEstimatorModule::doInitialize()
{
    // Load parameters
    loadParameters();

    // Initialize EKF
    _slip_ekf.initialize(
        _params.wheel_radius_m,
        _params.wheelbase_m,
        _params.process_noise,
        _params.measurement_noise
    );

    // Initialize friction estimator
    _friction_estimator.initialize(_params.friction_init);

    // Reset surface detector
    _surface_detector.reset();

    // Initialize state
    _estimator_state = {};
    _performance = {};

    logActivity(6, "Slip estimator initialized with method %d", _params.estimation_method);
    return true;
}

bool SlipEstimatorModule::doProcess(const TractionState& state, TractionCommand& command)
{
    // Update sensor data
    if (!updateSensorData()) {
        logActivity(5, "Failed to update sensor data");
        return false;
    }

    // Run slip estimation based on selected method
    if (_params.estimation_method == 1) {
        // Use EKF method
        runSlipEstimation();
    } else {
        // Use basic method (fallback)
        // Implementation for basic slip calculation
        for (int i = 0; i < 4; i++) {
            _estimator_state.slip_ratios[i] = calculateSlipRatio(
                _estimator_state.wheel_speeds[i],
                _estimator_state.vehicle_velocity_x,
                _params.wheel_radius_m
            );
        }
    }

    // Update friction estimate
    updateFrictionEstimate();

    // Detect surface changes
    detectSurfaceChanges();

    // Validate estimation
    if (!validateEstimation()) {
        _performance.failed_estimations++;
        return false;
    }

    // Publish results
    publishSlipEstimation(command);

    _performance.successful_estimations++;
    _estimator_state.last_update_time = hrt_absolute_time();

    return true;
}

void SlipEstimatorModule::doReset()
{
    _slip_ekf = SlipEKF();
    _friction_estimator = FrictionEstimator();
    _surface_detector.reset();
    _estimator_state = {};
    _performance = {};
}

void SlipEstimatorModule::loadParameters()
{
    // Load parameters from PX4 parameter system
    // This is a placeholder - actual implementation would load from DEFINE_PARAMETERS
    _params.wheel_radius_m = 0.4f;
    _params.wheelbase_m = 2.5f;
    _params.track_width_m = 1.8f;
    _params.process_noise = 0.01f;
    _params.measurement_noise = 0.1f;
    _params.slip_threshold = 0.05f;
    _params.friction_init = 0.6f;
    _params.estimation_method = 1;
    _params.front_encoder_idx = 0;
    _params.rear_encoder_idx = 1;
    _params.update_hz = 50;
    _params.enable_learning = true;
    _params.confidence_threshold = 0.7f;
}

bool SlipEstimatorModule::updateSensorData()
{
    bool data_updated = false;

    // Update wheel speeds
    if (updateWheelSpeeds()) {
        data_updated = true;
    }

    // Update vehicle dynamics
    if (updateVehicleDynamics()) {
        data_updated = true;
    }

    return data_updated;
}

bool SlipEstimatorModule::updateWheelSpeeds()
{
    sensor_quad_encoder_s encoder_data;
    bool updated = false;

    // Check for updated encoder data
    if (_encoder_sub.update(&encoder_data)) {
        // Convert encoder data to wheel speeds
        // This is simplified - actual implementation would handle multiple encoders
        _estimator_state.wheel_speeds[0] = encoder_data.speed_rad_s; // Front left
        _estimator_state.wheel_speeds[1] = encoder_data.speed_rad_s; // Front right
        _estimator_state.wheel_speeds[2] = encoder_data.speed_rad_s; // Rear left
        _estimator_state.wheel_speeds[3] = encoder_data.speed_rad_s; // Rear right
        updated = true;
    }

    return updated;
}

bool SlipEstimatorModule::updateVehicleDynamics()
{
    vehicle_attitude_s attitude;
    vehicle_local_position_s position;
    bool updated = false;

    // Update attitude
    if (_attitude_sub.update(&attitude)) {
        // Extract yaw rate from attitude
        _estimator_state.yaw_rate = attitude.rollspeed; // Simplified
        updated = true;
    }

    // Update position/velocity
    if (_local_position_sub.update(&position)) {
        _estimator_state.vehicle_velocity_x = position.vx;
        _estimator_state.vehicle_velocity_y = position.vy;
        updated = true;
    }

    return updated;
}

void SlipEstimatorModule::runSlipEstimation()
{
    static uint64_t last_update_time = 0;
    uint64_t current_time = hrt_absolute_time();

    if (last_update_time == 0) {
        last_update_time = current_time;
        return;
    }

    float dt = (current_time - last_update_time) / 1e6f; // Convert to seconds

    // EKF prediction step
    _slip_ekf.predict(dt);

    // EKF update with wheel speeds
    _slip_ekf.updateWheelSpeeds(_estimator_state.wheel_speeds);

    // EKF update with vehicle dynamics
    _slip_ekf.updateVehicleDynamics(
        _estimator_state.vehicle_velocity_x,
        _estimator_state.vehicle_velocity_y,
        _estimator_state.yaw_rate
    );

    // Extract results
    _estimator_state.slip_ratios[0] = _slip_ekf.getSlipFront();
    _estimator_state.slip_ratios[1] = _slip_ekf.getSlipFront();
    _estimator_state.slip_ratios[2] = _slip_ekf.getSlipRear();
    _estimator_state.slip_ratios[3] = _slip_ekf.getSlipRear();
    _estimator_state.friction_estimate = _slip_ekf.getFrictionEstimate();
    _estimator_state.estimation_confidence = _slip_ekf.getConfidence();

    last_update_time = current_time;
}

void SlipEstimatorModule::updateFrictionEstimate()
{
    if (_params.enable_learning) {
        // Update friction estimator with slip data
        float avg_slip = 0.0f;
        for (int i = 0; i < 4; i++) {
            avg_slip += fabsf(_estimator_state.slip_ratios[i]);
        }
        avg_slip /= 4.0f;

        // Simplified normal force calculation
        float normal_force = 1.0f; // Placeholder

        _friction_estimator.update(avg_slip, normal_force);

        // Blend EKF and RLS friction estimates
        float ekf_friction = _slip_ekf.getFrictionEstimate();
        float rls_friction = _friction_estimator.getFriction();
        float rls_confidence = _friction_estimator.getConfidence();

        _estimator_state.friction_estimate =
            ekf_friction * (1.0f - rls_confidence) + rls_friction * rls_confidence;
    }
}

void SlipEstimatorModule::detectSurfaceChanges()
{
    // Calculate slip variance
    float slip_mean = 0.0f;
    for (int i = 0; i < 4; i++) {
        slip_mean += _estimator_state.slip_ratios[i];
    }
    slip_mean /= 4.0f;

    float slip_variance = 0.0f;
    for (int i = 0; i < 4; i++) {
        float diff = _estimator_state.slip_ratios[i] - slip_mean;
        slip_variance += diff * diff;
    }
    slip_variance /= 4.0f;

    _surface_detector.update(_estimator_state.friction_estimate, slip_variance);

    if (_surface_detector.hasChanged()) {
        logActivity(6, "Surface change detected with confidence %.2f",
                   _surface_detector.getChangeConfidence());
        _performance.last_surface_change = hrt_absolute_time();
    }
}

void SlipEstimatorModule::publishSlipEstimation(TractionCommand& command)
{
    // Set slip-related parameters in the command
    command.parameters.target_slip_ratio = 0.1f; // Optimal slip for maximum traction

    // Adjust intervention level based on slip
    float max_slip = 0.0f;
    for (int i = 0; i < 4; i++) {
        max_slip = std::max(max_slip, fabsf(_estimator_state.slip_ratios[i]));
    }

    if (max_slip > _params.slip_threshold * 2.0f) {
        command.parameters.intervention_level = 3; // Aggressive
    } else if (max_slip > _params.slip_threshold) {
        command.parameters.intervention_level = 2; // Moderate
    } else {
        command.parameters.intervention_level = 1; // Mild
    }

    // Set status flags
    command.status.slip_detected = max_slip > _params.slip_threshold;
    command.status.intervention_active = command.status.slip_detected;

    // Update confidence
    _performance.avg_confidence =
        (_performance.avg_confidence * _performance.successful_estimations +
         _estimator_state.estimation_confidence) /
        (_performance.successful_estimations + 1);
}

float SlipEstimatorModule::calculateSlipRatio(float wheel_speed, float vehicle_speed, float wheel_radius) const
{
    float wheel_velocity = wheel_speed * wheel_radius;

    if (fabsf(vehicle_speed) < 0.1f) {
        return 0.0f; // Avoid division by zero at low speeds
    }

    return (wheel_velocity - vehicle_speed) / fabsf(vehicle_speed);
}

float SlipEstimatorModule::calculateLateralSlip(float lateral_velocity, float longitudinal_velocity) const
{
    if (fabsf(longitudinal_velocity) < 0.1f) {
        return 0.0f;
    }

    return atanf(lateral_velocity / longitudinal_velocity);
}

bool SlipEstimatorModule::validateEstimation() const
{
    // Check data validity
    if (!_estimator_state.data_valid) {
        return false;
    }

    // Check confidence threshold
    if (_estimator_state.estimation_confidence < _params.confidence_threshold) {
        return false;
    }

    // Check for reasonable slip values
    for (int i = 0; i < 4; i++) {
        if (fabsf(_estimator_state.slip_ratios[i]) > 1.0f) {
            return false; // Unrealistic slip ratio
        }
    }

    // Check friction estimate
    if (_estimator_state.friction_estimate < 0.1f || _estimator_state.friction_estimate > 1.5f) {
        return false; // Unrealistic friction coefficient
    }

    return true;
}

// SlipEKF implementation

SlipEstimatorModule::SlipEKF::SlipEKF()
{
    _state.setZero();
    _P.setIdentity();
    _Q.setIdentity();
    _R_wheels.setIdentity();
    _R_dynamics.setIdentity();
}

void SlipEstimatorModule::SlipEKF::initialize(float wheel_radius, float wheelbase,
                                            float process_noise, float measurement_noise)
{
    _wheel_radius = wheel_radius;
    _wheelbase = wheelbase;

    // Initialize state vector: [vx, vy, slip_front, slip_rear, friction]
    _state(0) = 0.0f; // vx
    _state(1) = 0.0f; // vy
    _state(2) = 0.0f; // slip_front
    _state(3) = 0.0f; // slip_rear
    _state(4) = 0.6f; // friction

    // Initialize covariance matrix
    _P.setIdentity();
    _P *= 0.1f;

    // Process noise
    _Q.setIdentity();
    _Q *= process_noise;

    // Measurement noise
    _R_wheels.setIdentity();
    _R_wheels *= measurement_noise;

    _R_dynamics.setIdentity();
    _R_dynamics *= measurement_noise * 0.5f;

    _initialized = true;
    _confidence = 0.5f;
}

void SlipEstimatorModule::SlipEKF::predict(float dt)
{
    if (!_initialized) return;

    // Simple prediction model - in practice this would be more sophisticated
    // State propagation (simplified)
    // _state remains mostly the same, with some process noise

    // Update covariance
    updateCovariance(dt);

    // Update confidence based on prediction uncertainty
    float trace_P = _P.trace();
    _confidence = 1.0f / (1.0f + trace_P);
}

void SlipEstimatorModule::SlipEKF::updateWheelSpeeds(const float wheel_speeds[4])
{
    if (!_initialized) return;

    // Measurement vector: [wheel_speed_front_avg, wheel_speed_rear_avg, lateral_accel, friction_indicator]
    matrix::Vector<float, 4> z;
    z(0) = (wheel_speeds[0] + wheel_speeds[1]) / 2.0f; // Front average
    z(1) = (wheel_speeds[2] + wheel_speeds[3]) / 2.0f; // Rear average
    z(2) = 0.0f; // Placeholder for lateral acceleration
    z(3) = 0.6f; // Placeholder for friction indicator

    // Predicted measurement
    matrix::Vector<float, 4> h;
    h(0) = _state(0) / _wheel_radius * (1.0f + _state(2)); // Front wheel speed
    h(1) = _state(0) / _wheel_radius * (1.0f + _state(3)); // Rear wheel speed
    h(2) = _state(1); // Lateral velocity
    h(3) = _state(4); // Friction estimate

    // Innovation
    matrix::Vector<float, 4> y = z - h;

    // Measurement Jacobian
    matrix::Matrix<float, 4, 5> H = getWheelMeasurementJacobian();

    // Innovation covariance
    matrix::SquareMatrix<float, 4> S = H * _P * H.transpose() + _R_wheels;

    // Kalman gain
    matrix::Matrix<float, 5, 4> K = _P * H.transpose() * S.I();

    // State update
    _state = _state + K * y;

    // Covariance update
    matrix::SquareMatrix<float, 5> I;
    I.setIdentity();
    _P = (I - K * H) * _P;

    // Update confidence
    float innovation_norm = y.norm();
    _confidence = expf(-innovation_norm * 0.5f);
}

void SlipEstimatorModule::SlipEKF::updateVehicleDynamics(float vx, float vy, float yaw_rate)
{
    if (!_initialized) return;

    // Measurement vector: [vx, vy, yaw_rate]
    matrix::Vector<float, 3> z;
    z(0) = vx;
    z(1) = vy;
    z(2) = yaw_rate;

    // Predicted measurement
    matrix::Vector<float, 3> h;
    h(0) = _state(0); // vx
    h(1) = _state(1); // vy
    h(2) = _state(1) / _wheelbase; // Approximate yaw rate

    // Innovation
    matrix::Vector<float, 3> y = z - h;

    // Measurement Jacobian
    matrix::Matrix<float, 3, 5> H = getDynamicsMeasurementJacobian();

    // Innovation covariance
    matrix::SquareMatrix<float, 3> S = H * _P * H.transpose() + _R_dynamics;

    // Kalman gain
    matrix::Matrix<float, 5, 3> K = _P * H.transpose() * S.I();

    // State update
    _state = _state + K * y;

    // Covariance update
    matrix::SquareMatrix<float, 5> I;
    I.setIdentity();
    _P = (I - K * H) * _P;
}

void SlipEstimatorModule::SlipEKF::updateCovariance(float dt)
{
    // Simple covariance propagation
    _P = _P + _Q * dt;
}

matrix::Matrix<float, 4, 5> SlipEstimatorModule::SlipEKF::getWheelMeasurementJacobian() const
{
    matrix::Matrix<float, 4, 5> H;
    H.setZero();

    // Partial derivatives of measurement function with respect to state
    H(0, 0) = 1.0f / _wheel_radius * (1.0f + _state(2)); // dh1/dvx
    H(0, 2) = _state(0) / _wheel_radius;                  // dh1/dslip_front
    H(1, 0) = 1.0f / _wheel_radius * (1.0f + _state(3)); // dh2/dvx
    H(1, 3) = _state(0) / _wheel_radius;                  // dh2/dslip_rear
    H(2, 1) = 1.0f;                                       // dh3/dvy
    H(3, 4) = 1.0f;                                       // dh4/dfriction

    return H;
}

matrix::Matrix<float, 3, 5> SlipEstimatorModule::SlipEKF::getDynamicsMeasurementJacobian() const
{
    matrix::Matrix<float, 3, 5> H;
    H.setZero();

    H(0, 0) = 1.0f;                    // dvx/dvx
    H(1, 1) = 1.0f;                    // dvy/dvy
    H(2, 1) = 1.0f / _wheelbase;       // dyaw_rate/dvy

    return H;
}

// FrictionEstimator implementation

SlipEstimatorModule::FrictionEstimator::FrictionEstimator()
{
    _theta.setZero();
    _P_rls.setIdentity();
    _P_rls *= 1000.0f; // Large initial uncertainty
}

void SlipEstimatorModule::FrictionEstimator::initialize(float initial_friction)
{
    _friction = initial_friction;
    _theta(0) = initial_friction;
    _theta(1) = 0.0f;
    _confidence = 0.1f;
    _sample_count = 0;
}

void SlipEstimatorModule::FrictionEstimator::update(float slip, float normalized_force)
{
    // Simple tire model: force = friction * normal_force * slip_function(slip)
    matrix::Vector<float, 2> phi;
    phi(0) = normalized_force * slip;
    phi(1) = normalized_force * slip * slip;

    // RLS update
    matrix::Vector<float, 2> K = _P_rls * phi / (_forgetting_factor + phi.transpose() * _P_rls * phi);

    float force_estimate = phi.transpose() * _theta;
    float error = normalized_force - force_estimate;

    _theta = _theta + K * error;
    _P_rls = (_P_rls - K * phi.transpose() * _P_rls) / _forgetting_factor;

    // Update friction estimate
    _friction = _theta(0);

    // Update confidence
    _sample_count++;
    _confidence = std::min(1.0f, _sample_count / 100.0f);
}

// SurfaceChangeDetector implementation

void SlipEstimatorModule::SurfaceChangeDetector::update(float friction, float slip_variance)
{
    _friction_history[_history_index] = friction;
    _slip_variance_history[_history_index] = slip_variance;
    _history_index = (_history_index + 1) % HISTORY_SIZE;

    // Simple change detection based on variance in friction estimate
    float friction_variance = calculateVariance(_friction_history, HISTORY_SIZE);
    float slip_var_variance = calculateVariance(_slip_variance_history, HISTORY_SIZE);

    // Threshold-based detection
    const float FRICTION_CHANGE_THRESHOLD = 0.1f;
    const float SLIP_VAR_CHANGE_THRESHOLD = 0.05f;

    _surface_changed = (friction_variance > FRICTION_CHANGE_THRESHOLD) ||
                      (slip_var_variance > SLIP_VAR_CHANGE_THRESHOLD);

    if (_surface_changed) {
        _change_confidence = std::min(1.0f, friction_variance + slip_var_variance);
    } else {
        _change_confidence = 0.0f;
    }
}

bool SlipEstimatorModule::SurfaceChangeDetector::hasChanged() const
{
    return _surface_changed;
}

float SlipEstimatorModule::SurfaceChangeDetector::getChangeConfidence() const
{
    return _change_confidence;
}

void SlipEstimatorModule::SurfaceChangeDetector::reset()
{
    for (int i = 0; i < HISTORY_SIZE; i++) {
        _friction_history[i] = 0.6f;
        _slip_variance_history[i] = 0.0f;
    }
    _history_index = 0;
    _surface_changed = false;
    _change_confidence = 0.0f;
}

float SlipEstimatorModule::SurfaceChangeDetector::calculateVariance(const float* data, int size) const
{
    float mean = 0.0f;
    for (int i = 0; i < size; i++) {
        mean += data[i];
    }
    mean /= size;

    float variance = 0.0f;
    for (int i = 0; i < size; i++) {
        float diff = data[i] - mean;
        variance += diff * diff;
    }
    return variance / size;
}

} // namespace traction_control
