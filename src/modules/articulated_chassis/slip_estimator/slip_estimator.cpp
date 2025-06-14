#include "slip_estimator.hpp"
#include <px4_platform_common/log.h>

SlipEstimator::SlipEstimator() :
    ModuleParams(nullptr)
{
}

bool SlipEstimator::init()
{
    // Initialize the slip estimation module
    return true;
}

void SlipEstimator::run()
{
    // Initialize estimators with parameter values
    _ekf.init(_param_wheel_radius.get(), _param_wheelbase.get(),
              _param_process_noise.get(), _param_measurement_noise.get(),
              _param_friction_init.get());

    // Initialize friction estimator
    _friction_estimator.init(_param_friction_init.get());

    while (!should_exit()) {
        updateVehicleState();

        slip_estimation_s slip{};
        slip.timestamp = hrt_absolute_time();

        // Run estimation based on selected method
        switch (_param_estimation_method.get()) {
            case 0:
                runBasicEstimation(slip);
                break;
            case 1:
                runEKFEstimation(slip);
                break;
            default:
                runEKFEstimation(slip); // Default to EKF
        }

        // Detect surface changes
        detectSurfaceChange(slip);

        // Publish results
        _slip_estimation_pub.publish(slip);

        // Use parameter-based update frequency
        int update_hz = _param_update_hz.get();
        px4_usleep(1000000 / update_hz); // Convert Hz to microseconds
    }
}

void SlipEstimator::updateVehicleState()
{
    // Update wheel speeds from sensor quad encoder (supports multiple encoders)
    // Get encoder indices from parameters
    const int front_idx = _param_front_encoder_idx.get();
    const int rear_idx = _param_rear_encoder_idx.get();

    // Check all available encoder instances
    for (uint8_t instance = 0; instance < _sensor_quad_encoder_sub.size(); instance++) {
        sensor_quad_encoder_s encoders;
        if (_sensor_quad_encoder_sub[instance].copy(&encoders)) {
            // Validate encoder indices and count
            if (encoders.count > front_idx && encoders.count > rear_idx) {
                // Front wheel encoder
                if (front_idx >= 0 && front_idx < encoders.count && encoders.valid[front_idx]) {
                    // Convert rad/s to linear speed (m/s) using wheel radius parameter
                    _vehicle_state.wheel_speed_front = encoders.velocity[front_idx] * _param_wheel_radius.get();
                }

                // Rear wheel encoder
                if (rear_idx >= 0 && rear_idx < encoders.count && encoders.valid[rear_idx]) {
                    // Convert rad/s to linear speed (m/s) using wheel radius parameter
                    _vehicle_state.wheel_speed_rear = encoders.velocity[rear_idx] * _param_wheel_radius.get();
                }

                _vehicle_state.last_update = encoders.timestamp;
                break; // Use first valid instance
            } else {
                PX4_WARN("SlipEstimator: Encoder indices (%d, %d) exceed available encoders (%d) in instance %d",
                         front_idx, rear_idx, encoders.count, instance);
            }
        }
    }

    // Update steering
    steering_status_s steering;
    if (_steering_status_sub.copy(&steering)) {
        _vehicle_state.steering_angle = math::radians(steering.steering_angle_deg);
    }

    _vehicle_state.last_update = hrt_absolute_time();

    // Update EKF-based vehicle state from messages
    updateFromEKFMessages();
}

void SlipEstimator::updateFromEKFMessages()
{
    // Get latest EKF results from messages instead of direct library access
    vehicle_attitude_s attitude;
    vehicle_local_position_s local_pos;
    estimator_states_s states;

    bool has_attitude = _vehicle_attitude_sub.copy(&attitude);
    bool has_position = _vehicle_local_position_sub.copy(&local_pos);
    bool has_states = _estimator_states_sub.copy(&states);

    if (has_attitude && has_position) {
        // Update vehicle velocity and acceleration from EKF results
        if (local_pos.v_xy_valid) {
            _vehicle_state.speed_x = local_pos.vx;
            _vehicle_state.speed_y = local_pos.vy;
        }

        // Update acceleration from EKF results (better than raw sensor data)
        _vehicle_state.accel_x = local_pos.ax;
        _vehicle_state.accel_y = local_pos.ay;

        // Get yaw rate from attitude quaternion time derivative
        // This replaces direct EKF library access
        matrix::Quatf q(attitude.q);
        matrix::Vector3f euler = matrix::Eulerf(q);
        _vehicle_state.yaw_rate = euler(2); // yaw component

        // Update EKF state in slip estimator
        if (has_states) {
            _ekf.updateFromEKFMessages(attitude, local_pos, states);
        }
    }
}

void SlipEstimator::runEKFEstimation(slip_estimation_s &slip)
{
    float dt = (_vehicle_state.last_update - slip.timestamp) * 1e-6f;
    if (dt <= 0 || dt > 1.0f) dt = 0.01f;

    // Predict step (now simplified since velocity comes from EKF messages)
    _ekf.predict(dt);

    // Update with wheel speeds (this calculates slip using EKF velocity as reference)
    _ekf.updateWheelSpeed(_vehicle_state.wheel_speed_front,
                          _vehicle_state.wheel_speed_rear);

    // Update with vehicle state information (replaces direct IMU/GPS updates)
    _ekf.updateVehicleState(_vehicle_state.accel_x,
                            _vehicle_state.accel_y,
                            _vehicle_state.yaw_rate);

    // Extract estimates
    slip.slip_ratio_front = _ekf.getSlipFront();
    slip.slip_ratio_rear = _ekf.getSlipRear();
    slip.slip_angle_front_rad = _ekf.getLateralSlip();
    slip.friction_coefficient = _ekf.getFrictionEstimate();

    // Use EKF velocity estimate (from messages) as reference speed
    slip.longitudinal_slip_velocity = sqrtf(_vehicle_state.speed_x * _vehicle_state.speed_x +
                                        _vehicle_state.speed_y * _vehicle_state.speed_y);
    slip.estimator_converged = true; // High confidence for EKF
    slip.terrain_type = 1;

    // Update friction estimator with EKF results
    float avg_slip = (fabsf(slip.slip_ratio_front) +
                      fabsf(slip.slip_ratio_rear)) / 2.0f;
    float normalized_force = _vehicle_state.accel_x / 9.81f;
    _friction_estimator.update(avg_slip, normalized_force, _param_slip_threshold.get());
}

void SlipEstimator::runBasicEstimation(slip_estimation_s &slip)
{
    // Simple slip calculation
    float vehicle_speed = sqrtf(_vehicle_state.speed_x * _vehicle_state.speed_x +
                               _vehicle_state.speed_y * _vehicle_state.speed_y);

    if (vehicle_speed > 0.5f) {
        slip.slip_ratio_front = (_vehicle_state.wheel_speed_front - vehicle_speed) / vehicle_speed;
        slip.slip_ratio_rear = (_vehicle_state.wheel_speed_rear - vehicle_speed) / vehicle_speed;
    } else {
        slip.slip_ratio_front = 0.0f;
        slip.slip_ratio_rear = 0.0f;
    }

    slip.slip_angle_front_rad = 0.0f; // Basic estimation doesn't calculate lateral slip
    slip.friction_coefficient = _param_friction_init.get(); // Use parameter value
    slip.estimator_converged = false; // Lower confidence for basic method
    slip.terrain_type = 0;
    slip.longitudinal_slip_velocity = vehicle_speed;
}

// SlipEKF Implementation - Updated to use EKF message data
SlipEstimator::SlipEKF::SlipEKF()
{
    _x.setZero();
    // Initial friction estimate will be set in init()
    _P.setIdentity();
    _P *= 0.1f;
    // Process and measurement noise will be set in init()
    _Q.setIdentity();
    _R_wheel.setIdentity();
    _R_vehicle.setIdentity();
}

void SlipEstimator::SlipEKF::init(float wheel_radius, float wheelbase, float process_noise, float measurement_noise, float friction_init)
{
    _wheel_radius = wheel_radius;
    _wheelbase = wheelbase;

    // Set initial friction estimate
    _x(4) = friction_init;

    // Set noise matrices using parameters
    _Q *= process_noise;
    _R_wheel *= measurement_noise;
    _R_vehicle *= (measurement_noise * 0.5f); // Vehicle measurements typically more accurate
}

void SlipEstimator::SlipEKF::updateFromEKFMessages(const vehicle_attitude_s &attitude,
                                                    const vehicle_local_position_s &position,
                                                    const estimator_states_s &states)
{
    // Extract EKF state information from messages instead of direct library access
    _ekf_state.valid = position.v_xy_valid && position.xy_valid;

    if (_ekf_state.valid) {
        // Get velocity from EKF local position message
        _ekf_state.vx = position.vx;
        _ekf_state.vy = position.vy;
        _ekf_state.vz = position.vz;

        // Get acceleration from local position message
        _ekf_state.ax = position.ax;
        _ekf_state.ay = position.ay;

        // Get quaternion from attitude message
        _ekf_state.q = matrix::Quatf(attitude.q);

        // Calculate yaw rate from quaternion (simplified)
        matrix::Vector3f euler = matrix::Eulerf(_ekf_state.q);
        _ekf_state.yaw_rate = euler(2); // yaw component

        // Use EKF velocity estimates directly for slip calculation
        // This replaces the internal EKF prediction step
        _x(0) = _ekf_state.vx; // Use EKF velocity estimate
        _x(1) = _ekf_state.vy; // Use EKF velocity estimate
    }
}

void SlipEstimator::SlipEKF::predict(float dt)
{
    // Simplified prediction using EKF message data
    // The main velocity estimation is now done by the EKF2 module
    // We only need to predict slip states

    if (!_ekf_state.valid) {
        return;
    }

    // Simple slip state prediction (slip changes slowly)
    matrix::Vector<float, 5> x_dot;
    x_dot(0) = 0; // vx - now from EKF messages
    x_dot(1) = 0; // vy - now from EKF messages
    x_dot(2) = 0; // slip_front_dot (assumed slowly varying)
    x_dot(3) = 0; // slip_rear_dot (assumed slowly varying)
    x_dot(4) = 0; // friction_dot (assumed slowly varying)

    // Only predict slip and friction states
    _x.slice<3, 1>(2, 0) += x_dot.slice<3, 1>(2, 0) * dt;

    // Simplified covariance prediction for slip states only
    matrix::SquareMatrix<float, 5> F;
    F.setIdentity();

    // Covariance prediction
    _P = F * _P * F.transpose() + _Q;
}

void SlipEstimator::SlipEKF::updateWheelSpeed(float front_speed, float rear_speed)
{
    if (!_ekf_state.valid) {
        return;
    }

    // Use EKF velocity estimate as reference speed
    float vehicle_speed = sqrtf(_ekf_state.vx * _ekf_state.vx + _ekf_state.vy * _ekf_state.vy);

    if (vehicle_speed < 0.1f) {
        return; // Avoid division by small numbers
    }

    // Calculate slip directly using EKF velocity as reference
    float front_slip = (front_speed - vehicle_speed) / vehicle_speed;
    float rear_slip = (rear_speed - vehicle_speed) / vehicle_speed;

    // Measurement model: compare calculated slip with current estimates
    matrix::Vector<float, 2> z;
    z(0) = front_slip;
    z(1) = rear_slip;

    matrix::Vector<float, 2> h;
    h(0) = _x(2); // estimated front slip
    h(1) = _x(3); // estimated rear slip

    // Measurement Jacobian
    matrix::Matrix<float, 2, 5> H;
    H.setZero();
    H(0, 2) = 1.0f; // d(h_front)/d(slip_front)
    H(1, 3) = 1.0f; // d(h_rear)/d(slip_rear)

    // Kalman gain
    matrix::SquareMatrix<float, 2> S = H * _P * H.transpose() + _R_wheel.slice<2, 2>(0, 0);
    matrix::Matrix<float, 5, 2> K = _P * H.transpose() * S.I();

    // State update
    _x += K * (z - h);

    // Covariance update
    matrix::SquareMatrix<float, 5> I;
    I.setIdentity();
    _P = (I - K * H) * _P;
}

void SlipEstimator::SlipEKF::updateVehicleState(float ax, float ay, float yaw_rate)
{
    if (!_ekf_state.valid) {
        return;
    }

    // Update vehicle states using EKF acceleration estimates
    // This replaces the previous updateIMU method
    matrix::Vector<float, 3> z;
    z(0) = ax;
    z(1) = ay;
    z(2) = yaw_rate;

    matrix::Vector<float, 3> h;
    h(0) = _ekf_state.ax; // Use EKF acceleration estimate
    h(1) = _ekf_state.ay; // Use EKF acceleration estimate
    h(2) = _ekf_state.yaw_rate; // Use EKF yaw rate estimate

    // Simple update for friction coefficient based on acceleration consistency
    float ax_innovation = z(0) - h(0);
    float ay_innovation = z(1) - h(1);

    // Update friction estimate if there's significant acceleration difference
    float accel_error = sqrtf(ax_innovation * ax_innovation + ay_innovation * ay_innovation);
    if (accel_error > 0.5f) {
        // Reduce friction estimate if acceleration doesn't match expectation
        _x(4) = math::max(0.1f, _x(4) - 0.01f * accel_error);
    }
}

float SlipEstimator::SlipEKF::getSlipFront() const
{
    return math::constrain(_x(2), -1.0f, 1.0f);
}

float SlipEstimator::SlipEKF::getSlipRear() const
{
    return math::constrain(_x(3), -1.0f, 1.0f);
}

float SlipEstimator::SlipEKF::getLateralSlip() const
{
    if (fabsf(_x(0)) > 0.5f) {
        return atanf(_x(1) / _x(0));
    }
    return 0.0f;
}

float SlipEstimator::SlipEKF::getFrictionEstimate() const
{
    return math::constrain(_x(4), 0.1f, 1.5f);
}

// Friction Estimator Implementation
SlipEstimator::FrictionEstimator::FrictionEstimator()
{
    _theta.setZero();
    // Initial friction estimate will be set in init()
    _P.setIdentity();
    _P *= 100.0f;
}

void SlipEstimator::FrictionEstimator::init(float friction_init)
{
    _mu = friction_init;
    _theta(0, 0) = friction_init;
    _theta(1, 0) = 0.0f;  // Initialize second parameter
}

void SlipEstimator::FrictionEstimator::update(float slip, float normalized_force, float slip_threshold)
{
    // RLS update for friction coefficient estimation
    if (fabsf(slip) < slip_threshold) return; // Use parameter-based threshold

    matrix::Matrix<float, 2, 1> phi;
    phi(0, 0) = slip;
    phi(1, 0) = slip * slip; // Nonlinear term

    float y = normalized_force;
    float y_hat = (phi.transpose() * _theta)(0, 0);
    float error = y - y_hat;

    // RLS update
    float denom = _lambda + (phi.transpose() * _P * phi)(0, 0);
    if (fabsf(denom) > 1e-6f) {
        matrix::Matrix<float, 2, 1> k = _P * phi / denom;
        _theta += k * error;
        _P = (_P - (k * (phi.transpose() * _P))) / _lambda;
    }

    // Extract friction coefficient
    _mu = math::constrain(_theta(0, 0), 0.1f, 1.5f);

    // Update confidence based on error magnitude
    _confidence = 1.0f / (1.0f + fabsf(error));
}

void SlipEstimator::detectSurfaceChange(slip_estimation_s &slip)
{
    static float last_friction = 0.8f;
    static uint64_t last_change_time = 0;

    float friction_change = fabsf(slip.friction_coefficient - last_friction);

    // Detect significant friction change
    if (friction_change > 0.2f) {
        uint64_t now = hrt_absolute_time();
        if (now - last_change_time > 1000000) { // 1 second debounce
            PX4_INFO("Surface change detected: friction %.2f -> %.2f",
                     (double)last_friction, (double)slip.friction_coefficient);
            last_friction = slip.friction_coefficient;
            last_change_time = now;
        }
    }

    // Classify surface type based on friction
    if (slip.friction_coefficient > 0.9f) {
        // Dry asphalt/concrete
        slip.wheel_speed_variance = 0.05f;
    } else if (slip.friction_coefficient > 0.6f) {
        // Wet surface or gravel
        slip.wheel_speed_variance = 0.1f;
    } else if (slip.friction_coefficient > 0.3f) {
        // Mud or snow
        slip.wheel_speed_variance = 0.2f;
    } else {
        // Ice or very slippery surface
        slip.wheel_speed_variance = 0.3f;
    }
}
