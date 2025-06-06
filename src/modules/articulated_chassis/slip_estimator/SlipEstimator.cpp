#include "SlipEstimator.hpp"
#include <px4_platform_common/log.h>

SlipEstimator::SlipEstimator() :
    ModuleParams(nullptr)
{
}

int SlipEstimator::task_spawn(int argc, char *argv[])
{
    SlipEstimator *instance = new SlipEstimator();

    if (instance) {
        _object.store(instance);
        _task_id = task_id_is_work_queue;

        if (instance->init()) {
            return PX4_OK;
        }

    } else {
        PX4_ERR("alloc failed");
    }

    delete instance;
    _object.store(nullptr);
    _task_id = -1;

    return PX4_ERROR;
}

void SlipEstimator::run()
{
    // Initialize estimators
    _ekf.init(_param_wheel_radius.get(), _param_wheelbase.get());

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

        px4_usleep(10000); // Run at 100Hz
    }
}

void SlipEstimator::updateVehicleState()
{
    // Update wheel speeds
    wheel_odometry_s odom;
    if (_wheel_odometry_front_sub.copy(&odom)) {
        _vehicle_state.wheel_speed_front = odom.velocity * _param_wheel_radius.get();
    }
    if (_wheel_odometry_rear_sub.copy(&odom)) {
        _vehicle_state.wheel_speed_rear = odom.velocity * _param_wheel_radius.get();
    }

    // Update IMU data
    sensor_accel_s accel;
    if (_sensor_accel_sub.copy(&accel)) {
        _vehicle_state.accel_x = accel.x;
        _vehicle_state.accel_y = accel.y;
    }

    vehicle_attitude_s attitude;
    if (_vehicle_attitude_sub.copy(&attitude)) {
        _vehicle_state.yaw_rate = attitude.yawspeed;
    }

    // Update position/velocity
    vehicle_local_position_s local_pos;
    if (_vehicle_local_position_sub.copy(&local_pos)) {
        _vehicle_state.speed_x = local_pos.vx;
        _vehicle_state.speed_y = local_pos.vy;
    }

    // Update steering
    steering_status_s steering;
    if (_steering_status_sub.copy(&steering)) {
        _vehicle_state.steering_angle = steering.steering_angle_rad;
    }

    _vehicle_state.last_update = hrt_absolute_time();
}

void SlipEstimator::runEKFEstimation(slip_estimation_s &slip)
{
    float dt = (_vehicle_state.last_update - slip.timestamp) * 1e-6f;
    if (dt <= 0 || dt > 1.0f) dt = 0.01f;

    // Predict step
    _ekf.predict(dt);

    // Update with wheel speeds
    _ekf.updateWheelSpeed(_vehicle_state.wheel_speed_front,
                          _vehicle_state.wheel_speed_rear);

    // Update with IMU
    _ekf.updateIMU(_vehicle_state.accel_x,
                   _vehicle_state.accel_y,
                   _vehicle_state.yaw_rate);

    // Update with GPS if available
    if (fabsf(_vehicle_state.speed_x) > 0.1f || fabsf(_vehicle_state.speed_y) > 0.1f) {
        _ekf.updateGPS(_vehicle_state.speed_x, _vehicle_state.speed_y);
    }

    // Extract estimates
    slip.longitudinal_slip_front = _ekf.getSlipFront();
    slip.longitudinal_slip_rear = _ekf.getSlipRear();
    slip.lateral_slip_angle_rad = _ekf.getLateralSlip();
    slip.surface_friction_estimate = _ekf.getFrictionEstimate();
    slip.estimated_vehicle_speed = _ekf.getState()(0); // vx
    slip.confidence = 0.9f; // High confidence for EKF
    slip.estimation_method = 1;

    // Update friction estimator with EKF results
    float avg_slip = (fabsf(slip.longitudinal_slip_front) +
                      fabsf(slip.longitudinal_slip_rear)) / 2.0f;
    float normalized_force = _vehicle_state.accel_x / 9.81f;
    _friction_estimator.update(avg_slip, normalized_force);
}

void SlipEstimator::runBasicEstimation(slip_estimation_s &slip)
{
    // Simple slip calculation
    float vehicle_speed = sqrtf(_vehicle_state.speed_x * _vehicle_state.speed_x +
                               _vehicle_state.speed_y * _vehicle_state.speed_y);

    if (vehicle_speed > 0.5f) {
        slip.longitudinal_slip_front = (_vehicle_state.wheel_speed_front - vehicle_speed) / vehicle_speed;
        slip.longitudinal_slip_rear = (_vehicle_state.wheel_speed_rear - vehicle_speed) / vehicle_speed;
    } else {
        slip.longitudinal_slip_front = 0.0f;
        slip.longitudinal_slip_rear = 0.0f;
    }

    slip.lateral_slip_angle_rad = 0.0f; // Basic estimation doesn't calculate lateral slip
    slip.surface_friction_estimate = 0.8f; // Default
    slip.confidence = 0.5f; // Lower confidence for basic method
    slip.estimation_method = 0;
    slip.estimated_vehicle_speed = vehicle_speed;
}

// SlipEKF Implementation
SlipEstimator::SlipEKF::SlipEKF()
{
    _x.setZero();
    _x(4) = 0.8f; // Initial friction estimate
    _P.setIdentity();
    _P *= 0.1f;
    _Q.setIdentity();
    _Q *= 0.01f;
    _R_wheel.setIdentity();
    _R_wheel *= 0.1f;
    _R_imu.setIdentity();
    _R_imu *= 0.05f;
}

void SlipEstimator::SlipEKF::init(float wheel_radius, float wheelbase)
{
    _wheel_radius = wheel_radius;
    _wheelbase = wheelbase;
}

void SlipEstimator::SlipEKF::predict(float dt)
{
    // Simple kinematic model with slip
    matrix::Vector<float, 5> x_dot;
    x_dot(0) = _x(4) * 9.81f * _x(2); // vx_dot = mu * g * slip_front (simplified)
    x_dot(1) = -_x(0) * _x(1) / (_wheelbase); // vy_dot (simplified)
    x_dot(2) = 0; // slip_front_dot (assumed slowly varying)
    x_dot(3) = 0; // slip_rear_dot
    x_dot(4) = 0; // friction_dot

    // State prediction
    _x += x_dot * dt;

    // Jacobian
    matrix::SquareMatrix<float, 5> F;
    F.setIdentity();
    F(0, 2) = _x(4) * 9.81f * dt;
    F(0, 4) = 9.81f * _x(2) * dt;

    // Covariance prediction
    _P = F * _P * F.transpose() + _Q;
}

void SlipEstimator::SlipEKF::updateWheelSpeed(float front_speed, float rear_speed)
{
    // Measurement model: wheel_speed = vehicle_speed * (1 + slip)
    matrix::Vector<float, 2> z;
    z(0) = front_speed;
    z(1) = rear_speed;

    matrix::Vector<float, 2> h;
    h(0) = _x(0) * (1.0f + _x(2)); // front wheel
    h(1) = _x(0) * (1.0f + _x(3)); // rear wheel

    // Measurement Jacobian
    matrix::Matrix<float, 2, 5> H;
    H.setZero();
    H(0, 0) = 1.0f + _x(2);
    H(0, 2) = _x(0);
    H(1, 0) = 1.0f + _x(3);
    H(1, 3) = _x(0);

    // Kalman gain
    matrix::SquareMatrix<float, 2> S = H * _P * H.transpose() + _R_wheel.slice<2, 2>(0, 0);
    matrix::Matrix<float, 5, 2> K = _P * H.transpose() * S.I();

    // State update
    _x += K * (z - h);

    // Covariance update
    _P = (matrix::SquareMatrix<float, 5>::identity() - K * H) * _P;
}

void SlipEstimator::SlipEKF::updateIMU(float ax, float ay, float yaw_rate)
{
    // Simple IMU update for vehicle state
    matrix::Vector<float, 3> z;
    z(0) = ax;
    z(1) = ay;
    z(2) = yaw_rate;

    matrix::Vector<float, 3> h;
    h(0) = _x(4) * 9.81f * _x(2); // Expected acceleration from slip
    h(1) = -_x(0) * _x(1) / _wheelbase; // Expected lateral acceleration
    h(2) = _x(1) / _wheelbase; // Expected yaw rate

    // Simple scalar updates (simplified for demonstration)
    float innovation = z(0) - h(0);
    _x(2) += 0.01f * innovation; // Update front slip
}

void SlipEstimator::SlipEKF::updateGPS(float vx, float vy)
{
    // Update velocity states with GPS
    _x(0) = 0.9f * _x(0) + 0.1f * vx; // Simple filter
    _x(1) = 0.9f * _x(1) + 0.1f * vy;
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
    _theta(0) = 0.8f; // Initial friction estimate
    _P.setIdentity();
    _P *= 100.0f;
}

void SlipEstimator::FrictionEstimator::update(float slip, float normalized_force)
{
    // RLS update for friction coefficient estimation
    if (fabsf(slip) < 0.01f) return; // Ignore very small slips

    matrix::Vector2f phi;
    phi(0) = slip;
    phi(1) = slip * slip; // Nonlinear term

    float y = normalized_force;
    float y_hat = phi.dot(_theta);
    float error = y - y_hat;

    // RLS update
    float denom = _lambda + phi.transpose() * _P * phi;
    if (fabsf(denom) > 1e-6f) {
        matrix::Vector2f k = _P * phi / denom;
        _theta += k * error;
        _P = (_P - k * phi.transpose() * _P) / _lambda;
    }

    // Extract friction coefficient
    _mu = math::constrain(_theta(0), 0.1f, 1.5f);

    // Update confidence based on error magnitude
    _confidence = 1.0f / (1.0f + fabsf(error));
}

void SlipEstimator::detectSurfaceChange(slip_estimation_s &slip)
{
    static float last_friction = 0.8f;
    static uint64_t last_change_time = 0;

    float friction_change = fabsf(slip.surface_friction_estimate - last_friction);

    // Detect significant friction change
    if (friction_change > 0.2f) {
        uint64_t now = hrt_absolute_time();
        if (now - last_change_time > 1000000) { // 1 second debounce
            PX4_INFO("Surface change detected: friction %.2f -> %.2f",
                     (double)last_friction, (double)slip.surface_friction_estimate);
            last_friction = slip.surface_friction_estimate;
            last_change_time = now;
        }
    }

    // Classify surface type based on friction
    if (slip.surface_friction_estimate > 0.9f) {
        // Dry asphalt/concrete
        slip.wheel_speed_variance = 0.05f;
    } else if (slip.surface_friction_estimate > 0.6f) {
        // Wet surface or gravel
        slip.wheel_speed_variance = 0.1f;
    } else if (slip.surface_friction_estimate > 0.3f) {
        // Mud or snow
        slip.wheel_speed_variance = 0.2f;
    } else {
        // Ice or very slippery surface
        slip.wheel_speed_variance = 0.3f;
    }
}
