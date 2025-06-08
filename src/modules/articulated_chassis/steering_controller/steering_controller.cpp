#include "steering_controller.hpp"
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>

SteeringController::SteeringController() :
    ModuleBase(MODULE_NAME, px4::wq_configurations::hp_default)
{
    // Initialize angle history for moving average
    for (int i = 0; i < FILTER_SIZE; i++) {
        _angle_history[i] = 0.0f;
    }

    // Initialize limit sensor instances to invalid values
    _limit_sensors.left_instance = 255;
    _limit_sensors.right_instance = 255;
}

int SteeringController::task_spawn(int argc, char *argv[])
{
    _task_id = px4_task_spawn_cmd("steering_controller",
                                  SCHED_DEFAULT,
                                  SCHED_PRIORITY_FAST_DRIVER,
                                  2048,
                                  (px4_main_t)&run_trampoline,
                                  (char *const *)argv);

    if (_task_id < 0) {
        _task_id = -1;
        return -errno;
    }

    return 0;
}

bool SteeringController::init()
{
    PX4_INFO("Simplified Steering Controller initialized for ST3125");
    PX4_INFO("Max angle: ±%.1f°, Max rate: %.1f°/s",
             (double)math::degrees(_max_steering_angle.get()), (double)math::degrees(_max_steering_rate.get()));
    PX4_INFO("ST3125 Servo ID: %d, Current limit: %.1fA", ST3125_SERVO_ID, (double)ST3125_CURRENT_LIMIT_A);

    return true;
}

void SteeringController::run()
{
    if (!init()) {
        PX4_ERR("Steering controller initialization failed");
        return;
    }

    while (!should_exit()) {
        px4_usleep(CONTROL_INTERVAL_US);

        const hrt_abstime now = hrt_absolute_time();
        _dt = (now - _last_update_time) * 1e-6f;
        _last_update_time = now;

        // Process servo feedback first
        process_servo_feedback();

        // Process limit sensors and update safety state
        process_limit_sensors();
        update_safety_state();

        // Main control pipeline
        process_steering_command();

        // Safety check
        check_command_timeout();

        // Send servo command to ST3125
        send_servo_command(_commanded_angle_rad, _max_steering_rate.get());

        // Update status and performance metrics
        update_status();
    }
}

void SteeringController::process_steering_command()
{
    // Get steering setpoint
    wheel_loader_setpoint_s setpoint{};
    if (_wheel_loader_setpoint_sub.update(&setpoint)) {
        _target_angle_rad = setpoint.articulation_angle_setpoint;
        _vehicle_speed_mps = setpoint.forward_speed_mps;
        _last_command_time = hrt_absolute_time();

        // Apply basic limits and adjustments
        _target_angle_rad = math::constrain(_target_angle_rad,
                                           -_max_steering_angle.get(),
                                           _max_steering_angle.get());

        // Apply trim
        _target_angle_rad += _steering_trim.get();

        // Reverse if configured
        if (_steering_reverse.get()) {
            _target_angle_rad = -_target_angle_rad;
        }
    }

    // Get slip data from predictive traction
    predictive_traction_s traction{};
    if (_predictive_traction_sub.update(&traction)) {
        _slip_state.slip_front = traction.predicted_slip_front;
        _slip_state.slip_rear = traction.predicted_slip_rear;
        _slip_state.slip_error = traction.slip_tracking_error;
        _slip_state.traction_active = traction.predictive_active;
        _slip_state.last_update = traction.timestamp;
    }

    // Apply slip compensation if enabled and data is recent
    float compensated_angle = _target_angle_rad;
    if (_slip_comp_enabled.get() &&
        (hrt_absolute_time() - _slip_state.last_update) < 100_ms) {
        compensated_angle = apply_slip_compensation(_target_angle_rad);
    }

    // Apply feedforward for better dynamic response
    if (_feedforward_gain.get() > 0.0f) {
        compensated_angle = apply_feedforward(compensated_angle, _current_angle_rad);
    }

    // Rate limiting for smooth control
    _commanded_angle_rad = rate_limit(compensated_angle, _commanded_angle_rad);

    // Check position limits against limit sensors
    if (_limit_sensors_enabled.get() && !check_position_limits(_commanded_angle_rad)) {
        // Limit sensor violation - stop at safe position
        _commanded_angle_rad = _safety_state.safe_position_rad;
        handle_safety_violation();
    }

    // Update current angle estimate from servo feedback if available
    if (_servo_state.feedback_valid) {
        _current_angle_rad = _servo_state.position_rad;
    } else {
        // Fallback: assume perfect tracking with small lag
        const float tracking_tc = 0.1f;  // 100ms time constant for ST3125 response
        _current_angle_rad += (_commanded_angle_rad - _current_angle_rad) * _dt / tracking_tc;
    }

    // Update angle history for moving average filtering
    _angle_history[_history_index] = _commanded_angle_rad;
    _history_index = (_history_index + 1) % FILTER_SIZE;
}

float SteeringController::apply_slip_compensation(float target_angle)
{
    if (!_slip_state.traction_active) {
        _slip_compensation_rad = 0.0f;
        return target_angle;
    }

    // Calculate slip asymmetry (difference between front and rear wheel slip)
    float slip_asymmetry = _slip_state.slip_front - _slip_state.slip_rear;

    // Compensate for slip by adjusting steering angle
    // Positive slip asymmetry (front slipping more) requires counter-steer
    float compensation = -_slip_comp_gain.get() * slip_asymmetry;

    // Add compensation based on slip tracking error
    compensation += -_slip_comp_gain.get() * 0.5f * _slip_state.slip_error;

    // Limit compensation to prevent over-correction
    compensation = math::constrain(compensation,
                                  -_slip_comp_max.get(),
                                  _slip_comp_max.get());

    // Smooth the compensation with low-pass filter
    const float filter_tc = 0.2f;  // 200ms filter time constant
    _slip_compensation_rad += (compensation - _slip_compensation_rad) * _dt / filter_tc;

    return target_angle + _slip_compensation_rad;
}

float SteeringController::apply_feedforward(float target_angle, float current_angle)
{
    // Calculate angle error and rate
    float angle_error = target_angle - current_angle;

    // Speed-dependent feedforward gain
    float speed_factor = 1.0f + _feedforward_speed_scale.get() * fabsf(_vehicle_speed_mps);

    // Feedforward term proportional to error and speed
    float feedforward = _feedforward_gain.get() * angle_error * speed_factor;

    // Limit feedforward to prevent instability (±5 degrees in radians)
    feedforward = math::constrain(feedforward, -math::radians(5.0f), math::radians(5.0f));

    return target_angle + feedforward;
}

float SteeringController::rate_limit(float target, float current)
{
    float max_change = _max_steering_rate.get() * _dt;
    float delta = target - current;

    // Apply deadband to reduce jitter around target
    if (fabsf(delta) < _steering_deadband.get()) {
        return current;
    }

    // Rate limit the change
    if (fabsf(delta) > max_change) {
        delta = (delta > 0) ? max_change : -max_change;
    }

    return current + delta;
}

void SteeringController::send_servo_command(float position_rad, float velocity_rad_s)
{
    robotic_servo_command_s servo_cmd{};

    servo_cmd.timestamp = hrt_absolute_time();
    servo_cmd.id = ST3125_SERVO_ID;
    servo_cmd.command_type = 0;  // Position control

    // Set target position with safety limits
    servo_cmd.goal_position = math::constrain(position_rad,
                                             -ST3125_MAX_ANGLE_RAD,
                                             ST3125_MAX_ANGLE_RAD);

    // Set velocity limit
    servo_cmd.goal_velocity = math::constrain(velocity_rad_s,
                                             0.0f,
                                             ST3125_MAX_VELOCITY_RAD_S);

    // Set current limit
    servo_cmd.goal_current = _servo_current_limit.get();

    // Use servo defaults for PID (ST3125 handles internal control)
    servo_cmd.position_p_gain = 0;
    servo_cmd.position_i_gain = 0;
    servo_cmd.position_d_gain = 0;

    // Enable torque and set control mode
    servo_cmd.torque_enable = !_emergency_stop;
    servo_cmd.led_enable = true;
    servo_cmd.control_mode = 0;  // Position control

    // Set limits
    servo_cmd.moving_threshold = ST3125_DEADBAND_RAD;
    servo_cmd.current_limit = (uint16_t)(ST3125_CURRENT_LIMIT_A * 1000);  // Convert to mA
    servo_cmd.velocity_limit = (uint16_t)(ST3125_MAX_VELOCITY_RAD_S * 100);  // Convert to appropriate units
    servo_cmd.max_position = (uint16_t)(ST3125_MAX_ANGLE_RAD * 1000);
    servo_cmd.min_position = (uint16_t)(-ST3125_MAX_ANGLE_RAD * 1000);

    _servo_command_pub.publish(servo_cmd);
}

void SteeringController::process_limit_sensors()
{
    // Read limit sensor data from uORB topic
    limit_sensor_s limit_data{};
    const hrt_abstime now = hrt_absolute_time();

    // Get configured limit sensor instances
    uint8_t left_instance = static_cast<uint8_t>(_limit_left_instance.get());
    uint8_t right_instance = static_cast<uint8_t>(_limit_right_instance.get());

    // Update instance IDs if they changed
    _limit_sensors.left_instance = left_instance;
    _limit_sensors.right_instance = right_instance;

    // Process all available limit sensor messages
    while (_limit_sensor_sub.update(&limit_data)) {
        if (limit_data.instance == left_instance &&
            limit_data.function == static_cast<uint8_t>(4)) { // STEERING_LEFT
            _limit_sensors.left_limit_active = limit_data.state;
            _limit_sensors.left_limit_healthy = limit_data.healthy;
            _limit_sensors.left_last_update = limit_data.timestamp;
        } else if (limit_data.instance == right_instance &&
                   limit_data.function == static_cast<uint8_t>(5)) { // STEERING_RIGHT
            _limit_sensors.right_limit_active = limit_data.state;
            _limit_sensors.right_limit_healthy = limit_data.healthy;
            _limit_sensors.right_last_update = limit_data.timestamp;
        }
    }

    // Check for sensor timeouts
    const uint64_t timeout_threshold = 500_ms; // 500ms timeout for limit sensors

    if ((now - _limit_sensors.left_last_update) > timeout_threshold) {
        _limit_sensors.left_limit_healthy = false;
    }

    if ((now - _limit_sensors.right_last_update) > timeout_threshold) {
        _limit_sensors.right_limit_healthy = false;
    }
}

bool SteeringController::check_position_limits(float target_angle)
{
    if (!_limit_sensors_enabled.get()) {
        return true; // No limit checking if disabled
    }

    const float margin = _limit_margin_rad.get();

    // Check left limit (negative angle direction)
    if (target_angle < -margin && _limit_sensors.left_limit_active) {
        if (_limit_sensors.left_limit_healthy) {
            PX4_WARN_THROTTLED(1000, "Left steering limit reached at %.1f°",
                              (double)math::degrees(target_angle));
            return false;
        } else {
            // Sensor unhealthy, assume limit is active for safety
            PX4_WARN_THROTTLED(1000, "Left limit sensor unhealthy, assuming limit active");
            return false;
        }
    }

    // Check right limit (positive angle direction)
    if (target_angle > margin && _limit_sensors.right_limit_active) {
        if (_limit_sensors.right_limit_healthy) {
            PX4_WARN_THROTTLED(1000, "Right steering limit reached at %.1f°",
                              (double)math::degrees(target_angle));
            return false;
        } else {
            // Sensor unhealthy, assume limit is active for safety
            PX4_WARN_THROTTLED(1000, "Right limit sensor unhealthy, assuming limit active");
            return false;
        }
    }

    return true; // No limits violated
}

void SteeringController::handle_safety_violation()
{
    _safety_state.safety_violation = true;
    _safety_state.position_limit_violation = true;
    _safety_state.last_violation_time = hrt_absolute_time();
    _safety_state.violation_count++;

    // Log safety violation
    PX4_ERR("Steering safety violation detected - count: %u", _safety_state.violation_count);

    // Check if too many violations occurred
    if (_safety_state.violation_count > static_cast<uint32_t>(_max_violations.get())) {
        _safety_state.emergency_stop_active = true;
        _emergency_stop = true;
        PX4_ERR("Too many safety violations - emergency stop activated");
    }
}

void SteeringController::update_safety_state()
{
    const hrt_abstime now = hrt_absolute_time();

    if (!_safety_manager_enabled.get()) {
        // Safety manager disabled, clear all safety states
        _safety_state.safety_violation = false;
        _safety_state.position_limit_violation = false;
        _safety_state.emergency_stop_active = false;
        return;
    }

    // Update safe position (typically center position)
    _safety_state.safe_position_rad = _safety_position_rad.get();

    // Check servo faults
    _safety_state.servo_fault = (_servo_state.error_flags != 0) ||
                                !_servo_state.feedback_valid ||
                                (_servo_state.current_a > ST3125_CURRENT_LIMIT_A * 1.2f);

    // Check sensor faults
    _safety_state.sensor_fault = !_limit_sensors.left_limit_healthy ||
                                 !_limit_sensors.right_limit_healthy;

    // Clear violation flag after timeout if no active violations
    const float timeout_ms = _fault_timeout_ms.get();
    if (_safety_state.safety_violation &&
        (now - _safety_state.last_violation_time) > (timeout_ms * 1000)) {

        // Check if we can clear the violation
        bool can_clear = !_safety_state.servo_fault &&
                        !_safety_state.sensor_fault &&
                        !_limit_sensors.left_limit_active &&
                        !_limit_sensors.right_limit_active;

        if (can_clear) {
            _safety_state.safety_violation = false;
            _safety_state.position_limit_violation = false;
            PX4_INFO("Safety violation cleared after timeout");
        }
    }

    // Update emergency stop state
    if (_safety_state.emergency_stop_active) {
        _emergency_stop = true;

        // Emergency stop can only be cleared manually (reset violation count)
        if (_safety_state.violation_count == 0) {
            _safety_state.emergency_stop_active = false;
            _emergency_stop = false;
            PX4_INFO("Emergency stop cleared");
        }
    }
}

void SteeringController::process_servo_feedback()
{
    // Process servo feedback from ST3125
    servo_feedback_s feedback{};
    if (_servo_feedback_sub.update(&feedback)) {
        if (feedback.id == ST3125_SERVO_ID) {
            _servo_state.position_rad = feedback.position;
            _servo_state.velocity_rad_s = feedback.velocity;
            _servo_state.current_a = feedback.current;
            _servo_state.temperature_c = feedback.temperature;
            _servo_state.error_flags = feedback.error_flags;
            _servo_state.torque_enabled = feedback.torque_enabled;
            _servo_state.feedback_valid = true;
            _last_feedback_time = feedback.timestamp;
        }
    } else {
        // Check for feedback timeout
        if ((hrt_absolute_time() - _last_feedback_time) > SERVO_FEEDBACK_TIMEOUT_US) {
            _servo_state.feedback_valid = false;
            _metrics.feedback_timeout_count++;
        }
    }

    // Note: No external sensor needed - ST3125 servo provides internal position feedback
    // Current angle is updated from servo feedback above
}

void SteeringController::check_command_timeout()
{
    if ((hrt_absolute_time() - _last_command_time) > COMMAND_TIMEOUT_US) {
        // No recent commands, gradually return to center
        _target_angle_rad = 0.0f;
        _metrics.timeout_count++;

        if (_metrics.timeout_count % 100 == 1) {  // Throttled warning
            PX4_WARN("Steering command timeout, returning to center");
        }
    }
}

void SteeringController::update_status()
{
    const hrt_abstime now = hrt_absolute_time();

    // Publish steering status
    steering_status_s status{};
    status.timestamp = now;
    status.steering_angle_deg = math::degrees(_current_angle_rad);
    status.steering_angle_setpoint_deg = math::degrees(_target_angle_rad);
    status.steering_rate_deg_s = math::degrees((_commanded_angle_rad - _current_angle_rad) / _dt);
    status.slip_compensation_deg = math::degrees(_slip_compensation_rad);
    status.control_mode = steering_status_s::CONTROL_MODE_POSITION;

    // Calculate and update performance metrics
    float error = fabsf(_target_angle_rad - _current_angle_rad);
    _metrics.max_error_rad = fmaxf(_metrics.max_error_rad, error);

    if (_metrics.command_count > 0) {
        _metrics.avg_error_rad = (_metrics.avg_error_rad * _metrics.command_count + error) /
                                (_metrics.command_count + 1);
    } else {
        _metrics.avg_error_rad = error;
    }
    _metrics.command_count++;

    // Set status flags
    status.is_healthy = (_metrics.max_error_rad < MAX_POSITION_ERROR_RAD) &&
                       (!_emergency_stop) &&
                       (_servo_state.feedback_valid) &&
                       (_servo_state.error_flags == 0) &&
                       (!_safety_state.safety_violation);
    status.emergency_stop = _emergency_stop;

    // Add safety and limit sensor status
    status.limit_left_active = _limit_sensors.left_limit_active;
    status.limit_right_active = _limit_sensors.right_limit_active;
    status.limit_sensors_healthy = _limit_sensors.left_limit_healthy && _limit_sensors.right_limit_healthy;
    status.safety_violation = _safety_state.safety_violation;

    _steering_status_pub.publish(status);
}

int SteeringController::print_status()
{
    PX4_INFO("ST3125 Steering Controller Status:");
    PX4_INFO("  Current Angle: %.1f°", (double)math::degrees(_current_angle_rad));
    PX4_INFO("  Target Angle: %.1f°", (double)math::degrees(_target_angle_rad));
    PX4_INFO("  Commanded Angle: %.1f°", (double)math::degrees(_commanded_angle_rad));
    PX4_INFO("  Slip Compensation: %.2f°", (double)math::degrees(_slip_compensation_rad));
    PX4_INFO("  Vehicle Speed: %.1f m/s", (double)_vehicle_speed_mps);
    PX4_INFO("  Servo State:");
    PX4_INFO("    Position: %.1f°", (double)math::degrees(_servo_state.position_rad));
    PX4_INFO("    Velocity: %.1f°/s", (double)math::degrees(_servo_state.velocity_rad_s));
    PX4_INFO("    Current: %.2fA", (double)_servo_state.current_a);
    PX4_INFO("    Temperature: %.1f°C", (double)_servo_state.temperature_c);
    PX4_INFO("    Error Flags: 0x%04X", _servo_state.error_flags);
    PX4_INFO("    Torque Enabled: %s", _servo_state.torque_enabled ? "YES" : "NO");
    PX4_INFO("    Feedback Valid: %s", _servo_state.feedback_valid ? "YES" : "NO");
    PX4_INFO("  Slip State:");
    PX4_INFO("    Front Slip: %.3f", (double)_slip_state.slip_front);
    PX4_INFO("    Rear Slip: %.3f", (double)_slip_state.slip_rear);
    PX4_INFO("    Tracking Error: %.3f", (double)_slip_state.slip_error);
    PX4_INFO("    Traction Active: %s", _slip_state.traction_active ? "YES" : "NO");
    PX4_INFO("  Performance:");
    PX4_INFO("    Max Error: %.2f°", (double)math::degrees(_metrics.max_error_rad));
    PX4_INFO("    Avg Error: %.2f°", (double)math::degrees(_metrics.avg_error_rad));
    PX4_INFO("    Commands: %u", _metrics.command_count);
    PX4_INFO("    Timeouts: %u", _metrics.timeout_count);
    PX4_INFO("    Feedback Timeouts: %u", _metrics.feedback_timeout_count);
    PX4_INFO("  Limit Sensors:");
    PX4_INFO("    Left (Instance %u): %s, Healthy: %s",
             _limit_sensors.left_instance,
             _limit_sensors.left_limit_active ? "ACTIVE" : "CLEAR",
             _limit_sensors.left_limit_healthy ? "YES" : "NO");
    PX4_INFO("    Right (Instance %u): %s, Healthy: %s",
             _limit_sensors.right_instance,
             _limit_sensors.right_limit_active ? "ACTIVE" : "CLEAR",
             _limit_sensors.right_limit_healthy ? "YES" : "NO");
    PX4_INFO("  Safety Manager:");
    PX4_INFO("    Enabled: %s", _safety_manager_enabled.get() ? "YES" : "NO");
    PX4_INFO("    Safety Violation: %s", _safety_state.safety_violation ? "YES" : "NO");
    PX4_INFO("    Position Limit Violation: %s", _safety_state.position_limit_violation ? "YES" : "NO");
    PX4_INFO("    Emergency Stop: %s", _safety_state.emergency_stop_active ? "YES" : "NO");
    PX4_INFO("    Servo Fault: %s", _safety_state.servo_fault ? "YES" : "NO");
    PX4_INFO("    Sensor Fault: %s", _safety_state.sensor_fault ? "YES" : "NO");
    PX4_INFO("    Violation Count: %u", _safety_state.violation_count);
    PX4_INFO("    Safe Position: %.1f°", (double)math::degrees(_safety_state.safe_position_rad));

    return 0;
}

int SteeringController::custom_command(int argc, char *argv[])
{
    if (!is_running()) {
        PX4_ERR("Controller not running");
        return -1;
    }

    if (!strcmp(argv[0], "status")) {
        return get_instance()->print_status();
    }

    if (!strcmp(argv[0], "reset_safety")) {
        if (get_instance()) {
            get_instance()->_safety_state.violation_count = 0;
            get_instance()->_safety_state.safety_violation = false;
            get_instance()->_safety_state.position_limit_violation = false;
            get_instance()->_safety_state.emergency_stop_active = false;
            get_instance()->_emergency_stop = false;
            PX4_INFO("Safety violations reset");
        }
        return 0;
    }

    return print_usage("unknown command");
}

int SteeringController::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Simplified steering controller for articulated wheel loader with ST3125 servo.

Features:
- Direct position commands to ST3125 (servo handles internal PID)
- Slip compensation using PredictiveTraction data
- Feedforward control for improved dynamic response
- Rate limiting and safety monitoring
- Limit sensor integration for position safety
- Safety manager with fault detection and emergency stop

### Examples
CLI usage example:
$ steering_controller start
$ steering_controller status
$ steering_controller reset_safety
$ steering_controller stop
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("steering_controller", "driver");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print status information");
    PRINT_MODULE_USAGE_COMMAND_DESCR("reset_safety", "Reset safety violations and emergency stop");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

extern "C" __EXPORT int steering_controller_main(int argc, char *argv[])
{
    return SteeringController::main(argc, argv);
}
