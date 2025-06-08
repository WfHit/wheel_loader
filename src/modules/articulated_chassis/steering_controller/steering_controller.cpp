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
                       (_servo_state.error_flags == 0);
    status.emergency_stop = _emergency_stop;

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

### Examples
CLI usage example:
$ steering_controller start
$ steering_controller status
$ steering_controller stop
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("steering_controller", "driver");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print status information");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

extern "C" __EXPORT int steering_controller_main(int argc, char *argv[])
{
    return SteeringController::main(argc, argv);
}
