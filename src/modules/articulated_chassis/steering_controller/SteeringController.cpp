#include "SteeringController.hpp"
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/WheelLoaderSetpoint.h>
#include <uORB/topics/WheelLoaderStatus.h>
#include <drivers/drv_hrt.h>
#include <mathlib/mathlib.h>
#include <module_constants/module_constants.h>

SteeringController::SteeringController() :
    ModuleBase(MODULE_NAME, px4::wq_configurations::hp_default)
{
    // Initialize PID controller for position control
    pid_init(&_position_pid, PID_MODE_DERIVATIV_CALC, 0.02f);

    // Initialize controller status
    _status = {};
    _status.module_id = MODULE_ID_STEERING;
    _status.timestamp = hrt_absolute_time();

    // Initialize steering status
    _steering_status = {};
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
    // Update PID parameters from PX4 parameters
    _position_pid.kp = _position_p_gain.get();
    _position_pid.ki = _position_i_gain.get();
    _position_pid.kd = _position_d_gain.get();
    _position_pid.integral_max = _position_i_max.get();

    // Calculate maximum rate per control cycle
    _max_rate_deg_per_cycle = _max_steering_rate.get() * (CONTROL_INTERVAL_US / 1e6f);

    PX4_INFO("Steering controller initialized");
    PX4_INFO("PID gains: P=%.3f, I=%.3f, D=%.3f",
             (double)_position_pid.kp, (double)_position_pid.ki, (double)_position_pid.kd);
    PX4_INFO("Max angle: ±%.1f°, Max rate: %.1f°/s",
             (double)_max_steering_angle.get(), (double)_max_steering_rate.get());

    return true;
}

void SteeringController::run()
{
    if (!init()) {
        PX4_ERR("Steering controller initialization failed");
        return;
    }

    // Main control loop
    while (!should_exit()) {

        // Check for parameter updates
        if (_parameter_update_sub.updated()) {
            parameter_update_s param_update;
            _parameter_update_sub.copy(&param_update);
            updateParams();

            // Update PID gains
            _position_pid.kp = _position_p_gain.get();
            _position_pid.ki = _position_i_gain.get();
            _position_pid.kd = _position_d_gain.get();
            _position_pid.integral_max = _position_i_max.get();

            // Update rate limit
            _max_rate_deg_per_cycle = _max_steering_rate.get() * (CONTROL_INTERVAL_US / 1e6f);
        }

        // Process position feedback
        process_position_feedback();

        // Update steering control
        update_steering_control();

        // Apply rate limiting
        apply_rate_limiting();

        // Safety checks
        check_safety_limits();

        // Communicate with servo
        communicate_with_servo();

        // Update controller status
        update_controller_status();

        // Sleep until next control cycle
        px4_usleep(CONTROL_INTERVAL_US);
    }
}

void SteeringController::update_steering_control()
{
    // Get wheel loader setpoint for articulation angle
    if (_wheel_loader_setpoint_sub.updated()) {
        wheel_loader_setpoint_s setpoint;
        _wheel_loader_setpoint_sub.copy(&setpoint);

        // Convert articulation angle from radians to degrees for internal use
        _target_angle_deg = math::degrees(setpoint.articulation_angle_setpoint);
        _last_command_time = hrt_absolute_time();
    }

    // Check command timeout
    if (hrt_elapsed_time(&_last_command_time) > WATCHDOG_TIMEOUT_US) {
        _target_angle_deg = 0.0f; // Return to center
        _performance.command_timeouts++;
        PX4_WARN_THROTTLE(1000, "Steering: Command timeout, returning to center");
    }

    // Clamp target angle to limits
    float max_angle = _max_steering_angle.get();
    _target_angle_deg = math::constrain(_target_angle_deg, -max_angle, max_angle);

    // Apply trim offset
    _target_angle_deg += _steering_trim.get();

    // Apply steering reverse if configured
    if (_steering_reverse.get()) {
        _target_angle_deg = -_target_angle_deg;
    }
}

void SteeringController::apply_rate_limiting()
{
    // Calculate desired change
    float angle_change = _target_angle_deg - _rate_limited_target_deg;

    // Limit rate of change
    if (fabsf(angle_change) > _max_rate_deg_per_cycle) {
        _rate_limited_target_deg += copysignf(_max_rate_deg_per_cycle, angle_change);
    } else {
        _rate_limited_target_deg = _target_angle_deg;
    }

    // Calculate PID output
    float position_error = _rate_limited_target_deg - _current_angle_deg;

    // Apply deadband
    if (fabsf(position_error) < _steering_deadband.get()) {
        position_error = 0.0f;
    }

    float dt = CONTROL_INTERVAL_US / 1e6f;
    float pid_output = pid_calculate(&_position_pid, position_error, dt);

    // Convert PID output to servo PWM
    _servo_pwm_us = angle_to_pwm(_current_angle_deg + pid_output);

    // Update performance metrics
    _performance.position_error_rms = 0.99f * _performance.position_error_rms +
                                      0.01f * position_error * position_error;

    float tracking_error = fabsf(_target_angle_deg - _current_angle_deg);
    if (tracking_error > _performance.tracking_error_max) {
        _performance.tracking_error_max = tracking_error;
    }
}

void SteeringController::process_position_feedback()
{
    // Get articulation angle from sensor
    _current_angle_deg = get_articulation_angle_deg();
    _position_feedback_valid = true; // Set based on actual sensor validity

    uint64_t current_time = hrt_absolute_time();
    _last_position_time = current_time;

    // Update response time performance metric
    static uint64_t command_start_time = 0;
    if (fabsf(_target_angle_deg - _previous_target_deg) > 1.0f) {
        command_start_time = current_time;
    }

    if (command_start_time > 0 &&
        fabsf(_current_angle_deg - _target_angle_deg) < _steering_deadband.get()) {
        float response_time = (current_time - command_start_time) / 1e6f;
        _performance.response_time_avg = 0.9f * _performance.response_time_avg +
                                         0.1f * response_time;
        command_start_time = 0;
    }

    _previous_target_deg = _target_angle_deg;
}

float SteeringController::get_articulation_angle_deg()
{
    // TODO: Read actual articulation angle from sensor
    // This could be from:
    // - Rotary encoder on steering mechanism
    // - Potentiometer feedback
    // - Hall sensor
    // - Calculated from servo position

    // For now, simulate feedback based on servo position
    return pwm_to_angle(_servo_pwm_us);
}

void SteeringController::check_safety_limits()
{
    bool safety_violation = false;

    // Check emergency stop
    _emergency_stop = is_emergency_stop_active();

    // Check position error
    float position_error = fabsf(_rate_limited_target_deg - _current_angle_deg);
    if (position_error > MAX_POSITION_ERROR_DEG) {
        PX4_WARN_THROTTLE(1000, "Steering: Excessive position error %.1f°",
                          (double)position_error);
        safety_violation = true;
    }

    // Check servo current
    if (_servo_current_ma > _servo_current_limit.get()) {
        PX4_WARN_THROTTLE(1000, "Steering: Servo overcurrent %.0f mA",
                          (double)_servo_current_ma);
        safety_violation = true;
    }

    // Check position feedback validity
    if (!_position_feedback_valid) {
        PX4_WARN_THROTTLE(1000, "Steering: Invalid position feedback");
        safety_violation = true;
    }

    // Emergency stop or safety violation
    if (_emergency_stop || safety_violation) {
        _servo_pwm_us = SERVO_PWM_CENTER; // Return to center
        _rate_limited_target_deg = 0.0f;
        _target_angle_deg = 0.0f;
        pid_reset_integral(&_position_pid);

        if (safety_violation) {
            _performance.safety_violations++;
        }
    }

    _module_healthy = !(_emergency_stop || safety_violation);
}

void SteeringController::communicate_with_servo()
{
    // TODO: Implement actual ST3125 servo communication
    // This is a placeholder for hardware interface

    // Clamp PWM to configured limits
    float pwm_min = _servo_pwm_min.get();
    float pwm_max = _servo_pwm_max.get();
    _servo_pwm_us = math::constrain(_servo_pwm_us, pwm_min, pwm_max);

    // Send PWM command to ST3125 (placeholder)
    // st3125_set_servo_pwm(STEERING_SERVO_ID, _servo_pwm_us);

    // Read servo current (placeholder)
    // _servo_current_ma = st3125_get_servo_current(STEERING_SERVO_ID);
    _servo_current_ma = fabsf(_servo_pwm_us - SERVO_PWM_CENTER) / SERVO_PWM_RANGE * 1500.0f; // Simulated
}

void SteeringController::update_controller_status()
{
    uint64_t timestamp = hrt_absolute_time();

    // Update controller status
    _status.timestamp = timestamp;
    _status.module_id = MODULE_ID_STEERING;
    _status.health_status = _module_healthy ? MODULE_HEALTH_OK : MODULE_HEALTH_ERROR;
    _status.operational_status = _emergency_stop ? MODULE_OP_EMERGENCY_STOP : MODULE_OP_NORMAL;

    // Status data
    _status.data[0] = _current_angle_deg;          // Current angle
    _status.data[1] = _target_angle_deg;           // Target angle
    _status.data[2] = _rate_limited_target_deg;    // Rate limited target
    _status.data[3] = _servo_pwm_us;               // Servo PWM
    _status.data[4] = _servo_current_ma;           // Servo current
    _status.data[5] = _performance.position_error_rms; // RMS position error

    _module_status_pub.publish(_status);

    // Update steering status (legacy)
    _steering_status.timestamp = timestamp;
    _steering_status.angle_deg = _current_angle_deg;
    _steering_status.angle_setpoint_deg = _rate_limited_target_deg;
    _steering_status.angular_velocity_deg_s = 0.0f; // TODO: Calculate from position derivative
    _steering_status.servo_pwm_us = _servo_pwm_us;
    _steering_status.position_error_deg = _rate_limited_target_deg - _current_angle_deg;
    _steering_status.servo_current_ma = _servo_current_ma;
    _steering_status.feedback_valid = _position_feedback_valid;
    _steering_status.safety_ok = _module_healthy;

    _steering_status_pub.publish(_steering_status);

    // Update wheel loader status with articulation angle feedback
    _wheel_loader_status.timestamp = timestamp;
    // Convert current angle from degrees to radians for the message
    _wheel_loader_status.articulation_angle_feedback = math::radians(_current_angle_deg);
    // Note: Other fields in wheel loader status should be updated by other modules
    // We only update the articulation angle feedback here

    _wheel_loader_status_pub.publish(_wheel_loader_status);
}

float SteeringController::angle_to_pwm(float angle_deg)
{
    // Convert angle to PWM microseconds
    // Assumes linear relationship between angle and PWM
    float normalized_angle = angle_deg / _max_steering_angle.get();
    return SERVO_PWM_CENTER + normalized_angle * SERVO_PWM_RANGE;
}

float SteeringController::pwm_to_angle(float pwm_us)
{
    // Convert PWM microseconds to angle
    float normalized_pwm = (pwm_us - SERVO_PWM_CENTER) / SERVO_PWM_RANGE;
    return normalized_pwm * _max_steering_angle.get();
}

float SteeringController::normalize_angle(float angle_deg)
{
    // Normalize angle to [-180, 180] range
    while (angle_deg > 180.0f) angle_deg -= 360.0f;
    while (angle_deg < -180.0f) angle_deg += 360.0f;
    return angle_deg;
}

bool SteeringController::is_emergency_stop_active()
{
    if (_vehicle_status_sub.updated()) {
        vehicle_status_s vehicle_status;
        _vehicle_status_sub.copy(&vehicle_status);
        return vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_STANDBY;
    }
    return false;
}

int SteeringController::print_status()
{
    PX4_INFO("Steering Controller Status:");
    PX4_INFO("  Current Angle: %.1f°", (double)_current_angle_deg);
    PX4_INFO("  Target Angle: %.1f°", (double)_target_angle_deg);
    PX4_INFO("  Rate Limited Target: %.1f°", (double)_rate_limited_target_deg);
    PX4_INFO("  Servo PWM: %.0f μs", (double)_servo_pwm_us);
    PX4_INFO("  Servo Current: %.0f mA", (double)_servo_current_ma);
    PX4_INFO("  Position Error: %.2f°", (double)(_rate_limited_target_deg - _current_angle_deg));
    PX4_INFO("  Controller Healthy: %s", _module_healthy ? "YES" : "NO");
    PX4_INFO("  Emergency Stop: %s", _emergency_stop ? "ACTIVE" : "INACTIVE");
    PX4_INFO("  Position Feedback: %s", _position_feedback_valid ? "VALID" : "INVALID");
    PX4_INFO("  Performance:");
    PX4_INFO("    Position Error RMS: %.2f°", (double)_performance.position_error_rms);
    PX4_INFO("    Max Tracking Error: %.2f°", (double)_performance.tracking_error_max);
    PX4_INFO("    Avg Response Time: %.2f s", (double)_performance.response_time_avg);
    PX4_INFO("    Safety Violations: %u", _performance.safety_violations);
    PX4_INFO("    Command Timeouts: %u", _performance.command_timeouts);

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
Steering controller for articulated wheel loader.

Controls the steering servo via ST3125 servo controller with PID position control,
rate limiting, and safety monitoring.

### Examples
CLI usage example:
$ steering_controller start
$ steering_controller status
$ steering_controller stop
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("steering_module", "driver");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print status information");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

extern "C" __EXPORT int steering_module_main(int argc, char *argv[])
{
    return SteeringController::main(argc, argv);
}
