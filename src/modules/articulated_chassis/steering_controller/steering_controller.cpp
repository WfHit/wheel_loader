#include "steering_controller.hpp"
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/module.h>
#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <lib/mathlib/math/filter/LowPassFilter2p.hpp>

#define MODULE_NAME "steering_controller"

SteeringController::SteeringController() :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
    // Initialize angle history for moving average filter
    for (int i = 0; i < FILTER_SIZE; i++) {
        _angle_history[i] = 0.0f;
    }

    // Initialize limit sensor instances to invalid values
    _limit_sensors.left_instance = 255;
    _limit_sensors.right_instance = 255;

    // Initialize safety state
    _safety_state.safe_position_rad = 0.0f;
}

bool SteeringController::init()
{
    // Schedule at desired rate
    ScheduleOnInterval(CONTROL_INTERVAL_US);

    PX4_INFO("Steering Controller initialized for ST3125");
    PX4_INFO("Max angle: ±%.1f°, Max rate: %.1f°/s",
             (double)math::degrees(_max_steering_angle.get()),
             (double)math::degrees(_max_steering_rate.get()));
    PX4_INFO("ST3125 Servo ID: %ld, Current limit: %.1fA",
             (long)_st3125_servo_id.get(), (double)_st3125_current_limit.get());

    return true;
}

void SteeringController::Run()
{
    // ScheduledWorkItem interface - delegate to run()
    run();
}

void SteeringController::run()
{
    if (should_exit()) {
        ScheduleClear();
        exit_and_cleanup();
        return;
    }

    // Update timing
    const uint64_t now = hrt_absolute_time();
    if (_last_update_time != 0) {
        _dt = math::constrain((now - _last_update_time) * 1e-6f, 0.001f, 0.1f);
    }
    _last_update_time = now;

    // Update vehicle status
    vehicle_status_s vehicle_status{};
    if (_vehicle_status_sub.update(&vehicle_status)) {
        _emergency_stop = (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_TERMINATION);
    }

    // Process inputs and feedback
    process_servo_feedback();
    process_limit_sensors();
    process_steering_command();

    // Check safety conditions
    check_command_timeout();
    update_safety_state();

    // Publish status
    update_status();
}

void SteeringController::process_steering_command()
{
    steering_setpoint_s setpoint{};
    if (_steering_setpoint_sub.update(&setpoint)) {
        _last_command_time = hrt_absolute_time();
        _metrics.command_count++;

        // Apply reverse if configured
        float raw_angle = setpoint.steering_angle_rad;
        if (_steering_reverse.get()) {
            raw_angle = -raw_angle;
        }

        // Apply trim
        raw_angle += _steering_trim.get();

        // Constrain to limits
        _target_angle_rad = math::constrain(raw_angle,
            -_max_steering_angle.get(),
            _max_steering_angle.get());

        // Apply slip compensation if enabled
        if (_slip_comp_enabled.get()) {
            _target_angle_rad = apply_slip_compensation(_target_angle_rad);
        }

        // Apply feedforward
        float feedforward_angle = apply_feedforward(_target_angle_rad, _current_angle_rad);

        // Rate limit
        float limited_angle = rate_limit(feedforward_angle, _commanded_angle_rad);

        // Check position limits
        if (check_position_limits(limited_angle)) {
            // Apply deadband
            if (fabsf(limited_angle) < _steering_deadband.get()) {
                limited_angle = 0.0f;
            }

            _commanded_angle_rad = limited_angle;

            // Send command to servo
            float velocity_cmd = 0.0f;
            if (_feedforward_gain.get() > 0.0f) {
                velocity_cmd = (_target_angle_rad - _current_angle_rad) * _feedforward_gain.get();
                velocity_cmd = math::constrain(velocity_cmd,
                    -_st3125_max_velocity.get(),
                    _st3125_max_velocity.get());
            }

            send_servo_command(_commanded_angle_rad, velocity_cmd);
        }
    }
}

float SteeringController::apply_slip_compensation(float target_angle)
{
    predictive_traction_s traction{};
    if (_predictive_traction_sub.update(&traction)) {
        _slip_state.last_update = hrt_absolute_time();
        _slip_state.slip_front = traction.predicted_slip_front;
        _slip_state.slip_rear = traction.predicted_slip_rear;
        _slip_state.traction_active = traction.predictive_active;

        if (_slip_state.traction_active) {
            // Calculate slip error (positive = understeer, negative = oversteer)
            _slip_state.slip_error = _slip_state.slip_front - _slip_state.slip_rear;

            // Apply compensation
            float compensation = _slip_state.slip_error * _slip_comp_gain.get();
            compensation = math::constrain(compensation,
                -_slip_comp_max.get(),
                _slip_comp_max.get());

            _slip_compensation_rad = compensation;
            return target_angle + compensation;
        }
    }

    _slip_compensation_rad = 0.0f;
    return target_angle;
}

float SteeringController::apply_feedforward(float target_angle, float current_angle)
{
    if (_feedforward_gain.get() <= 0.0f) {
        return target_angle;
    }

    // Basic feedforward based on angle error and vehicle speed
    float angle_error = target_angle - current_angle;
    float ff_gain = _feedforward_gain.get();

    // Scale feedforward with vehicle speed
    if (_feedforward_speed_scale.get() > 0.0f && _vehicle_speed_mps > 0.1f) {
        ff_gain *= (1.0f + _vehicle_speed_mps * _feedforward_speed_scale.get());
    }

    float feedforward = angle_error * ff_gain * _dt;
    return current_angle + feedforward;
}

float SteeringController::rate_limit(float target, float current)
{
    float max_change = _max_steering_rate.get() * _dt;
    float delta = target - current;

    if (fabsf(delta) > max_change) {
        delta = (delta > 0.0f ? 1.0f : -1.0f) * max_change;
    }

    return current + delta;
}

void SteeringController::send_servo_command(float position_rad, float velocity_rad_s)
{
    robotic_servo_command_s cmd{};
    cmd.timestamp = hrt_absolute_time();
    cmd.id = _st3125_servo_id.get();
    cmd.command_type = 0; // position control
    cmd.goal_position = position_rad;
    cmd.goal_velocity = velocity_rad_s;
    cmd.goal_current = _st3125_current_limit.get();
    cmd.torque_enable = true;

    _servo_command_pub.publish(cmd);
}

void SteeringController::process_servo_feedback()
{
    robotic_servo_feedback_s feedback{};
    if (_servo_feedback_sub.update(&feedback)) {
        if (feedback.id == _st3125_servo_id.get()) {
            _last_feedback_time = hrt_absolute_time();

            _servo_state.position_rad = feedback.position;
            _servo_state.velocity_rad_s = feedback.velocity;
            _servo_state.current_a = feedback.current;
            _servo_state.temperature_c = feedback.temperature;
            _servo_state.error_flags = feedback.error_flags;
            _servo_state.torque_enabled = feedback.torque_enabled;
            _servo_state.feedback_valid = true;

            // Update current angle with filtering
            _angle_history[_history_index] = feedback.position;
            _history_index = (_history_index + 1) % FILTER_SIZE;

            float sum = 0.0f;
            for (int i = 0; i < FILTER_SIZE; i++) {
                sum += _angle_history[i];
            }
            _current_angle_rad = sum / FILTER_SIZE;

            // Update metrics
            float error = fabsf(_commanded_angle_rad - _current_angle_rad);
            _metrics.max_error_rad = math::max(_metrics.max_error_rad, error);
            _metrics.avg_error_rad = (_metrics.avg_error_rad * 0.95f) + (error * 0.05f);
        }
    }

    // Check for feedback timeout
    if ((hrt_absolute_time() - _last_feedback_time) > (_feedback_timeout_ms.get() * 1000)) {
        _servo_state.feedback_valid = false;
        _metrics.feedback_timeout_count++;
    }
}

void SteeringController::process_limit_sensors()
{
    if (!_limit_sensors_enabled.get()) {
        return;
    }

    // Update limit sensor instances from parameters if needed
    if (_limit_sensors.left_instance != _limit_left_instance.get()) {
        _limit_sensors.left_instance = _limit_left_instance.get();
    }
    if (_limit_sensors.right_instance != _limit_right_instance.get()) {
        _limit_sensors.right_instance = _limit_right_instance.get();
    }

    limit_sensor_s sensor_data{};
    while (_limit_sensor_sub.update(&sensor_data)) {
        _metrics.sensor_update_count++;

        if (sensor_data.instance == _limit_sensors.left_instance) {
            _limit_sensors.left_last_update = hrt_absolute_time();
            _limit_sensors.left_limit_active = sensor_data.state;
            _limit_sensors.left_limit_healthy = !sensor_data.redundancy_fault;

            if (sensor_data.redundancy_fault) {
                _metrics.sensor_error_count++;
            }
        }
        else if (sensor_data.instance == _limit_sensors.right_instance) {
            _limit_sensors.right_last_update = hrt_absolute_time();
            _limit_sensors.right_limit_active = sensor_data.state;
            _limit_sensors.right_limit_healthy = !sensor_data.redundancy_fault;

            if (sensor_data.redundancy_fault) {
                _metrics.sensor_error_count++;
            }
        }
    }

    // Check for sensor timeouts
    const uint64_t now = hrt_absolute_time();
    const uint64_t sensor_timeout_us = _sensor_timeout_ms.get() * 1000;
    if ((now - _limit_sensors.left_last_update) > sensor_timeout_us) {
        _limit_sensors.left_limit_healthy = false;
        _metrics.sensor_timeout_count++;
    }
    if ((now - _limit_sensors.right_last_update) > sensor_timeout_us) {
        _limit_sensors.right_limit_healthy = false;
        _metrics.sensor_timeout_count++;
    }

    _sensor_valid = _limit_sensors.left_limit_healthy && _limit_sensors.right_limit_healthy;
}

bool SteeringController::check_position_limits(float target_angle)
{
    if (!_limit_sensors_enabled.get()) {
        return true;  // No limit checking if disabled
    }

    // Check if we're approaching a limit
    bool left_limit_approaching = (target_angle < 0) &&
        (_limit_sensors.left_limit_active ||
         (fabsf(target_angle) > (_max_steering_angle.get() - _limit_margin_rad.get())));

    bool right_limit_approaching = (target_angle > 0) &&
        (_limit_sensors.right_limit_active ||
         (fabsf(target_angle) > (_max_steering_angle.get() - _limit_margin_rad.get())));

    if (left_limit_approaching || right_limit_approaching) {
        _safety_state.position_limit_violation = true;
        return false;
    }

    return true;
}

void SteeringController::handle_safety_violation()
{
    _safety_state.safety_violation = true;
    _safety_state.violation_count++;
    _safety_state.last_violation_time = hrt_absolute_time();

    // Command to safe position
    send_servo_command(_safety_state.safe_position_rad, 0.0f);
}

void SteeringController::update_safety_state()
{
    const uint64_t now = hrt_absolute_time();

    // Update safety position from parameter
    _safety_state.safe_position_rad = _safety_position_rad.get();

    // Check for emergency stop
    if (_emergency_stop) {
        _safety_state.emergency_stop_active = true;
        handle_safety_violation();
        return;
    } else {
        _safety_state.emergency_stop_active = false;
    }

    // Check servo faults
    if (_servo_state.feedback_valid && (_servo_state.error_flags != 0)) {
        _safety_state.servo_fault = true;
    } else {
        _safety_state.servo_fault = false;
    }

    // Check sensor faults
    if (_limit_sensors_enabled.get() && !_sensor_valid) {
        _safety_state.sensor_fault = true;
    } else {
        _safety_state.sensor_fault = false;
    }

    // Check position error
    float position_error = fabsf(_commanded_angle_rad - _current_angle_rad);
    if (position_error > _max_position_error.get()) {
        handle_safety_violation();
    }

    // Clear safety violation after timeout
    if (_safety_state.safety_violation &&
        (now - _safety_state.last_violation_time) > (_fault_timeout_ms.get() * 1000)) {
        if (_safety_state.violation_count < (uint32_t)_max_violations.get()) {
            _safety_state.safety_violation = false;
            _safety_state.position_limit_violation = false;
        }
    }
}

void SteeringController::check_command_timeout()
{
    if ((hrt_absolute_time() - _last_command_time) > (_command_timeout_ms.get() * 1000)) {
        _metrics.timeout_count++;
        _target_angle_rad = 0.0f;  // Return to center if no commands
    }
}

void SteeringController::update_status()
{
    steering_status_s status{};
    status.timestamp = hrt_absolute_time();

    // Current state
    status.steering_angle_deg = math::degrees(_current_angle_rad);
    status.steering_rate_deg_s = math::degrees(_servo_state.velocity_rad_s);
    status.steering_angle_setpoint_deg = math::degrees(_commanded_angle_rad);
    status.slip_compensation_deg = math::degrees(_slip_compensation_rad);

    // Servo state
    status.actual_angle_rad = _current_angle_rad;
    status.actual_rate_rad_s = _servo_state.velocity_rad_s;
    status.servo_position_rad = _current_angle_rad;
    status.servo_torque_nm = 0.0f; // Not available from ST3125
    status.steering_temperature_c = _servo_state.temperature_c;

    // Health status
    status.is_healthy = _servo_state.feedback_valid && (_servo_state.error_flags == 0);
    status.servo_healthy = _servo_state.feedback_valid && (_servo_state.error_flags == 0);
    status.position_valid = _servo_state.feedback_valid;
    status.emergency_stop = _safety_state.emergency_stop_active;
    status.emergency_stop_active = _safety_state.emergency_stop_active;

    // Limit sensors
    status.limit_left_active = _limit_sensors.left_limit_active;
    status.limit_right_active = _limit_sensors.right_limit_active;
    status.limit_sensors_healthy = _sensor_valid;

    // Safety state
    status.safety_violation = _safety_state.safety_violation;
    status.error_flags = _servo_state.error_flags;

    _steering_status_pub.publish(status);
}

int SteeringController::print_status()
{
    PX4_INFO("Steering Controller Status");
    PX4_INFO("  Current angle: %.1f deg", (double)math::degrees(_current_angle_rad));
    PX4_INFO("  Commanded: %.1f deg", (double)math::degrees(_commanded_angle_rad));
    PX4_INFO("  Target: %.1f deg", (double)math::degrees(_target_angle_rad));
    PX4_INFO("  Slip comp: %.1f deg", (double)math::degrees(_slip_compensation_rad));

    PX4_INFO("Servo State:");
    PX4_INFO("  Valid: %s", _servo_state.feedback_valid ? "YES" : "NO");
    PX4_INFO("  Current: %.2f A", (double)_servo_state.current_a);
    PX4_INFO("  Temp: %.1f C", (double)_servo_state.temperature_c);
    PX4_INFO("  Errors: 0x%04x", _servo_state.error_flags);

    PX4_INFO("Limit Sensors:");
    PX4_INFO("  Left: %s (healthy: %s)",
        _limit_sensors.left_limit_active ? "ACTIVE" : "inactive",
        _limit_sensors.left_limit_healthy ? "YES" : "NO");
    PX4_INFO("  Right: %s (healthy: %s)",
        _limit_sensors.right_limit_active ? "ACTIVE" : "inactive",
        _limit_sensors.right_limit_healthy ? "YES" : "NO");

    PX4_INFO("Safety:");
    PX4_INFO("  Violations: %lu", (unsigned long)_safety_state.violation_count);
    PX4_INFO("  Emergency stop: %s", _safety_state.emergency_stop_active ? "ACTIVE" : "inactive");

    PX4_INFO("Metrics:");
    PX4_INFO("  Commands: %lu", (unsigned long)_metrics.command_count);
    PX4_INFO("  Timeouts: %lu", (unsigned long)_metrics.timeout_count);
    PX4_INFO("  Max error: %.1f deg", (double)math::degrees(_metrics.max_error_rad));
    PX4_INFO("  Avg error: %.1f deg", (double)math::degrees(_metrics.avg_error_rad));

    return 0;
}

int SteeringController::task_spawn(int argc, char *argv[])
{
    SteeringController *instance = new SteeringController();

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

SteeringController *SteeringController::instantiate(int argc, char *argv[])
{
    return new SteeringController();
}

int SteeringController::custom_command(int argc, char *argv[])
{
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

Controls the ST3125 steering servo with:
- Position control with rate limiting
- Slip compensation from predictive traction
- Limit sensor monitoring
- Safety management with emergency stop

### Implementation
The controller runs at 50Hz and sends position commands to the ST3125 servo.
The servo handles its own PID control internally.

### Examples
$ steering_controller start
$ steering_controller status
$ steering_controller stop
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("steering_controller", "controller");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}
