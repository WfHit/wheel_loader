#include "wheel_controller.hpp"
#include <mathlib/mathlib.h>
#include <drivers/drv_hrt.h>

WheelController::WheelController(uint8_t instance, bool is_front) :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
    _instance(instance),
    _is_front_wheel(is_front),
    _loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
    _control_latency_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": latency")),
    _encoder_timeout_perf(perf_alloc(PC_COUNT, MODULE_NAME": enc_timeout"))
{
    // Initialize module status (0=front_wheel, 1=rear_wheel based on ModuleStatus.msg)
    _status.module_id = _is_front_wheel ? 0 : 1;
    _status.healthy = true;
    _status.initialized = false;
}

WheelController::~WheelController()
{
    perf_free(_loop_perf);
    perf_free(_control_latency_perf);
    perf_free(_encoder_timeout_perf);
}

bool WheelController::init()
{
    // Initialize PID controller - PX4 PID uses object-oriented approach
    // Set initial gains (will be updated from parameters)
    _speed_pid.setGains(1.0f, 0.0f, 0.0f);
    _speed_pid.setOutputLimit(100.0f);  // Set reasonable output limit
    _speed_pid.setIntegralLimit(10.0f); // Set reasonable integral limit

    // Update parameters
    parameters_update();

    // Schedule at 100Hz (matching existing rear wheel controller)
    ScheduleOnInterval(CONTROL_INTERVAL_US);

    _status.initialized = true;
    _status.healthy = true;
    _performance.last_health_check = hrt_absolute_time();

    return true;
}

void WheelController::Run()
{
    if (should_exit()) {
        ScheduleClear();
        exit_and_cleanup();
        return;
    }

    perf_begin(_loop_perf);

    // Check for parameter updates
    if (_parameter_update_sub.updated()) {
        parameter_update_s param_update;
        _parameter_update_sub.copy(&param_update);
        parameters_update();
    }

    // Update vehicle status for arming state
    vehicle_status_s vehicle_status;
    if (_vehicle_status_sub.copy(&vehicle_status)) {
        _armed = vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED;
    }

    // Process encoder data
    process_encoder_data();

    // Update wheel speed
    _current_speed_rpm = calculate_wheel_speed_rpm();

    // Apply speed filtering
    _current_speed_rpm = _speed_filter.apply(_current_speed_rpm);

    // Update speed control
    perf_begin(_control_latency_perf);
    update_speed_control();
    perf_end(_control_latency_perf);

    // Apply traction control
    apply_traction_control();

    // Check safety limits
    check_safety_limits();

    // Communicate with H-bridge
    communicate_with_hbridge();

    // Update controller status
    update_controller_status();

    // Update performance metrics
    update_performance_metrics();

    perf_end(_loop_perf);
}

void WheelController::process_encoder_data()
{
    sensor_quad_encoder_s encoder_data;

    if (_sensor_quad_encoder_sub.update(&encoder_data)) {
        const uint8_t encoder_idx = _instance; // 0 for front, 1 for rear

        if (encoder_idx < encoder_data.count && encoder_data.valid[encoder_idx]) {
            // Get encoder position and velocity from sensor_quad_encoder
            const int32_t current_position = encoder_data.position[encoder_idx];
            const float velocity_rad_s = encoder_data.velocity[encoder_idx];

            // Update encoder counts
            _encoder_count_prev = _encoder_count;
            _encoder_count = current_position;

            // Convert rad/s to RPM for internal calculations
            _current_speed_rpm = velocity_rad_s * 60.0f / (2.0f * M_PI_F);

            _last_encoder_time = encoder_data.timestamp;
            _controller_healthy = true; // No explicit health field, assume healthy if data received
        } else {
            perf_count(_encoder_timeout_perf);
            _performance.encoder_errors++;
        }
    } else {
        // Check for encoder timeout
        if (hrt_elapsed_time(&_last_encoder_time) > WATCHDOG_TIMEOUT_US) {
            perf_count(_encoder_timeout_perf);
            _controller_healthy = false;
            _emergency_stop = true;
        }
    }
}

float WheelController::calculate_wheel_speed_rpm()
{
    if (_last_encoder_time == 0) {
        return 0.0f;
    }

    const hrt_abstime now = hrt_absolute_time();
    const float dt = (now - _last_encoder_time) * 1e-6f; // Convert to seconds

    if (dt <= 0.0f || dt > 1.0f) {
        return _current_speed_rpm; // Return last known speed
    }

    // Calculate speed from encoder counts
    const int32_t delta_counts = _encoder_count - _encoder_count_prev;
    const float motor_revolutions = static_cast<float>(delta_counts) / _encoder_cpr.get();
    const float wheel_revolutions = motor_revolutions / _gear_ratio.get();
    const float wheel_rpm = (wheel_revolutions / dt) * 60.0f;

    return wheel_rpm;
}

void WheelController::update_speed_control()
{
    // Get speed setpoint
    wheel_speeds_setpoint_s setpoint;
    if (_wheel_speeds_setpoint_sub.update(&setpoint)) {
        // Convert rad/s to RPM for internal calculations
        _target_speed_rpm = _is_front_wheel ?
            setpoint.front_wheel_speed_rad_s * 60.0f / (2.0f * M_PI_F) :
            setpoint.rear_wheel_speed_rad_s * 60.0f / (2.0f * M_PI_F);
        _last_command_time = setpoint.timestamp;
    }

    // Apply rate limiting to setpoint
    const float rate_limit = _speed_ramp_rate.get() * 0.01f; // Convert to RPM per control cycle
    _target_speed_rpm = math::constrain(_target_speed_rpm,
                                       _current_speed_rpm - rate_limit,
                                       _current_speed_rpm + rate_limit);

    // Limit to max speed
    _target_speed_rpm = math::constrain(_target_speed_rpm,
                                       -_max_wheel_speed.get(),
                                       _max_wheel_speed.get());

    // PID control
    const float speed_error = _target_speed_rpm - _current_speed_rpm;

    // Update PID parameters
    _speed_pid.setGains(_speed_p_gain.get(),
                       _speed_i_gain.get(),
                       _speed_d_gain.get());
    _speed_pid.setIntegralLimit(_speed_i_max.get());
    _speed_pid.setOutputLimit(MAX_PWM_OUTPUT);
    _speed_pid.setSetpoint(_target_speed_rpm);

    // Calculate control output using PX4 PID
    const float dt = 0.01f; // 10ms control interval
    _pwm_output = _speed_pid.update(_current_speed_rpm, dt);

    // Apply traction limit
    _pwm_output *= _traction_limit_factor;

    // Update performance metrics
    _performance.speed_error_rms = sqrtf(0.95f * _performance.speed_error_rms * _performance.speed_error_rms +
                                        0.05f * speed_error * speed_error);
}

void WheelController::apply_traction_control()
{
    if (!_traction_control_enable.get()) {
        _traction_limit_factor = 1.0f;
        return;
    }

    // Subscribe to traction control commands
    traction_control_s traction_cmd;
    if (_traction_control_sub.update(&traction_cmd)) {
        _slip_detected = traction_cmd.slip_detected;

        // Apply torque reduction based on wheel instance
        if (_is_front_wheel) {
            _traction_limit_factor = 1.0f - traction_cmd.torque_reduction_front;
        } else {
            _traction_limit_factor = 1.0f - traction_cmd.torque_reduction_rear;
        }

        if (_slip_detected && _slip_start_time == 0) {
            _slip_start_time = hrt_absolute_time();
            _performance.slip_events++;
        } else if (!_slip_detected) {
            _slip_start_time = 0;
        }
    }

    // Local slip detection based on acceleration
    const float acceleration = (_current_speed_rpm - _encoder_count_prev) / 0.01f; // RPM/s
    if (fabsf(acceleration) > _slip_threshold.get() * 100.0f) {
        _slip_detected = true;
        _performance.slip_events++;

        // Reduce traction limit
        _traction_limit_factor = math::max(0.5f, _traction_limit_factor - 0.1f);
    } else if (_traction_limit_factor < 1.0f) {
        // Slowly restore traction
        _traction_limit_factor = math::min(1.0f, _traction_limit_factor + 0.01f);
    }
}

void WheelController::check_safety_limits()
{
    const hrt_abstime now = hrt_absolute_time();

    // Check command timeout
    if ((now - _last_command_time) > WATCHDOG_TIMEOUT_US) {
        _emergency_stop = true;
        _performance.safety_violations++;
    }

    // Check speed error
    const float speed_error = fabsf(_target_speed_rpm - _current_speed_rpm);
    if (speed_error > MAX_SPEED_ERROR_RPM) {
        _performance.max_speed_error = math::max(_performance.max_speed_error, speed_error);
        if (speed_error > MAX_SPEED_ERROR_RPM * 2.0f) {
            _emergency_stop = true;
            _performance.safety_violations++;
        }
    }

    // Check encoder health
    if ((now - _last_encoder_time) > WATCHDOG_TIMEOUT_US) {
        _emergency_stop = true;
        _controller_healthy = false;
    }

    // Simulate current monitoring (would come from actual sensor)
    _motor_current = _current_filter.apply(fabsf(_pwm_output) * 0.015f); // Rough estimate
    if (_motor_current > _current_limit.get()) {
        _emergency_stop = true;
        _performance.safety_violations++;
    }

    // Reset emergency stop if conditions clear
    if (_emergency_stop && !is_emergency_stop_active()) {
        reset_emergency_stop();
    }
}

bool WheelController::is_emergency_stop_active()
{
    const hrt_abstime now = hrt_absolute_time();

    // Check all emergency conditions
    if ((now - _last_command_time) > WATCHDOG_TIMEOUT_US) return true;
    if ((now - _last_encoder_time) > WATCHDOG_TIMEOUT_US) return true;
    if (fabsf(_target_speed_rpm - _current_speed_rpm) > MAX_SPEED_ERROR_RPM * 2.0f) return true;
    if (_motor_current > _current_limit.get()) return true;
    if (!_controller_healthy) return true;

    return false;
}

void WheelController::reset_emergency_stop()
{
    _emergency_stop = false;
    // Reset PID integrator
    _speed_pid.resetIntegral();
}

void WheelController::communicate_with_hbridge()
{
    if (!_armed || _emergency_stop) {
        _pwm_output = 0.0f;
    }

    // Saturate PWM output
    _pwm_output = saturate_pwm(_pwm_output);

    // Publish to H-bridge driver using hbridge_command
    hbridge_command_s cmd{};
    cmd.timestamp = hrt_absolute_time();
    cmd.channel = _instance; // 0 for front, 1 for rear

    // Convert normalized PWM to H-bridge format
    float normalized_output = _pwm_output / MAX_PWM_OUTPUT; // Normalize to [-1, 1]
    cmd.duty_cycle = normalized_output;  // Use duty_cycle directly (-1.0 to 1.0)
    cmd.enable = _armed && !_emergency_stop;

    _hbridge_command_pub.publish(cmd);

    // Also publish actuator_outputs for compatibility
    publish_actuator_outputs();
}

void WheelController::publish_actuator_outputs()
{
    actuator_outputs_s outputs{};
    outputs.timestamp = hrt_absolute_time();
    outputs.noutputs = actuator_outputs_s::NUM_ACTUATOR_OUTPUTS;

    for (int i = 0; i < actuator_outputs_s::NUM_ACTUATOR_OUTPUTS; i++) {
        outputs.output[i] = 0.0f;
    }

    if (_instance < actuator_outputs_s::NUM_ACTUATOR_OUTPUTS) {
        outputs.output[_instance] = _pwm_output;
    }

    _actuator_outputs_pub.publish(outputs);
}

float WheelController::saturate_pwm(float pwm_value)
{
    return math::constrain(pwm_value, -MAX_PWM_OUTPUT, MAX_PWM_OUTPUT);
}

void WheelController::update_controller_status()
{
    // Update module status for health reporting
    _status.timestamp = hrt_absolute_time();
    _status.healthy = _controller_healthy && !_emergency_stop;
    _status.armed = _armed;
    _status.initialized = true;

    // Calculate health score
    calculate_health_score();

    // Update status fields
    _status.cpu_usage_percent = 5.0f; // Placeholder - would come from actual measurement
    _status.memory_usage_bytes = 1024.0f; // Placeholder
    _status.temperature_c = 25.0f; // Would come from actual sensor
    _status.voltage_v = 12.0f; // Would come from actual measurement
    _status.current_draw_a = _motor_current;

    // Update counters
    _status.cycle_count++;
    _status.last_update_time = hrt_absolute_time();

    // Set performance level based on health
    if (_health_score > 90.0f) {
        _status.performance_level = 2; // Optimal
    } else if (_health_score > 70.0f) {
        _status.performance_level = 1; // Normal
    } else {
        _status.performance_level = 0; // Degraded
    }

    _status.control_quality = 1.0f - (_performance.speed_error_rms / 100.0f);
    _status.response_time_ms = 10.0f; // 100Hz = 10ms

    // Set emergency stop status
    _status.emergency_stop_active = _emergency_stop;

    // Set failure mode
    if (_performance.encoder_errors > 0) {
        _status.failure_mode = 1; // Sensor failure
    } else if (_motor_current > _current_limit.get()) {
        _status.failure_mode = 2; // Actuator failure
    } else if ((hrt_absolute_time() - _last_command_time) > WATCHDOG_TIMEOUT_US) {
        _status.failure_mode = 3; // Communication failure
    } else {
        _status.failure_mode = 0; // No failure
    }

    // Set diagnostic flags
    _status.diagnostic_flags = 0;
    if (_emergency_stop) _status.diagnostic_flags |= (1 << 0);
    if (_slip_detected) _status.diagnostic_flags |= (1 << 1);
    if (_performance.encoder_errors > 0) _status.diagnostic_flags |= (1 << 2);
    if (_motor_current > _current_limit.get()) _status.diagnostic_flags |= (1 << 3);

    _module_status_pub.publish(_status);
}

void WheelController::update_performance_metrics()
{
    // Update control effort average
    _performance.control_effort_avg = 0.95f * _performance.control_effort_avg +
                                     0.05f * fabsf(_pwm_output / MAX_PWM_OUTPUT);

    // Check if we missed any control cycles
    const hrt_abstime now = hrt_absolute_time();
    if ((now - _performance.last_health_check) > 2 * CONTROL_INTERVAL_US) {
        _performance.missed_updates++;
    }
    _performance.last_health_check = now;
}

void WheelController::calculate_health_score()
{
    float health = 100.0f;

    // Deduct points for various issues
    health -= _performance.safety_violations * 5.0f;
    health -= _performance.encoder_errors * 2.0f;
    health -= _performance.missed_updates * 1.0f;
    health -= _performance.slip_events * 0.5f;

    // Deduct for high RMS error
    if (_performance.speed_error_rms > 10.0f) {
        health -= (_performance.speed_error_rms - 10.0f) * 2.0f;
    }

    // Ensure health score is in valid range
    _health_score = math::constrain(health, 0.0f, 100.0f);
}

void WheelController::parameters_update()
{
    // Update local parameter cache
    _speed_p_gain.update();
    _speed_i_gain.update();
    _speed_d_gain.update();
    _speed_i_max.update();
    _max_wheel_speed.update();
    _speed_ramp_rate.update();
    _traction_control_enable.update();
    _slip_threshold.update();
    _current_limit.update();
    _gear_ratio.update();
    _encoder_cpr.update();
}

float WheelController::calculate_slip_ratio(float wheel_speed, float reference_speed)
{
    if (fabsf(reference_speed) < 0.1f) {
        return 0.0f; // Avoid division by zero
    }

    return (wheel_speed - reference_speed) / fabsf(reference_speed);
}

int WheelController::print_status()
{
    PX4_INFO("Wheel Controller %s (Instance %d)", _is_front_wheel ? "FRONT" : "REAR", _instance);
    PX4_INFO("  Status: %s, Health: %.1f%%",
             _controller_healthy ? "OK" : "ERROR",
             (double)_health_score);
    PX4_INFO("  Speed: %.1f RPM (Target: %.1f RPM)",
             (double)_current_speed_rpm,
             (double)_target_speed_rpm);
    PX4_INFO("  PWM Output: %.1f", (double)_pwm_output);
    PX4_INFO("  Emergency Stop: %s", _emergency_stop ? "ACTIVE" : "inactive");
    PX4_INFO("  Traction Control: %s (Limit: %.2f)",
             _traction_control_enable.get() ? "enabled" : "disabled",
             (double)_traction_limit_factor);
    PX4_INFO("  Performance:");
    PX4_INFO("    Speed Error RMS: %.2f RPM", (double)_performance.speed_error_rms);
    PX4_INFO("    Control Effort: %.1f%%", (double)(_performance.control_effort_avg * 100.0f));
    PX4_INFO("    Slip Events: %lu", _performance.slip_events);
    PX4_INFO("    Safety Violations: %lu", _performance.safety_violations);

    perf_print_counter(_loop_perf);
    perf_print_counter(_control_latency_perf);
    perf_print_counter(_encoder_timeout_perf);

    return 0;
}
