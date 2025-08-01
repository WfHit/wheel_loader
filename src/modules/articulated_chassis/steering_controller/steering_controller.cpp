#include "steering_controller.hpp"
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/module.h>
#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>

#define MODULE_NAME "steering_controller"

SteeringController::SteeringController() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl),
	_servo_controller(this),
	_slip_compensator(this),
	_limit_sensor(this),
	_safety_manager(this)
{
}

bool SteeringController::init()
{
	// Schedule at desired rate
	ScheduleOnInterval(CONTROL_INTERVAL_US);

	PX4_INFO("Steering Controller initialized with modular design");
	PX4_INFO("Max angle: ±%.1f°, Max rate: %.1f°/s",
		 (double)math::degrees(_max_steering_angle.get()),
		 (double)math::degrees(_max_steering_rate.get()));

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

	// Update all components
	_servo_controller.update_feedback(_dt);
	_slip_compensator.update();
	_limit_sensor.update();

	// Get current position from servo controller
	_current_angle_rad = _servo_controller.get_filtered_position();

	// Process steering commands
	process_steering_command();

	// Update safety state
	_safety_manager.update_safety_state(_servo_controller, _limit_sensor,
					    _commanded_angle_rad, _current_angle_rad);

	// Handle safety violations
	if (!_safety_manager.is_safe()) {
		_safety_manager.handle_safety_violation(_servo_controller);
	}

	// Check command timeout
	check_command_timeout();

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
		if (_slip_compensator.is_active()) {
			_target_angle_rad = _slip_compensator.apply_compensation(_target_angle_rad);
		}

		// Apply feedforward
		float feedforward_angle = apply_feedforward(_target_angle_rad, _current_angle_rad);

		// Rate limit
		float limited_angle = rate_limit(feedforward_angle, _commanded_angle_rad);

		// Check position limits
		if (_limit_sensor.check_position_limits(limited_angle, _max_steering_angle.get())) {
			// Apply deadband
			if (fabsf(limited_angle) < _steering_deadband.get()) {
				limited_angle = 0.0f;
			}

			_commanded_angle_rad = limited_angle;

			// Calculate velocity command for feedforward
			float velocity_cmd = 0.0f;

			if (_feedforward_gain.get() > 0.0f) {
				velocity_cmd = (_target_angle_rad - _current_angle_rad) * _feedforward_gain.get();
			}

			// Send command to servo
			_servo_controller.send_position_command(_commanded_angle_rad, velocity_cmd);

			// Update metrics
			float error = fabsf(_commanded_angle_rad - _current_angle_rad);
			_metrics.max_error_rad = math::max(_metrics.max_error_rad, error);
			_metrics.avg_error_rad = (_metrics.avg_error_rad * 0.95f) + (error * 0.05f);
		}
	}
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
	status.steering_angle_setpoint_deg = math::degrees(_commanded_angle_rad);

	// Get component states
	const auto &servo_state = _servo_controller.get_state();
	const auto &slip_state = _slip_compensator.get_state();
	const auto &limit_state = _limit_sensor.get_state();
	const auto &safety_state = _safety_manager.get_state();

	// Servo state
	status.steering_rate_deg_s = math::degrees(servo_state.velocity_rad_s);
	status.actual_angle_rad = _current_angle_rad;
	status.actual_rate_rad_s = servo_state.velocity_rad_s;
	status.servo_position_rad = _current_angle_rad;
	status.servo_torque_nm = 0.0f; // Not available from ST3125
	status.steering_temperature_c = servo_state.temperature_c;

	// Slip compensation
	status.slip_compensation_deg = math::degrees(slip_state.compensation_rad);

	// Health status
	status.is_healthy = _servo_controller.is_healthy();
	status.servo_healthy = _servo_controller.is_healthy();
	status.position_valid = servo_state.feedback_valid;
	status.emergency_stop = safety_state.emergency_stop_active;
	status.emergency_stop_active = safety_state.emergency_stop_active;

	// Limit sensors
	status.limit_left_active = limit_state.left_limit_active;
	status.limit_right_active = limit_state.right_limit_active;
	status.limit_sensors_healthy = _limit_sensor.are_sensors_healthy();

	// Safety state
	status.safety_violation = safety_state.safety_violation;
	status.error_flags = servo_state.error_flags;

	_steering_status_pub.publish(status);
}

int SteeringController::print_status()
{
	PX4_INFO("Steering Controller Status (Modular Design)");
	PX4_INFO("  Current angle: %.1f deg", (double)math::degrees(_current_angle_rad));
	PX4_INFO("  Commanded: %.1f deg", (double)math::degrees(_commanded_angle_rad));
	PX4_INFO("  Target: %.1f deg", (double)math::degrees(_target_angle_rad));

	// Component states
	const auto &servo_state = _servo_controller.get_state();
	const auto &slip_state = _slip_compensator.get_state();
	const auto &limit_state = _limit_sensor.get_state();
	const auto &safety_state = _safety_manager.get_state();

	PX4_INFO("Servo State:");
	PX4_INFO("  Valid: %s", servo_state.feedback_valid ? "YES" : "NO");
	PX4_INFO("  Current: %.2f A", (double)servo_state.current_a);
	PX4_INFO("  Temp: %.1f C", (double)servo_state.temperature_c);
	PX4_INFO("  Errors: 0x%04x", servo_state.error_flags);

	PX4_INFO("Slip Compensation:");
	PX4_INFO("  Active: %s", _slip_compensator.is_active() ? "YES" : "NO");
	PX4_INFO("  Compensation: %.1f deg", (double)math::degrees(slip_state.compensation_rad));

	PX4_INFO("Limit Sensors:");
	PX4_INFO("  Enabled: %s", _limit_sensor.are_enabled() ? "YES" : "NO");
	PX4_INFO("  Left: %s (healthy: %s)",
		 limit_state.left_limit_active ? "ACTIVE" : "inactive",
		 limit_state.left_limit_healthy ? "YES" : "NO");
	PX4_INFO("  Right: %s (healthy: %s)",
		 limit_state.right_limit_active ? "ACTIVE" : "inactive",
		 limit_state.right_limit_healthy ? "YES" : "NO");

	PX4_INFO("Safety Manager:");
	PX4_INFO("  Enabled: %s", _safety_manager.is_enabled() ? "YES" : "NO");
	PX4_INFO("  Safe: %s", _safety_manager.is_safe() ? "YES" : "NO");
	PX4_INFO("  Violations: %lu", (unsigned long)safety_state.violation_count);
	PX4_INFO("  Emergency stop: %s", safety_state.emergency_stop_active ? "ACTIVE" : "inactive");

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
Modular steering controller for articulated wheel loader.

Refactored with composition-based architecture:
- SteeringServoController: ST3125 servo communication
- SteeringSlipCompensator: Slip compensation logic
- SteeringLimitSensor: Limit sensor monitoring
- SteeringSafetyManager: Safety management and emergency stops

### Implementation
The controller runs at 50Hz using focused, testable components.
Each component handles a specific responsibility, improving maintainability.

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
