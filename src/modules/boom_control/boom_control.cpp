/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "boom_control.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

#include <lib/mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>

BoomControl::BoomControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
	// Initialize PID controller
	_position_pid.setOutputLimit(-1.0f, 1.0f);
}

BoomControl::~BoomControl()
{
	perf_free(_cycle_perf);
	perf_free(_interval_perf);
}

bool BoomControl::init()
{
	// Update parameters and initialize kinematics
	update_parameters();
	update_kinematics_from_params();

	// Initialize trajectory generator
	_trajectory_generator.setMaxVel(_param_boom_max_vel.get() / 1000.0f); // Convert mm/s to m/s
	_trajectory_generator.setMaxAccel(_param_boom_max_acc.get() / 1000.0f);
	_trajectory_generator.setMaxJerk(_param_boom_max_jerk.get() / 1000.0f);

	// Set initial target to current position
	_target_boom_angle = _param_boom_pos_carry.get();
	_target_actuator_length = boom_angle_to_actuator_length(_target_boom_angle);

	// Start scheduled execution
	int update_rate_hz = _param_update_rate.get();
	ScheduleOnInterval(1_s / update_rate_hz);

	return true;
}

void BoomControl::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);
	perf_count(_interval_perf);

	// Check for parameter updates
	parameter_update_s param_update;
	if (_parameter_update_sub.update(&param_update)) {
		update_parameters();
		update_kinematics_from_params();
	}

	// Update sensor data from AS5600
	update_sensor_data();

	// Process H-bridge channel status feedback
	process_hbridge_channel_status();

	// Process boom command inputs
	process_boom_command();

	// Update trajectory planning
	update_trajectory();

	// Run position control
	run_position_control();

	// Check safety limits
	check_limits_and_safety();

	// Publish H-bridge channel command
	publish_hbridge_channel_command();

	// Publish boom status
	publish_boom_status();

	perf_end(_cycle_perf);
}

void BoomControl::update_parameters()
{
	updateParams();

	// Update PID gains
	_position_pid.setKp(_param_boom_p.get());
	_position_pid.setKi(_param_boom_i.get());
	_position_pid.setKd(_param_boom_d.get());

	// Update trajectory generator limits
	_trajectory_generator.setMaxVel(_param_boom_max_vel.get() / 1000.0f);
	_trajectory_generator.setMaxAccel(_param_boom_max_acc.get() / 1000.0f);
	_trajectory_generator.setMaxJerk(_param_boom_max_jerk.get() / 1000.0f);
}

void BoomControl::update_kinematics_from_params()
{
	_kinematics.pivot_to_actuator_base = _param_kin_pivot_to_base.get();
	_kinematics.pivot_to_actuator_attach = _param_kin_pivot_to_attach.get();
	_kinematics.actuator_base_angle = math::radians(_param_kin_base_angle.get());
	_kinematics.min_actuator_length = _param_actuator_min_length.get();
	_kinematics.max_actuator_length = _param_actuator_max_length.get();
}

void BoomControl::update_sensor_data()
{
	sensor_mag_encoder_s mag_encoder_data;
	if (_mag_encoder_sub.update(&mag_encoder_data) && mag_encoder_data.device_id == static_cast<uint32_t>(_param_mag_encoder_instance_id.get())) {
		_last_sensor_update = hrt_absolute_time();
		_sensor_valid = (mag_encoder_data.magnet_detected == 1) &&
		                (mag_encoder_data.magnet_too_strong == 0) &&
		                (mag_encoder_data.magnet_too_weak == 0);

		// Magnetic encoder provides angle directly in radians
		float raw_angle_deg = math::degrees(mag_encoder_data.angle);

		// Apply calibration
		float calibrated_angle = raw_angle_deg * _param_mag_encoder_scale.get() + _param_mag_encoder_offset.get();

		// Ensure angle is in 0-360 range
		while (calibrated_angle < 0.0f) {
			calibrated_angle += 360.0f;
		}
		while (calibrated_angle >= 360.0f) {
			calibrated_angle -= 360.0f;
		}

		// Convert AS5600 angle to actuator length
		_current_actuator_length = as5600_angle_to_actuator_length(calibrated_angle);

		// Convert actuator length to boom angle
		_current_boom_angle = actuator_length_to_boom_angle(_current_actuator_length);
	} else if (is_sensor_timeout()) {
		_sensor_valid = false;
		emergency_stop();
	}
}

void BoomControl::process_hbridge_channel_status()
{
	hbridge_channel_s status{};
	if (_hbridge_channel_status_sub.update(&status) && status.channel_id == static_cast<uint8_t>(_param_hbridge_channel.get())) {
		// Check for faults from the H-bridge channel
		if (status.fault_overcurrent || status.fault_overheat) {
			PX4_WARN("H-bridge channel fault detected: channel %d, overcurrent: %d, overheat: %d",
					status.channel_id, status.fault_overcurrent, status.fault_overheat);
			_state = BoomState::ERROR;
			emergency_stop();
		}

		// Monitor channel state
		if (status.state == 2) { // FAULT state
			_state = BoomState::ERROR;
		}

		// Log runtime information (optional, can be removed for production)
		static uint64_t last_log_time = 0;
		if (hrt_elapsed_time(&last_log_time) > 5_s) {
			PX4_INFO("H-bridge channel status: channel %d, command %.2f, state %d, current %.2fA, temp %.1fC",
					status.channel_id, (double)status.current_command, status.state,
					(double)status.actual_current, (double)status.temperature);
			last_log_time = hrt_absolute_time();
		}
	}
}

float BoomControl::as5600_angle_to_actuator_length(float as5600_angle)
{
	// AS5600 measures the angle at the actuator pivot point
	// Use law of cosines to find actuator length

	float theta = math::radians(as5600_angle);
	float a = _kinematics.pivot_to_actuator_base;
	float b = _kinematics.pivot_to_actuator_attach;
	float gamma = _kinematics.actuator_base_angle + theta;

	// Law of cosines: c² = a² + b² - 2ab*cos(gamma)
	float length_squared = a * a + b * b - 2.0f * a * b * cosf(gamma);

	// Ensure we don't take square root of negative number
	if (length_squared < 0.0f) {
		length_squared = 0.0f;
	}

	return sqrtf(length_squared);
}

float BoomControl::boom_angle_to_actuator_length(float boom_angle)
{
	// Inverse kinematics: Given boom angle, find required actuator length

	float theta = math::radians(boom_angle);
	float a = _kinematics.pivot_to_actuator_base;
	float b = _kinematics.pivot_to_actuator_attach;

	// Calculate the angle between pivot-to-base and pivot-to-attach
	float gamma = _kinematics.actuator_base_angle - theta;

	// Use law of cosines to find actuator length
	float length_squared = a * a + b * b - 2.0f * a * b * cosf(gamma);

	if (length_squared < 0.0f) {
		length_squared = 0.0f;
	}

	return sqrtf(length_squared);
}

float BoomControl::actuator_length_to_boom_angle(float actuator_length)
{
	// Forward kinematics: Given actuator length, find boom angle

	float a = _kinematics.pivot_to_actuator_base;
	float b = _kinematics.pivot_to_actuator_attach;
	float c = actuator_length;

	// Law of cosines to find angle
	float cos_gamma = (a * a + b * b - c * c) / (2.0f * a * b);
	cos_gamma = math::constrain(cos_gamma, -1.0f, 1.0f); // Ensure valid range

	float gamma = acosf(cos_gamma);
	float boom_angle = math::degrees(_kinematics.actuator_base_angle - gamma);

	return boom_angle;
}

void BoomControl::process_boom_command()
{
	boom_command_s cmd;
	if (_boom_command_sub.update(&cmd)) {
		_last_command_time = hrt_absolute_time();

		// Handle emergency stop first
		if (cmd.emergency_stop) {
			emergency_stop();
			return;
		}

		// Process command based on mode
		switch (cmd.command_mode) {
		case 0: // Position control
			_target_boom_angle = cmd.target_angle;

			// Constrain to limits
			_target_boom_angle = math::constrain(_target_boom_angle,
			                                    _param_boom_min_angle.get(),
			                                    _param_boom_max_angle.get());

			// Update trajectory parameters
			_trajectory_generator.setMaxVelocity(cmd.max_velocity);
			_trajectory_generator.setMaxAcceleration(_param_boom_max_acc.get());
			_trajectory_generator.setMaxJerk(_param_boom_max_jerk.get());

			_state = BoomState::MOVING;
			break;

		case 1: // Velocity control
			// Direct velocity command - integrate to get position
			float dt = 0.01f; // Assume 100Hz update rate
			_target_boom_angle += cmd.target_angle * dt; // target_angle used as velocity

			// Constrain to limits
			_target_boom_angle = math::constrain(_target_boom_angle,
			                                    _param_boom_min_angle.get(),
			                                    _param_boom_max_angle.get());

			_state = BoomState::MOVING;
			break;

		case 2: // Force control (not implemented yet)
			PX4_WARN("Force control mode not implemented");
			break;

		default:
			PX4_WARN("Unknown boom command mode: %d", cmd.command_mode);
			break;
		}
	}
}

void BoomControl::set_target_position(BoomPreset preset)
{
	switch (preset) {
	case BoomPreset::GROUND:
		_target_boom_angle = _param_boom_pos_ground.get();
		break;
	case BoomPreset::CARRY:
		_target_boom_angle = _param_boom_pos_carry.get();
		break;
	case BoomPreset::MAX_HEIGHT:
		_target_boom_angle = _param_boom_pos_max.get();
		break;
	default:
		return; // Don't change target for unknown preset
	}

	_state = BoomState::MOVING;
}

void BoomControl::update_trajectory()
{
	if (_state == BoomState::ERROR) {
		return;
	}

	// Convert boom angle to actuator length for trajectory planning
	_target_actuator_length = boom_angle_to_actuator_length(_target_boom_angle);

	// Check actuator limits
	if (!check_actuator_limits(_target_actuator_length)) {
		PX4_WARN("Target actuator length %.1f mm exceeds limits [%.1f, %.1f]",
		         (double)_target_actuator_length,
		         (double)_kinematics.min_actuator_length,
		         (double)_kinematics.max_actuator_length);
		return;
	}

	// Update S-curve trajectory generator
	_trajectory_generator.setCurrentPosition(_current_actuator_length / 1000.0f); // Convert to meters
	_trajectory_generator.setTargetPosition(_target_actuator_length / 1000.0f);
	_trajectory_generator.update(1.0f / _param_update_rate.get()); // dt in seconds
}

void BoomControl::run_position_control()
{
	if (_state == BoomState::ERROR || !_sensor_valid) {
		_motor_output = 0.0f;
		return;
	}

	// Get smooth position from trajectory generator (in meters)
	float desired_position_m = _trajectory_generator.getCurrentPosition();
	float desired_velocity_m_s = _trajectory_generator.getCurrentVelocity();

	// Convert back to mm for control
	float desired_position_mm = desired_position_m * 1000.0f;

	// Position error in mm
	float position_error = desired_position_mm - _current_actuator_length;

	// Check if we've reached the target
	if (fabsf(position_error) < 5.0f && fabsf(desired_velocity_m_s) < 0.001f) { // 5mm tolerance
		_state = BoomState::HOLDING;
	}

	// PID control
	float dt = 1.0f / _param_update_rate.get();
	_motor_output = _position_pid.update(position_error, dt);

	// Add velocity feedforward
	float velocity_ff = desired_velocity_m_s / (_param_boom_max_vel.get() / 1000.0f);
	_motor_output += velocity_ff * 0.3f; // Feedforward gain

	// Apply deadzone compensation
	if (fabsf(_motor_output) > _param_motor_deadzone.get()) {
		if (_motor_output > 0.0f) {
			_motor_output += _param_motor_deadzone.get();
		} else {
			_motor_output -= _param_motor_deadzone.get();
		}
	}

	// Constrain output
	_motor_output = math::constrain(_motor_output, -1.0f, 1.0f);
}

void BoomControl::check_limits_and_safety()
{
	// Check boom angle limits
	_lower_limit_reached = _current_boom_angle <= _param_boom_min_angle.get();
	_upper_limit_reached = _current_boom_angle >= _param_boom_max_angle.get();

	// Apply software limits
	if (_lower_limit_reached && _motor_output < 0.0f) {
		_motor_output = 0.0f;
		PX4_WARN("Lower boom limit reached");
	}

	if (_upper_limit_reached && _motor_output > 0.0f) {
		_motor_output = 0.0f;
		PX4_WARN("Upper boom limit reached");
	}

	// Check actuator limits
	if (_current_actuator_length <= _kinematics.min_actuator_length && _motor_output < 0.0f) {
		_motor_output = 0.0f;
	}

	if (_current_actuator_length >= _kinematics.max_actuator_length && _motor_output > 0.0f) {
		_motor_output = 0.0f;
	}

	// Check for sensor timeout
	if (is_sensor_timeout()) {
		emergency_stop();
	}
}

void BoomControl::publish_hbridge_channel_command()
{
	hbridge_channel_cmd_s cmd{};
	cmd.timestamp = hrt_absolute_time();

	// Set channel from parameter
	cmd.channel_id = static_cast<uint8_t>(_param_hbridge_channel.get());

	// Apply deadzone to prevent motor hunting around zero
	float output = _motor_output;
	if (fabsf(output) < _param_motor_deadzone.get()) {
		output = 0.0f;
	}

	// Set command value (-1.0 to 1.0 range)
	// Positive = extend actuator (boom up), Negative = retract actuator (boom down)
	cmd.command_value = math::constrain(output, -1.0f, 1.0f);

	// Enable channel if not in error state
	cmd.enable = (_state != BoomState::ERROR);

	// Set control mode based on state
	switch (_state) {
		case BoomState::IDLE:
			cmd.command_mode = 0; // IDLE/POSITION_HOLD
			break;
		case BoomState::MOVING:
		case BoomState::HOLDING:
			cmd.command_mode = 1; // VELOCITY
			break;
		case BoomState::ERROR:
			cmd.command_mode = 0; // IDLE
			cmd.enable = false;
			break;
	}

	// Set current limit and safety
	cmd.max_current = 10.0f; // 10A limit
	cmd.ignore_limits = false; // Respect limit switches
	cmd.priority = 1; // Normal priority

	_hbridge_channel_cmd_pub.publish(cmd);
}

void BoomControl::publish_boom_status()
{
	boom_status_s status{};
	status.timestamp = hrt_absolute_time();

	// Current position and state
	status.current_angle = _current_boom_angle;
	status.target_angle = _target_boom_angle;
	status.current_actuator_length = _current_actuator_length;
	status.target_actuator_length = _target_actuator_length;

	// Control state
	switch (_state) {
		case BoomState::IDLE:
			status.state = 0;
			break;
		case BoomState::MOVING:
			status.state = 1;
			break;
		case BoomState::HOLDING:
			status.state = 2;
			break;
		case BoomState::ERROR:
			status.state = 3;
			break;
	}

	// Motion status
	status.is_moving = (_state == BoomState::MOVING);
	status.at_target = (_state == BoomState::HOLDING);

	// Limits and safety
	status.lower_limit_reached = _lower_limit_reached;
	status.upper_limit_reached = _upper_limit_reached;
	status.sensor_valid = _sensor_valid;

	// Motor output and control
	status.motor_output = _motor_output;
	status.last_command_time = _last_command_time;
	status.last_sensor_update = _last_sensor_update;

	_boom_status_pub.publish(status);
}

bool BoomControl::check_actuator_limits(float length)
{
	return (length >= _kinematics.min_actuator_length && length <= _kinematics.max_actuator_length);
}

bool BoomControl::check_boom_limits(float angle)
{
	return (angle >= _param_boom_min_angle.get() && angle <= _param_boom_max_angle.get());
}

void BoomControl::emergency_stop()
{
	_motor_output = 0.0f;
	_state = BoomState::ERROR;
	PX4_ERR("Emergency stop activated");
}

bool BoomControl::is_sensor_timeout()
{
	return (hrt_absolute_time() - _last_sensor_update) > 500_ms; // 500ms timeout
}

void BoomControl::reset_trajectory()
{
	_trajectory_generator.setCurrentPosition(_current_actuator_length / 1000.0f);
	_trajectory_generator.setTargetPosition(_current_actuator_length / 1000.0f);
}

int BoomControl::task_spawn(int argc, char *argv[])
{
	BoomControl *instance = new BoomControl();

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

int BoomControl::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	if (!strcmp(argv[0], "preset")) {
		if (argc < 2) {
			PX4_ERR("usage: boom_control preset <ground|carry|max>");
			return 1;
		}

		BoomPreset preset;
		if (!strcmp(argv[1], "ground")) {
			preset = BoomPreset::GROUND;
		} else if (!strcmp(argv[1], "carry")) {
			preset = BoomPreset::CARRY;
		} else if (!strcmp(argv[1], "max")) {
			preset = BoomPreset::MAX_HEIGHT;
		} else {
			PX4_ERR("Unknown preset: %s", argv[1]);
			return 1;
		}

		get_instance()->set_target_position(preset);
		return 0;
	}

	return print_usage("unknown command");
}

int BoomControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Boom control module for wheel loader.

Controls the boom angle using AS5600 sensor feedback and DC motor with H-bridge (DRV8701).
Includes proper kinematics transformation, S-curve motion planning, and PID position control.

### Implementation
The module uses triangle-based kinematics to convert between boom angles and actuator lengths.
The AS5600 magnetic encoder provides position feedback at the actuator pivot point.
A PID controller with S-curve trajectory generation provides smooth and precise motion.

### Examples
To set boom to carry position:
$ boom_control preset carry

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("boom_control", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_COMMENT("Optional parameters:");
	PRINT_MODULE_USAGE_COMMAND_DESCR("preset", "Set boom to preset position (ground|carry|max)");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int boom_control_main(int argc, char *argv[])
{
	return BoomControl::main(argc, argv);
}
