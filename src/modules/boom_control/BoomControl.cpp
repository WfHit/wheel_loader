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

#include "BoomControl.hpp"

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

	// Process H-bridge status feedback
	process_hbridge_status();

	// Process control inputs
	process_manual_control();
	process_vehicle_commands();

	// Update trajectory planning
	update_trajectory();

	// Run position control
	run_position_control();

	// Check safety limits
	check_limits_and_safety();

	// Publish H-bridge command
	publish_hbridge_command();

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
	sensor_mag_s mag_data;
	if (_as5600_sub.update(&mag_data) && mag_data.device_id == static_cast<uint32_t>(_param_as5600_instance_id.get())) {
		_last_sensor_update = hrt_absolute_time();
		_sensor_valid = true;

		// AS5600 provides angle at actuator pivot
		// Convert raw magnetometer reading to angle (AS5600 uses x-axis for angle)
		float raw_angle_deg = atan2f(mag_data.y, mag_data.x) * 180.0f / M_PI_F;

		// Apply calibration
		float calibrated_angle = raw_angle_deg * _param_as5600_scale.get() + _param_as5600_offset.get();

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

void BoomControl::process_hbridge_status()
{
	hbridge_status_s status{};
	if (_hbridge_status_sub.update(&status) && status.channel == _param_hbridge_channel.get()) {
		// Check for faults from the H-bridge driver
		if (status.fault) {
			PX4_WARN("H-bridge fault detected: channel %d, fault code %d", status.channel, status.fault_code);
			_state = BoomState::ERROR;
			emergency_stop();
		}

		// Monitor driver state
		if (status.state == 2) { // FAULT state
			_state = BoomState::ERROR;
		}

		// Log runtime information (optional, can be removed for production)
		static uint64_t last_log_time = 0;
		if (hrt_elapsed_time(&last_log_time) > 5_s) {
			PX4_INFO("H-bridge status: channel %d, duty %.2f, state %d, runtime %d ms",
					status.channel, (double)status.actual_duty, status.state, status.runtime_ms);
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

void BoomControl::process_manual_control()
{
	manual_control_setpoint_s manual_control;
	if (_manual_control_sub.update(&manual_control)) {
		_last_command_time = hrt_absolute_time();

		// Use aux1 channel for boom control (-1 to 1)
		// Map to angle range
		float normalized = (manual_control.aux1 + 1.0f) * 0.5f; // Convert to 0-1 range
		_target_boom_angle = _param_boom_min_angle.get() +
		                    normalized * (_param_boom_max_angle.get() - _param_boom_min_angle.get());

		// Constrain to limits
		_target_boom_angle = math::constrain(_target_boom_angle,
		                                    _param_boom_min_angle.get(),
		                                    _param_boom_max_angle.get());

		_state = BoomState::MOVING;
	}
}

void BoomControl::process_vehicle_commands()
{
	vehicle_command_s cmd;
	if (_vehicle_command_sub.update(&cmd)) {
		_last_command_time = hrt_absolute_time();

		switch (cmd.command) {
		case vehicle_command_s::VEHICLE_CMD_DO_SET_ACTUATOR:
			// param1: actuator group, param2: actuator index, param3: value
			if (static_cast<int>(cmd.param2) == _param_hbridge_channel.get()) {
				_target_boom_angle = cmd.param3;
				_state = BoomState::MOVING;
				publish_command_ack(cmd.command, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
			}
			break;

		case vehicle_command_s::VEHICLE_CMD_CUSTOM_0:
			// Custom command for preset positions
			set_target_position(static_cast<BoomPreset>(static_cast<int>(cmd.param1)));
			publish_command_ack(cmd.command, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
			break;

		case vehicle_command_s::VEHICLE_CMD_CUSTOM_1:
			// Emergency stop
			emergency_stop();
			publish_command_ack(cmd.command, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
			break;

		default:
			publish_command_ack(cmd.command, vehicle_command_ack_s::VEHICLE_CMD_RESULT_UNSUPPORTED);
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

void BoomControl::publish_hbridge_command()
{
	hbridge_cmd_s cmd{};
	cmd.timestamp = hrt_absolute_time();

	// Set channel from parameter
	cmd.channel = _param_hbridge_channel.get();

	// Apply deadzone to prevent motor hunting around zero
	float output = _motor_output;
	if (fabsf(output) < _param_motor_deadzone.get()) {
		output = 0.0f;
	}

	// Set speed and direction based on motor output
	// Positive = extend actuator (boom up), Negative = retract actuator (boom down)
	cmd.speed = fabsf(output);
	cmd.direction = (output >= 0.0f) ? 0 : 1;  // 0: forward (extend), 1: reverse (retract)

	// Enable channel if not in error state
	cmd.enable_request = (_state != BoomState::ERROR);

	// Set control mode based on state
	switch (_state) {
		case BoomState::IDLE:
			cmd.control_mode = 0; // IDLE
			break;
		case BoomState::MOVING:
		case BoomState::HOLDING:
			cmd.control_mode = 1; // NORMAL
			break;
		case BoomState::ERROR:
			cmd.control_mode = 0; // IDLE
			break;
	}

	// Set timeout and limits
	cmd.timeout_ms = 500; // 500ms timeout
	cmd.current_limit = 10.0f; // 10A limit (for future use)

	_hbridge_cmd_pub.publish(cmd);
}

void BoomControl::publish_command_ack(uint16_t command, uint8_t result)
{
	vehicle_command_ack_s ack{};
	ack.timestamp = hrt_absolute_time();
	ack.command = command;
	ack.result = result;

	_vehicle_command_ack_pub.publish(ack);
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
