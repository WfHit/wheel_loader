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

#include "bucket_control.hpp"
#include "bucket_kinematics.hpp"
#include "bucket_hardware_interface.hpp"
#include "bucket_motion_controller.hpp"
#include "bucket_state_manager.hpp"

#include <px4_platform_common/log.h>
#include <mathlib/mathlib.h>
#include <perf/perf_counter.h>
#include <cmath>

BucketControl::BucketControl() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME))
{
	// Create component instances
	_kinematics = new BucketKinematics(this);
	_hardware_interface = new BucketHardwareInterface(this);
	_motion_controller = new BucketMotionController(this);
	_state_manager = new BucketStateManager(this);
}

BucketControl::~BucketControl()
{
	// Manual cleanup
	delete _kinematics;
	delete _hardware_interface;
	delete _motion_controller;
	delete _state_manager;

	perf_end(_cycle_perf);
	perf_free(_cycle_perf);
}

bool BucketControl::init()
{
	// Load parameters first
	update_parameters();

	// Check if module is enabled
	if (_param_enabled.get() == 0) {
		PX4_INFO("Bucket control module disabled");
		return false;
	}

	// Initialize hardware interface
	uint8_t motor_index = static_cast<uint8_t>(_param_motor_index.get());
	uint8_t encoder_index = static_cast<uint8_t>(_param_encoder_index.get());

	if (!_hardware_interface->initialize(motor_index, encoder_index)) {
		PX4_ERR("Failed to initialize hardware interface");
		return false;
	}

	// Validate and update kinematic configuration
	_kinematics->update_configuration();
	if (!_kinematics->validate_configuration()) {
		PX4_ERR("Invalid kinematic configuration");
		return false;
	}

	// Initialize motion controller with parameter-based configuration
	// Set safety limits first from hardware parameters
	_motion_controller->set_safety_limits(_param_hbg_min_len.get(), _param_hbg_max_len.get());

	// Initialize from parameters (will call updateParams internally)
	_motion_controller->update_parameters();

	PX4_INFO("Motion controller initialized from parameter system");

	// Perform hardware self-test
	if (!_hardware_interface->perform_self_test()) {
		PX4_ERR("Hardware self-test failed");
		return false;
	}

	// Start periodic execution
	float update_rate = _param_update_rate.get();
	uint32_t interval_us = static_cast<uint32_t>(1000000.0f / update_rate);
	interval_us = math::max(interval_us, CONTROL_INTERVAL_US);  // Minimum 50 Hz

	ScheduleOnInterval(interval_us);

	PX4_INFO("Bucket control initialized (update rate: %.1f Hz)", (double)update_rate);
	return true;
}

void BucketControl::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	// Clean, sequential control flow
	update_sensor_data();
	process_commands();
	execute_control();
	publish_telemetry();
}

void BucketControl::update_sensor_data()
{
	// Check for parameter updates
	parameter_update_s param_update;
	if (_parameter_update_sub.update(&param_update)) {
		update_parameters();
		_kinematics->update_configuration();

		// Update motion controller configuration if needed
		// This could reload PID gains, motion limits, etc.
	}

	// Read hardware sensors
	BucketHardwareInterface::SensorData sensor_data;
	bool sensors_valid = _hardware_interface->update_sensors(sensor_data);

	// Check for command timeout
	bool command_timeout = false;
	if (_last_command_time != 0) {
		command_timeout = (hrt_elapsed_time(&_last_command_time) > COMMAND_TIMEOUT_US);
	}

	// Update state manager with sensor status
	_state_manager->update(
		sensors_valid,
		_hardware_interface->is_healthy(),
		command_timeout,
		sensor_data.hbridge_position,
		sensor_data.limit_switch_load,
		sensor_data.limit_switch_dump
	);
}

void BucketControl::process_commands()
{
	bucket_command_s command;
	bool new_command = _bucket_command_sub.update(&command);

	if (!new_command) {
		return;
	}

	_last_command_time = hrt_absolute_time();

	// Process command based on current state
	auto state_info = _state_manager->get_state_info();

	// Handle calibration command regardless of state
	if (command.control_mode == bucket_command_s::MODE_CALIBRATE) {
		_state_manager->start_calibration();
		return;
	}

	// Check if system is operational for other commands
	if (!_state_manager->is_operational()) {
		PX4_WARN("Cannot process command - system not operational (state: %d)",
			 static_cast<int>(state_info.state));
		return;
	}

	// Handle different command types
	switch (command.control_mode) {
		case bucket_command_s::MODE_POSITION:
			// Absolute bucket angle command (chassis-relative)
			_target_bucket_angle_chassis = command.target_angle;

			// Transition to active state
			_state_manager->request_state_transition(
				BucketStateManager::OperationalState::ACTIVE,
				"Position command received"
			);

			PX4_DEBUG("Chassis-relative position command: %.2f°",
				 (double)math::degrees(_target_bucket_angle_chassis));
			break;

		case bucket_command_s::MODE_DIRECT:
			// Direct actuator position command
			_commanded_actuator_position = command.target_angle;

			_state_manager->request_state_transition(
				BucketStateManager::OperationalState::ACTIVE,
				"Direct position command received"
			);

			PX4_DEBUG("Direct position command: %.1f mm", (double)_commanded_actuator_position);
			break;

		case bucket_command_s::MODE_STOP:
			// Stop command - transition to ready
			_state_manager->request_state_transition(
				BucketStateManager::OperationalState::READY,
				"Stop command received"
			);
			break;

		case bucket_command_s::MODE_EMERGENCY_STOP:
			// Emergency stop
			_state_manager->trigger_emergency_stop("Command emergency stop");
			break;

		default:
			PX4_WARN("Unknown control mode: %d", command.control_mode);
			break;
	}
}

void BucketControl::execute_control()
{
	auto state_info = _state_manager->get_state_info();

	// Handle emergency stop
	if (state_info.state == BucketStateManager::OperationalState::EMERGENCY_STOP) {
		_hardware_interface->emergency_stop();
		return;
	}

	// Get current sensor data
	BucketHardwareInterface::SensorData sensor_data;
	if (!_hardware_interface->update_sensors(sensor_data)) {
		PX4_DEBUG("No valid sensor data for control");
		return;
	}

	// Determine target position based on current state
	float target_actuator_position = 0.0f;
	if (!determine_target_position(state_info, sensor_data, target_actuator_position)) {
		send_zero_command();
		return;
	}

	// Execute motion control
	execute_motion_control(target_actuator_position, sensor_data);
}

void BucketControl::publish_telemetry()
{
	bucket_status_s status{};
	status.timestamp = hrt_absolute_time();

	// Get state information
	auto state_info = _state_manager->get_state_info();

	// Get sensor data
	BucketHardwareInterface::SensorData sensor_data;
	bool sensors_valid = _hardware_interface->update_sensors(sensor_data);

	// Calculate boom angle from sensor angle using triangle geometry
	float boom_angle = 0.0f;
	if (sensors_valid && sensor_data.sensor_angle_valid) {
		boom_angle = _kinematics->encoder_angle_to_boom_angle(sensor_data.sensor_angle);
	}

	if (sensors_valid) {
		// Compute current bucket angle using forward kinematics
		auto linkage_state = _kinematics->compute_forward_kinematics(
			sensor_data.hbridge_position,
			boom_angle
		);

		// Populate sensor data fields
		status.bucket_angle = linkage_state.bucket_angle;
		status.actuator_length = sensor_data.hbridge_position;
		status.target_actuator_length = _commanded_actuator_position;
		status.velocity = sensor_data.hbridge_velocity;
		status.limit_switch_load = sensor_data.limit_switch_load;
		status.limit_switch_dump = sensor_data.limit_switch_dump;
		status.boom_angle = boom_angle;
	}

	// Populate state and control information
	status.state = static_cast<uint8_t>(state_info.state);
	status.calibration_phase = static_cast<uint8_t>(state_info.calibration_phase);
	status.error_flags = state_info.error_flags;
	status.calibration_progress = static_cast<uint8_t>(state_info.calibration_progress * 100.0f);

	// Control mode information - convert chassis-relative to ground-relative for status
	status.target_ground_angle = _target_bucket_angle_chassis + boom_angle;

	// Hardware status
	bool motor_enabled, encoder_valid, limits_valid;
	if (_hardware_interface->get_hardware_status(motor_enabled, encoder_valid, limits_valid)) {
		status.motor_enabled = motor_enabled;
		status.encoder_valid = encoder_valid;
		status.limits_valid = limits_valid;
	}

	// Performance metrics
	float pos_rms_error, vel_rms_error, control_effort;
	_motion_controller->get_performance_metrics(pos_rms_error, vel_rms_error, control_effort);
	status.position_error_rms = pos_rms_error;
	status.velocity_error_rms = vel_rms_error;
	status.control_effort = control_effort;

	_bucket_status_pub.publish(status);
}

void BucketControl::update_parameters()
{
	ModuleParams::updateParams();

	// Update scheduling rate if changed
	float update_rate = _param_update_rate.get();
	uint32_t new_interval_us = static_cast<uint32_t>(1000000.0f / update_rate);
	new_interval_us = math::max(new_interval_us, CONTROL_INTERVAL_US);  // Minimum 50 Hz

	// Update the scheduling interval (will take effect on next schedule)
	ScheduleOnInterval(new_interval_us);
	PX4_DEBUG("Control rate set to %.1f Hz (interval: %u us)",
		(double)(1000000.0f / new_interval_us), new_interval_us);

	// Update kinematic configuration
	if (_kinematics) {
		_kinematics->update_configuration();
		PX4_DEBUG("Updated kinematic configuration");
	}

	// Update motion controller configuration
	if (_motion_controller) {
		// Update safety limits from hardware parameters
		_motion_controller->set_safety_limits(_param_hbg_min_len.get(), _param_hbg_max_len.get());

		// Let the motion controller update its own parameters
		_motion_controller->update_parameters();

		PX4_DEBUG("Motion controller parameters updated via parameter system");
	}

	// Update hardware interface configuration if needed
	if (_hardware_interface) {
		// Update hardware interface parameters
		_hardware_interface->update_parameters();
		PX4_DEBUG("Hardware interface parameters updated");
	}

	// Update state manager configuration if needed
	if (_state_manager) {
		// Update state manager parameters
		_state_manager->update_parameters();
		PX4_DEBUG("State manager parameters updated");
	}
}

bool BucketControl::determine_target_position(
	const BucketStateManager::StateInfo& state_info,
	const BucketHardwareInterface::SensorData& sensor_data,
	float& target_actuator_position)
{
	// Calculate boom angle from sensor angle using triangle geometry
	float boom_angle = 0.0f;
	if (sensor_data.sensor_angle_valid) {
		boom_angle = _kinematics->encoder_angle_to_boom_angle(sensor_data.sensor_angle);
	}

	// Determine target position based on state and mode
	if (state_info.state == BucketStateManager::OperationalState::CALIBRATING) {
		// Use calibration command
		float cal_pos, cal_vel;
		if (_state_manager->get_calibration_command(cal_pos, cal_vel)) {
			target_actuator_position = cal_pos;
			return true;
		}

	} else if (state_info.state == BucketStateManager::OperationalState::ACTIVE) {
		// Use operational commands - determine if it's position or direct mode
		if (fabsf(_target_bucket_angle_chassis) > 1e-6f) {
			// Position mode: maintain chassis-relative bucket angle using kinematics
			// Convert chassis-relative to ground-absolute for kinematics
			float target_ground_angle = _target_bucket_angle_chassis + boom_angle;
			auto target_linkage = _kinematics->compute_inverse_kinematics(
				target_ground_angle,
				boom_angle
			);

			if (target_linkage.is_valid) {
				target_actuator_position = target_linkage.actuator_length;
				return true;
			} else {
				PX4_WARN("Cannot compute target actuator position for bucket angle");
				return false;
			}

		} else {
			// Direct mode: use commanded actuator position
			target_actuator_position = _commanded_actuator_position;
			return true;
		}

	} else if (state_info.state == BucketStateManager::OperationalState::READY) {
		// Hold current position
		target_actuator_position = sensor_data.hbridge_position;
		return true;
	}

	return false;  // No valid target position
}

void BucketControl::send_zero_command()
{
	// Send zero command if no valid target
	BucketHardwareInterface::HbridgeCommand hbridge_cmd{};
	hbridge_cmd.duty_cycle = 0.0f;
	hbridge_cmd.enable = false;
	_hardware_interface->send_hbridge_command(hbridge_cmd);
}

void BucketControl::execute_motion_control(
	float target_actuator_position,
	const BucketHardwareInterface::SensorData& sensor_data)
{
	// Plan smooth trajectory
	const float dt = static_cast<float>(CONTROL_INTERVAL_US) * MICROSECONDS_TO_SECONDS;
	auto motion_setpoint = _motion_controller->plan_trajectory(
		sensor_data.hbridge_position,
		target_actuator_position,
		dt
	);

	// Compute control output
	auto control_output = _motion_controller->compute_control(
		motion_setpoint,
		sensor_data.hbridge_position,
		sensor_data.hbridge_velocity,
		dt
	);

	// Send command to hardware
	BucketHardwareInterface::HbridgeCommand hbridge_cmd{};
	hbridge_cmd.duty_cycle = control_output.duty_cycle;
	hbridge_cmd.enable = !control_output.safety_stop;

	_hardware_interface->send_hbridge_command(hbridge_cmd);
}

// Static methods for module management
int BucketControl::task_spawn(int argc, char *argv[])
{
	BucketControl *instance = new BucketControl();

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

int BucketControl::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	if (argc > 0) {
		if (strcmp(argv[0], "calibrate") == 0) {
			// Trigger calibration
			get_instance()->_state_manager->start_calibration();
			return PX4_OK;
		}

		if (strcmp(argv[0], "emergency_stop") == 0) {
			// Trigger emergency stop
			get_instance()->_state_manager->trigger_emergency_stop("Manual emergency stop");
			return PX4_OK;
		}

		if (strcmp(argv[0], "clear_emergency") == 0) {
			// Clear emergency stop
			if (get_instance()->_state_manager->clear_emergency_stop()) {
				return PX4_OK;
			} else {
				return PX4_ERROR;
			}
		}

		if (strcmp(argv[0], "status") == 0) {
			// Print detailed status
			auto state_info = get_instance()->_state_manager->get_state_info();
			PX4_INFO("State: %d, Errors: 0x%lx, Message: %s",
				 static_cast<int>(state_info.state),
				 (unsigned long)state_info.error_flags,
				 state_info.status_message);
			return PX4_OK;
		}
	}

	return print_usage("unknown command");
}

int BucketControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Bucket control module for wheel loader applications.

This module manages the bucket actuator control including:
- Kinematic calculations for bucket angle and linkage geometry
- Motion planning with smooth trajectory generation
- Integrated boom angle compensation within kinematics
- Hardware interface for motors, encoders, and limit switches
- State management with calibration and fault handling
- Safety monitoring and emergency stop functionality

### Implementation
The module uses a component-based architecture with separate classes for:
- BucketKinematics: Linkage geometry and angle calculations
- BucketHardwareInterface: Hardware abstraction layer
- BucketMotionController: Trajectory planning and control
- BucketStateManager: State machine and fault handling

### Examples
Start the module:
$ bucket_control start

Stop the module:
$ bucket_control stop

Trigger calibration:
$ bucket_control calibrate

Emergency stop:
$ bucket_control emergency_stop

Clear emergency stop:
$ bucket_control clear_emergency

Check status:
$ bucket_control status
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("bucket_control", "actuator");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_COMMAND("calibrate");
	PRINT_MODULE_USAGE_COMMAND("emergency_stop");
	PRINT_MODULE_USAGE_COMMAND("clear_emergency");
	PRINT_MODULE_USAGE_COMMAND("status");

	return 0;
}
