/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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

#include "operation_mode_manager.hpp"
#include <px4_platform_common/getopt.h>

using namespace wheel_loader;

OperationModeManager::OperationModeManager() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
}

OperationModeManager::~OperationModeManager()
{
	perf_free(loop_perf);
	perf_free(mode_switch_perf);
}

bool OperationModeManager::init()
{
	// Initialize performance counters
	loop_perf = perf_alloc(PC_ELAPSED, MODULE_NAME": cycle");
	mode_switch_perf = perf_alloc(PC_ELAPSED, MODULE_NAME": mode_switch");

	// Create operation modes
	manual_mode = std::make_unique<ManualMode>();
	vla_mode = std::make_unique<VlaMode>();

	// Initialize all modes
	if (!manual_mode->init() || !vla_mode->init()) {
		PX4_ERR("Failed to initialize operation modes");
		return false;
	}

	// Set default mode
	OperationMode default_mode = static_cast<OperationMode>(param_default_mode.get());
	switch_mode(default_mode);

	// Schedule at configured rate (default 100Hz)
	float update_rate = param_update_rate.get();
	uint32_t interval_us = static_cast<uint32_t>(1000000.0f / update_rate);
	ScheduleOnInterval(interval_us);

	PX4_INFO("Operation Mode Manager initialized");
	return true;
}

void OperationModeManager::Run()
{
	perf_begin(loop_perf);

	// Update all subscriptions
	update_subscriptions();

	// Update vehicle status
	update_vehicle_status();

	// Handle parameter updates
	if (parameter_update_sub.updated()) {
		parameter_update_s pupdate;
		parameter_update_sub.copy(&pupdate);
		updateParams();
	}

	// Check for emergency conditions first
	handle_emergency();

	if (!emergency_stop) {
		// Check for mode switching requests
		check_mode_switching();

		// Update current active mode
		update_current_mode();

		// Publish trajectory setpoints
		publish_trajectory_setpoints();
	}

	perf_end(loop_perf);
}

void OperationModeManager::update_subscriptions()
{
	// Update manual control
	if (manual_control_setpoint_sub.updated()) {
		manual_control_setpoint_sub.copy(&manual_control);
		manual_control_valid = (hrt_absolute_time() - manual_control.timestamp) < 200000; // 200ms timeout
	}

	// Update vehicle status
	if (vehicle_status_sub.updated()) {
		vehicle_status_sub.copy(&vehicle_status);
		system_armed = (vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
	}

	// Update vehicle position
	if (vehicle_local_position_sub.updated()) {
		vehicle_local_position_sub.copy(&vehicle_position);
		position_valid = vehicle_position.xy_valid && vehicle_position.z_valid;
	}

	// Update VLA command
	if (vla_command_sub.updated()) {
		vla_command_sub.copy(&vla_command);
		vla_command_valid = (hrt_absolute_time() - vla_command.timestamp) < 1000000; // 1s timeout
	}
}

void OperationModeManager::update_vehicle_status()
{
	// Check position accuracy
	if (position_valid) {
		float pos_accuracy = sqrtf(vehicle_position.eph * vehicle_position.eph +
		                          vehicle_position.epv * vehicle_position.epv);
		position_valid = pos_accuracy < param_min_position_accuracy.get();
	}
}

void OperationModeManager::check_mode_switching()
{
	// Determine requested mode from RC input
	OperationMode new_requested_mode = current_operation_mode;

	if (manual_control_valid) {
		// Use configured channel for mode switching
		int switch_channel = param_mode_switch_channel.get();
		float switch_value = 0.0f;

		// Map RC channel to switch value (simplified)
		switch (switch_channel) {
		case 5: switch_value = manual_control.aux1; break;
		case 6: switch_value = manual_control.aux2; break;
		case 7: switch_value = manual_control.aux3; break;
		case 8: switch_value = manual_control.aux4; break;
		case 9: switch_value = manual_control.aux5; break;
		default: switch_value = 0.0f; break;
		}

		// Determine mode based on switch position
		float threshold = param_mode_switch_threshold.get();
		if (switch_value < -threshold) {
			new_requested_mode = OperationMode::MANUAL;
		} else if (switch_value > threshold) {
			new_requested_mode = OperationMode::VLA;
		}
	}

	// Check for VLA command override
	if (vla_command_valid && vla_command.mode_request != 0) {
		new_requested_mode = OperationMode::VLA;
	}

	// Update requested mode
	if (new_requested_mode != requested_mode) {
		requested_mode = new_requested_mode;
		mode_switch_pending = true;
		mode_switch_time = hrt_absolute_time();
	}

	// Execute mode switch if conditions are met
	if (mode_switch_pending) {
		hrt_abstime now = hrt_absolute_time();

		// Check delay and minimum interval
		bool delay_met = (now - mode_switch_time) > MODE_SWITCH_DELAY;
		bool interval_met = (now - last_mode_switch) > MIN_MODE_SWITCH_INTERVAL;

		if (delay_met && interval_met && check_safety_conditions()) {
			perf_begin(mode_switch_perf);

			if (switch_mode(requested_mode)) {
				mode_switch_pending = false;
				last_mode_switch = now;
				PX4_INFO("Switched to %s mode", get_mode_name(current_operation_mode));
			}

			perf_end(mode_switch_perf);
		}
	}
}

bool OperationModeManager::switch_mode(OperationMode new_mode)
{
	// Deactivate current mode
	if (current_mode != nullptr) {
		current_mode->deactivate();
	}

	// Switch to new mode
	switch (new_mode) {
	case OperationMode::MANUAL:
		current_mode = manual_mode.get();
		break;
	case OperationMode::VLA:
		current_mode = vla_mode.get();
		break;
	case OperationMode::EMERGENCY:
		current_mode = nullptr;
		break;
	default:
		PX4_ERR("Unknown operation mode: %d", static_cast<int>(new_mode));
		return false;
	}

	// Activate new mode
	if (current_mode != nullptr) {
		current_mode->activate();
	}

	current_operation_mode = new_mode;
	return true;
}

void OperationModeManager::update_current_mode()
{
	if (current_mode != nullptr) {
		current_mode->update();
	}
}

void OperationModeManager::publish_trajectory_setpoints()
{
	if (current_mode != nullptr) {
		// Get setpoints from current mode
		ChassisTrajectorySetpoint chassis_setpoint = current_mode->get_chassis_setpoint();
		BucketTrajectorySetpoint bucket_setpoint = current_mode->get_bucket_setpoint();

		// Publish chassis setpoint
		if (chassis_setpoint.valid) {
			chassis_setpoint_pub.publish(chassis_setpoint);
		}

		// Publish bucket setpoint
		if (bucket_setpoint.valid) {
			bucket_setpoint_pub.publish(bucket_setpoint);
		}
	}

	// Publish mode status
	operation_mode_status_s status{};
	status.current_mode = static_cast<uint8_t>(current_operation_mode);
	status.requested_mode = static_cast<uint8_t>(requested_mode);
	status.mode_switch_pending = mode_switch_pending;
	status.emergency_stop = emergency_stop;
	status.system_armed = system_armed;
	status.position_valid = position_valid;
	status.manual_control_valid = manual_control_valid;
	status.vla_command_valid = vla_command_valid;
	status.timestamp = hrt_absolute_time();

	mode_status_pub.publish(status);
}

void OperationModeManager::handle_emergency()
{
	bool emergency_detected = false;

	// Check safety conditions if enabled
	if (param_safety_enable.get() > 0) {
		// Check for loss of position
		if (!position_valid) {
			emergency_detected = true;
			PX4_WARN("Emergency: Position invalid");
		}

		// Check for RC loss in manual mode
		if (current_operation_mode == OperationMode::MANUAL && !manual_control_valid) {
			emergency_detected = true;
			PX4_WARN("Emergency: Manual control lost");
		}

		// Check for VLA command timeout in VLA mode
		if (current_operation_mode == OperationMode::VLA && !vla_command_valid) {
			hrt_abstime timeout = static_cast<hrt_abstime>(param_emergency_timeout.get() * 1e6f);
			if ((hrt_absolute_time() - vla_command.timestamp) > timeout) {
				emergency_detected = true;
				PX4_WARN("Emergency: VLA command timeout");
			}
		}

		// Check system disarmed
		if (!system_armed) {
			emergency_detected = true;
		}
	}

	// Handle emergency state change
	if (emergency_detected && !emergency_stop) {
		emergency_stop = true;
		switch_mode(OperationMode::EMERGENCY);
		reset_all_modes();
		PX4_ERR("EMERGENCY STOP ACTIVATED");
	} else if (!emergency_detected && emergency_stop) {
		emergency_stop = false;
		// Return to default mode
		switch_mode(static_cast<OperationMode>(param_default_mode.get()));
		PX4_INFO("Emergency stop cleared, returning to normal operation");
	}
}

void OperationModeManager::reset_all_modes()
{
	if (manual_mode) {
		manual_mode->deactivate();
	}
	if (vla_mode) {
		vla_mode->deactivate();
	}
	current_mode = nullptr;
}

bool OperationModeManager::check_safety_conditions()
{
	// Basic safety checks for mode switching
	if (param_safety_enable.get() == 0) {
		return true; // Safety checks disabled
	}

	// Must have valid position
	if (!position_valid) {
		return false;
	}

	// Must be armed
	if (!system_armed) {
		return false;
	}

	// Mode-specific checks
	switch (requested_mode) {
	case OperationMode::MANUAL:
		return manual_control_valid;
	case OperationMode::VLA:
		return vla_command_valid;
	case OperationMode::EMERGENCY:
		return true; // Always allow emergency mode
	default:
		return false;
	}
}

const char* OperationModeManager::get_mode_name(OperationMode mode)
{
	switch (mode) {
	case OperationMode::MANUAL: return "MANUAL";
	case OperationMode::VLA: return "VLA";
	case OperationMode::EMERGENCY: return "EMERGENCY";
	default: return "UNKNOWN";
	}
}

int OperationModeManager::print_status()
{
	PX4_INFO("Operation Mode Manager Status:");
	PX4_INFO("  Current Mode: %s", get_mode_name(current_operation_mode));
	PX4_INFO("  Requested Mode: %s", get_mode_name(requested_mode));
	PX4_INFO("  Mode Switch Pending: %s", mode_switch_pending ? "YES" : "NO");
	PX4_INFO("  Emergency Stop: %s", emergency_stop ? "YES" : "NO");
	PX4_INFO("  System Armed: %s", system_armed ? "YES" : "NO");
	PX4_INFO("  Position Valid: %s", position_valid ? "YES" : "NO");
	PX4_INFO("  Manual Control Valid: %s", manual_control_valid ? "YES" : "NO");
	PX4_INFO("  VLA Command Valid: %s", vla_command_valid ? "YES" : "NO");

	// Print mode-specific status
	if (current_mode != nullptr) {
		current_mode->print_status();
	}

	return 0;
}

int OperationModeManager::task_spawn(int argc, char *argv[])
{
	OperationModeManager *instance = new OperationModeManager();

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

OperationModeManager *OperationModeManager::instantiate(int argc, char *argv[])
{
	return new OperationModeManager();
}

int OperationModeManager::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int OperationModeManager::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Operation Mode Manager for wheel loader robot. Manages switching between manual and VLA modes,
and coordinates trajectory generation with safety oversight.

### Implementation
The module implements two main operation modes:
- Manual Mode: RC input processing with trajectory generation
- VLA Mode: Vision-Language-Action autonomous operation with trajectory decomposition

The manager publishes trajectory setpoints that are consumed by separate trajectory followers
for chassis and bucket control.

### Examples
Start the operation mode manager:
$ operation_mode_manager start

Stop the operation mode manager:
$ operation_mode_manager stop

Print status:
$ operation_mode_manager status
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("operation_mode_manager", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_COMMAND("status");

	return 0;
}

extern "C" __EXPORT int operation_mode_manager_main(int argc, char *argv[])
{
	return OperationModeManager::main(argc, argv);
}
