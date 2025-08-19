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

#include "wheel_loader_controller.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <lib/mathlib/mathlib.h>
#include <cstring>

WheelLoaderController::WheelLoaderController() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
}

WheelLoaderController::~WheelLoaderController()
{
	perf_free(_cycle_perf);
	perf_free(_emergency_stop_perf);
}

bool WheelLoaderController::init()
{
	// Initialize performance counters
	_cycle_perf = perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle");
	_emergency_stop_perf = perf_alloc(PC_COUNT, MODULE_NAME ": emergency_stops");

	// Set initial state
	_control_state = ControlState::INITIALIZING;
	_state_entered_time = hrt_absolute_time();

	// Update parameters
	updateParams();

	// Initialize subsystem health states
	_boom_health = HealthState::UNKNOWN;
	_bucket_health = HealthState::UNKNOWN;
	_steering_health = HealthState::UNKNOWN;
	_front_wheel_health = HealthState::UNKNOWN;
	_rear_wheel_health = HealthState::UNKNOWN;

	ScheduleOnInterval(CONTROL_INTERVAL_US);

	return true;
}

void WheelLoaderController::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_cycle_perf);

	// Check for parameter updates
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}

	// Process incoming commands and update state
	processWheelLoaderCommand();
	processTaskExecution();
	processVehicleCommand();
	processVlaCommand();
	processSlipEstimation();

	// Process mode switching and management
	processModeSwitch();

	// Update subsystem health monitoring
	updateSubsystemHealth();

	// Perform safety checks
	performSafetyChecks();

	// Update control state
	updateControlState();

	// Generate and publish commands to subsystems
	publishCommands();

	// Publish status
	publishStatus();

	perf_end(_cycle_perf);
}

void WheelLoaderController::processWheelLoaderCommand()
{
	if (_wheel_loader_command_sub.updated()) {
		wheel_loader_command_s cmd;

		if (_wheel_loader_command_sub.copy(&cmd)) {
			// Validate command
			if (validateCommand(cmd)) {
				// Store command based on source
				switch (cmd.command_source) {
				case wheel_loader_command_s::SOURCE_MANUAL_CONTROL:
					_manual_command = cmd;
					break;

				case wheel_loader_command_s::SOURCE_TASK_EXECUTION:
					_task_command = cmd;
					break;

				case wheel_loader_command_s::SOURCE_EXTERNAL:
					_external_command = cmd;
					break;

				default:
					PX4_WARN("Unknown command source: %d", cmd.command_source);
					return;
				}

				_last_command_time = hrt_absolute_time();

				if (_diagnostic_enable.get()) {
					PX4_INFO("Received command from source %d", cmd.command_source);
				}
			}
		}
	}
}

void WheelLoaderController::processTaskExecution()
{
	if (_task_execution_command_sub.updated()) {
		task_execution_command_s task_cmd;

		if (_task_execution_command_sub.copy(&task_cmd)) {
			// Convert task execution command to wheel loader command
			wheel_loader_command_s cmd{};
			cmd.timestamp = task_cmd.timestamp;
			cmd.command_source = wheel_loader_command_s::SOURCE_TASK_EXECUTION;

			// Map task command fields to wheel loader command
			// This would be customized based on task execution interface
			cmd.drive_mode = wheel_loader_command_s::DRIVE_MODE_VELOCITY;
			cmd.hydraulic_mode = wheel_loader_command_s::HYDRAULIC_MODE_AUTO;
			cmd.emergency_stop = task_cmd.emergency_stop;

			_task_command = cmd;
			_last_command_time = hrt_absolute_time();
		}
	}
}

void WheelLoaderController::processVehicleCommand()
{
	if (_manual_control_setpoint_sub.updated()) {
		manual_control_setpoint_s manual;

		if (_manual_control_setpoint_sub.copy(&manual)) {
			// Convert manual control to wheel loader command
			wheel_loader_command_s cmd{};
			cmd.timestamp = manual.timestamp;
			cmd.command_source = wheel_loader_command_s::SOURCE_MANUAL_CONTROL;

			// Map manual control inputs to wheel loader commands
			// Scale manual inputs (-1 to 1) to appropriate ranges
			cmd.front_left_wheel_speed = manual.pitch * _max_speed.get();
			cmd.front_right_wheel_speed = manual.pitch * _max_speed.get();
			cmd.rear_left_wheel_speed = manual.pitch * _max_speed.get();
			cmd.rear_right_wheel_speed = manual.pitch * _max_speed.get();

			cmd.steering_angle_cmd = manual.roll * 0.5f; // Max ±0.5 rad
			cmd.boom_lift_cmd = manual.throttle * 0.1f; // Manual boom control
			cmd.bucket_angle_cmd = manual.yaw * 0.2f; // Manual bucket control

			cmd.drive_mode = wheel_loader_command_s::DRIVE_MODE_MANUAL;
			cmd.hydraulic_mode = wheel_loader_command_s::HYDRAULIC_MODE_MANUAL;
			cmd.emergency_stop = (manual.buttons & (1 << 0)) != 0; // Use button 0 for emergency stop

			_manual_command = cmd;
			_last_command_time = hrt_absolute_time();
		}
	}
}

void WheelLoaderController::processSlipEstimation()
{
	if (_slip_estimation_sub.updated()) {
		slip_estimation_s slip_data;
		if (_slip_estimation_sub.copy(&slip_data)) {
			_current_slip_data = slip_data;
			_last_slip_estimation_time = hrt_absolute_time();

			// Update slip detection flags
			_slip_detected = slip_data.slip_detected;
			_critical_slip = slip_data.critical_slip;

			// Calculate traction reduction factor based on slip
			if (_critical_slip) {
				// Reduce power significantly for critical slip
				_traction_reduction_factor = 0.3f;
				if (_diagnostic_enable.get()) {
					PX4_WARN("Critical slip detected - reducing traction to 30%%");
				}
			} else if (_slip_detected) {
				// Moderate reduction for detected slip
				_traction_reduction_factor = 0.7f;
				if (_diagnostic_enable.get()) {
					PX4_INFO("Slip detected - reducing traction to 70%%");
				}
			} else {
				// Gradually restore full traction when no slip
				_traction_reduction_factor = math::min(_traction_reduction_factor + 0.05f, 1.0f);
			}

			// Apply additional safety measures for wheel speed variance
			if (slip_data.wheel_speed_variance > 0.5f) {
				_traction_reduction_factor *= 0.8f; // Further reduce for high variance
			}
		}
	}
}

WheelLoaderController::CommandSource WheelLoaderController::selectActiveCommandSource()
{
	hrt_abstime now = hrt_absolute_time();
	float timeout_us = _cmd_timeout.get() * 1_s;

	// Emergency stop has highest priority
	if (_emergency_stop_active) {
		return CommandSource::NONE;
	}

	// Manual control always has priority over autonomous (safety requirement)
	if ((now - _manual_command.timestamp) < timeout_us) {
		return CommandSource::MANUAL;
	}

	// Select command source based on operation mode
	switch (_operation_mode) {
	case OperationMode::AUTO_MODE:
		// In auto mode, prioritize VLA commands
		if (_vla_valid && (now - _vla_command.timestamp) < timeout_us) {
			return CommandSource::SMOL_VLA;
		}
		// Fall back to task execution if VLA not available
		if ((now - _task_command.timestamp) < timeout_us) {
			return CommandSource::TASK_EXECUTION;
		}
		break;

	case OperationMode::MANUAL_MODE:
		// In manual mode, only accept external commands as backup
		if ((now - _external_command.timestamp) < timeout_us) {
			return CommandSource::EXTERNAL;
		}
		break;

	case OperationMode::TRANSITION_MODE:
		// During transition, no autonomous commands accepted
		break;
	}

	return CommandSource::NONE;
}

bool WheelLoaderController::validateCommand(const wheel_loader_command_s &cmd)
{
	// Check timestamp validity
	hrt_abstime now = hrt_absolute_time();

	if (cmd.timestamp == 0 || (now - cmd.timestamp) > (1_s)) {
		PX4_WARN("Command timestamp invalid or too old");
		return false;
	}

	// Validate speed commands
	float max_speed = _max_speed.get();

	if (fabsf(cmd.front_left_wheel_speed) > max_speed ||
		fabsf(cmd.front_right_wheel_speed) > max_speed ||
		fabsf(cmd.rear_left_wheel_speed) > max_speed ||
		fabsf(cmd.rear_right_wheel_speed) > max_speed) {
		PX4_WARN("Wheel speed command exceeds limits");
		return false;
	}

	// Validate electric motor commands
	if (fabsf(cmd.boom_lift_cmd) > M_PI_2_F ||
		fabsf(cmd.boom_extend_cmd) > 1.0f ||
		fabsf(cmd.bucket_angle_cmd) > M_PI_F) {
		PX4_WARN("Electric motor command exceeds limits");
		return false;
	}

	// Validate steering commands
	if (fabsf(cmd.steering_angle_cmd) > M_PI_4_F ||
		fabsf(cmd.articulation_angle_cmd) > M_PI_4_F) {
		PX4_WARN("Steering command exceeds limits");
		return false;
	}

	return true;
}

void WheelLoaderController::applyCommandLimits(wheel_loader_command_s &cmd)
{
	float max_speed = _max_speed.get();
	float max_accel = _max_accel.get();

	// Limit wheel speeds
	cmd.front_left_wheel_speed = math::constrain(cmd.front_left_wheel_speed, -max_speed, max_speed);
	cmd.front_right_wheel_speed = math::constrain(cmd.front_right_wheel_speed, -max_speed, max_speed);
	cmd.rear_left_wheel_speed = math::constrain(cmd.rear_left_wheel_speed, -max_speed, max_speed);
	cmd.rear_right_wheel_speed = math::constrain(cmd.rear_right_wheel_speed, -max_speed, max_speed);

	// Limit electric motor commands
	cmd.boom_lift_cmd = math::constrain(cmd.boom_lift_cmd, -M_PI_2_F, M_PI_2_F);
	cmd.boom_extend_cmd = math::constrain(cmd.boom_extend_cmd, 0.0f, 1.0f);
	cmd.bucket_angle_cmd = math::constrain(cmd.bucket_angle_cmd, -M_PI_F, M_PI_F);

	// Limit steering commands
	cmd.steering_angle_cmd = math::constrain(cmd.steering_angle_cmd, -M_PI_4_F, M_PI_4_F);
	cmd.articulation_angle_cmd = math::constrain(cmd.articulation_angle_cmd, -M_PI_4_F, M_PI_4_F);

	// Apply acceleration limits (simplified rate limiting)
	static hrt_abstime last_limit_time = 0;
	hrt_abstime now = hrt_absolute_time();
	float dt = (now - last_limit_time) / 1e6f;

	if (dt > 0.001f && dt < 0.1f) { // Reasonable delta time
		// Rate limit implementation would go here
		// For now, we just enforce the max acceleration parameter exists
		(void)max_accel; // Suppress unused variable warning
	}

	last_limit_time = now;
}

void WheelLoaderController::generateSubsystemCommands(const wheel_loader_command_s &cmd)
{
	hrt_abstime now = hrt_absolute_time();

	// Apply traction control factor to wheel speeds if slip is detected
	float front_left_speed = cmd.front_left_wheel_speed * _traction_reduction_factor;
	float front_right_speed = cmd.front_right_wheel_speed * _traction_reduction_factor;
	float rear_left_speed = cmd.rear_left_wheel_speed * _traction_reduction_factor;
	float rear_right_speed = cmd.rear_right_wheel_speed * _traction_reduction_factor;

	// Generate front wheel controller command
	wheel_speeds_setpoint_s front_wheel_cmd{};
	front_wheel_cmd.timestamp = now;
	const float wheel_radius_m = 0.25f; // Should be parameterized
	front_wheel_cmd.front_wheel_speed_rad_s = (front_left_speed + front_right_speed) * 0.5f / wheel_radius_m;
	front_wheel_cmd.max_acceleration = _max_accel.get();

	// Generate rear wheel controller command
	wheel_speeds_setpoint_s rear_wheel_cmd{};
	rear_wheel_cmd.timestamp = now;
	rear_wheel_cmd.rear_wheel_speed_rad_s = (rear_left_speed + rear_right_speed) * 0.5f / wheel_radius_m;
	rear_wheel_cmd.max_acceleration = _max_accel.get();
	rear_wheel_cmd.synchronized = true;
	rear_wheel_cmd.differential_enable = false;

	// Generate boom command
	boom_command_s boom_cmd{};
	boom_cmd.timestamp = now;
	boom_cmd.lift_angle_cmd = cmd.boom_lift_cmd;
	boom_cmd.extend_position_cmd = cmd.boom_extend_cmd;
	boom_cmd.control_mode = (cmd.hydraulic_mode == wheel_loader_command_s::HYDRAULIC_MODE_MANUAL) ?
							boom_command_s::MODE_VELOCITY : boom_command_s::MODE_POSITION;
	boom_cmd.emergency_stop = cmd.emergency_stop || _emergency_stop_active;
	boom_cmd.command_priority = boom_command_s::PRIORITY_NORMAL;

	// Generate bucket command
	bucket_command_s bucket_cmd{};
	bucket_cmd.timestamp = now;
	bucket_cmd.target_angle = cmd.bucket_angle_cmd;
	bucket_cmd.command_mode = (cmd.hydraulic_mode == wheel_loader_command_s::HYDRAULIC_MODE_MANUAL) ?
							  bucket_command_s::MODE_MANUAL : bucket_command_s::MODE_AUTO_LEVEL;
	bucket_cmd.control_mode = bucket_cmd.command_mode;

	// Generate steering command
	steering_command_s steering_cmd{};
	steering_cmd.timestamp = now;
	steering_cmd.front_steering_angle_cmd = cmd.steering_angle_cmd;
	steering_cmd.articulation_angle_cmd = cmd.articulation_angle_cmd;
	steering_cmd.steering_mode = steering_command_s::MODE_FRONT_ONLY;
	steering_cmd.control_type = steering_command_s::CONTROL_POSITION;
	steering_cmd.command_priority = steering_command_s::PRIORITY_NORMAL;

	// Apply emergency stop overrides
	if (cmd.emergency_stop || _emergency_stop_active) {
		front_wheel_cmd.front_wheel_speed_rad_s = 0.0f;
		rear_wheel_cmd.rear_wheel_speed_rad_s = 0.0f;
	}

	// Publish commands
	_front_wheel_setpoint_pub.publish(front_wheel_cmd);
	_rear_wheel_setpoint_pub.publish(rear_wheel_cmd);
	_boom_command_pub.publish(boom_cmd);
	_bucket_command_pub.publish(bucket_cmd);
	_steering_command_pub.publish(steering_cmd);
}

void WheelLoaderController::updateControlState()
{
	ControlState new_state = _control_state;
	CommandSource active_source = selectActiveCommandSource();

	switch (_control_state) {
	case ControlState::INITIALIZING:
		if (isSystemHealthy()) {
			new_state = ControlState::IDLE;
		}
		break;

	case ControlState::IDLE:
		if (_emergency_stop_active) {
			new_state = ControlState::EMERGENCY_STOP;
		} else if (active_source == CommandSource::MANUAL) {
			new_state = ControlState::MANUAL_OPERATION;
		} else if (active_source == CommandSource::SMOL_VLA) {
			new_state = ControlState::AUTO_OPERATION;
		} else if (active_source == CommandSource::TASK_EXECUTION) {
			new_state = ControlState::TASK_EXECUTION;
		}
		break;

	case ControlState::MANUAL_OPERATION:
		if (_emergency_stop_active) {
			new_state = ControlState::EMERGENCY_STOP;
		} else if (active_source != CommandSource::MANUAL) {
			new_state = ControlState::IDLE;
		}
		break;

	case ControlState::AUTO_OPERATION:
		if (_emergency_stop_active) {
			new_state = ControlState::EMERGENCY_STOP;
		} else if (active_source == CommandSource::MANUAL) {
			// Manual override always takes precedence
			new_state = ControlState::MANUAL_OPERATION;
		} else if (active_source != CommandSource::SMOL_VLA) {
			new_state = ControlState::IDLE;
		} else {
			// Process auto load/dump sequences
			if (_auto_load_enable.get()) {
				processAutoLoadSequence();
			}
			if (_auto_dump_enable.get()) {
				processAutoDumpSequence();
			}
		}
		break;

	case ControlState::TASK_EXECUTION:
		if (_emergency_stop_active) {
			new_state = ControlState::EMERGENCY_STOP;
		} else if (active_source == CommandSource::MANUAL) {
			new_state = ControlState::MANUAL_OPERATION;
		} else if (active_source == CommandSource::NONE) {
			new_state = ControlState::IDLE;
		}
		break;

	case ControlState::MODE_TRANSITION:
		// Mode transition is handled by processModeSwitch()
		// This state will transition automatically after timeout
		if (_emergency_stop_active) {
			new_state = ControlState::EMERGENCY_STOP;
		}
		break;

	case ControlState::EMERGENCY_STOP:
		if (!_emergency_stop_active && isSystemHealthy()) {
			new_state = ControlState::IDLE;
		}
		break;

	case ControlState::ERROR:
		if (isSystemHealthy()) {
			new_state = ControlState::IDLE;
		}
		break;
	}

	// Check for critical system faults
	if (!isSystemHealthy() && new_state != ControlState::EMERGENCY_STOP) {
		new_state = ControlState::ERROR;
	}

	if (new_state != _control_state) {
		transitionToState(new_state);
	}

	_active_command_source = active_source;
}

void WheelLoaderController::transitionToState(ControlState new_state)
{
	if (!isValidStateTransition(_control_state, new_state)) {
		PX4_WARN("Invalid state transition from %d to %d", (int)_control_state, (int)new_state);
		return;
	}

	PX4_INFO("State transition: %d -> %d", (int)_control_state, (int)new_state);

	_previous_state = _control_state;
	_control_state = new_state;
	_state_entered_time = hrt_absolute_time();

	// Perform state entry actions
	switch (new_state) {
	case ControlState::EMERGENCY_STOP:
		handleEmergencyStop();
		break;

	case ControlState::IDLE:
		resetControlState();
		break;

	default:
		break;
	}
}

bool WheelLoaderController::isValidStateTransition(ControlState from, ControlState to)
{
	// Emergency stop and error states can be entered from any state
	if (to == ControlState::EMERGENCY_STOP || to == ControlState::ERROR) {
		return true;
	}

	// All states can transition to idle
	if (to == ControlState::IDLE) {
		return true;
	}

	// Mode transition can be entered from operational states
	if (to == ControlState::MODE_TRANSITION) {
		return from == ControlState::MANUAL_OPERATION || from == ControlState::AUTO_OPERATION || 
		       from == ControlState::IDLE;
	}

	// Other transitions depend on current state
	switch (from) {
	case ControlState::INITIALIZING:
		return to == ControlState::IDLE;

	case ControlState::IDLE:
		return to == ControlState::MANUAL_OPERATION || to == ControlState::AUTO_OPERATION || 
		       to == ControlState::TASK_EXECUTION || to == ControlState::MODE_TRANSITION;

	case ControlState::MANUAL_OPERATION:
		return to == ControlState::AUTO_OPERATION || to == ControlState::TASK_EXECUTION ||
		       to == ControlState::MODE_TRANSITION;

	case ControlState::AUTO_OPERATION:
		return to == ControlState::MANUAL_OPERATION || to == ControlState::MODE_TRANSITION;

	case ControlState::TASK_EXECUTION:
		return to == ControlState::MANUAL_OPERATION || to == ControlState::AUTO_OPERATION;

	case ControlState::MODE_TRANSITION:
		return to == ControlState::MANUAL_OPERATION || to == ControlState::AUTO_OPERATION;

	default:
		return false;
	}
}

void WheelLoaderController::resetControlState()
{
	// Clear active commands
	memset(&_current_command, 0, sizeof(_current_command));
	memset(&_manual_command, 0, sizeof(_manual_command));
	memset(&_task_command, 0, sizeof(_task_command));
	memset(&_external_command, 0, sizeof(_external_command));

	_active_command_source = CommandSource::NONE;
	_safety_override_active = false;
}

void WheelLoaderController::publishCommands()
{
	// Select active command based on current state and source priority
	wheel_loader_command_s active_cmd{};
	bool has_valid_command = false;

	switch (_active_command_source) {
	case CommandSource::MANUAL:
		if (_control_state == ControlState::MANUAL_OPERATION) {
			active_cmd = _manual_command;
			has_valid_command = true;
		}
		break;

	case CommandSource::SMOL_VLA:
		if (_control_state == ControlState::AUTO_OPERATION) {
			active_cmd = _vla_command;
			has_valid_command = true;
		}
		break;

	case CommandSource::TASK_EXECUTION:
		if (_control_state == ControlState::TASK_EXECUTION) {
			active_cmd = _task_command;
			has_valid_command = true;
		}
		break;

	case CommandSource::EXTERNAL:
		active_cmd = _external_command;
		has_valid_command = true;
		break;

	default:
		break;
	}

	if (has_valid_command && _control_state != ControlState::EMERGENCY_STOP && 
	    _control_state != ControlState::MODE_TRANSITION) {
		// Apply safety limits
		applyCommandLimits(active_cmd);

		// Store as current command
		_current_command = active_cmd;

		// Generate and publish subsystem commands
		generateSubsystemCommands(active_cmd);

	} else {
		// Publish safe/idle commands
		wheel_loader_command_s safe_cmd{};
		safe_cmd.timestamp = hrt_absolute_time();
		safe_cmd.emergency_stop = _emergency_stop_active;

		generateSubsystemCommands(safe_cmd);
	}
}

void WheelLoaderController::publishStatus()
{
	wheel_loader_status_s status{};
	status.timestamp = hrt_absolute_time();

	// Current wheel speeds (from wheel status feedback) - EKF2-style SubscriptionMultiArray
	wheel_status_s front_wheel_status, rear_wheel_status;
	uint8_t front_wheel_idx = static_cast<uint8_t>(_front_wheel_idx.get());
	uint8_t rear_wheel_idx = static_cast<uint8_t>(_rear_wheel_idx.get());

	if (front_wheel_idx < _wheel_status_subs.size() &&
	    _wheel_status_subs[front_wheel_idx].updated() &&
	    _wheel_status_subs[front_wheel_idx].copy(&front_wheel_status)) {
		status.front_wheel_speed = front_wheel_status.current_speed_rpm * (2.0f * M_PI_F / 60.0f); // Convert to rad/s
		status.front_motor_current = front_wheel_status.motor_current_amps;
	}

	if (rear_wheel_idx < _wheel_status_subs.size() &&
	    _wheel_status_subs[rear_wheel_idx].updated() &&
	    _wheel_status_subs[rear_wheel_idx].copy(&rear_wheel_status)) {
		status.rear_wheel_speed = rear_wheel_status.current_speed_rpm * (2.0f * M_PI_F / 60.0f); // Convert to rad/s
		status.rear_motor_current = rear_wheel_status.motor_current_amps;
	}

	// System health
	status.system_health = static_cast<uint8_t>(evaluateOverallHealth());
	status.motor_fault = (front_wheel_status.controller_healthy == false) || (rear_wheel_status.controller_healthy == false);
	status.communication_fault = (_front_wheel_health == HealthState::ERROR) || (_rear_wheel_health == HealthState::ERROR);

	// Temperature monitoring (use front wheel as representative)
	status.motor_temperature = front_wheel_status.motor_temperature_c;
	status.controller_temperature = front_wheel_status.motor_temperature_c; // Placeholder

	// Power status (placeholder values - would come from power monitoring)
	status.supply_voltage = 24.0f; // Placeholder
	status.power_consumption = 1000.0f; // Placeholder

	// Operational counters (placeholder values - would be persistent)
	status.operating_hours = 0; // Placeholder
	status.cycle_count = 0; // Placeholder

	_wheel_loader_status_pub.publish(status);
}

void WheelLoaderController::performSafetyChecks()
{
	// Check for emergency stop conditions
	bool emergency_triggered = false;

	// Check if emergency stop is enabled and commanded
	if (_estop_enable.get() && _current_command.emergency_stop) {
		emergency_triggered = true;
	}

	// Check for subsystem faults
	if (evaluateOverallHealth() == HealthState::CRITICAL) {
		emergency_triggered = true;
	}

	// Check for command timeout in critical situations
	hrt_abstime now = hrt_absolute_time();

	if (_control_state == ControlState::MANUAL_CONTROL || _control_state == ControlState::TASK_EXECUTION) {
		if ((now - _last_command_time) > (_cmd_timeout.get() * 2_s)) { // Extended timeout for safety
			emergency_triggered = true;
			PX4_WARN("Command timeout triggered emergency stop");
		}
	}

	if (emergency_triggered && !_emergency_stop_active) {
		_emergency_stop_active = true;
		_emergency_stop_time = now;
		perf_count(_emergency_stop_perf);
		PX4_WARN("Emergency stop activated");
	}

	// Check for emergency stop reset conditions
	if (_emergency_stop_active && !_current_command.emergency_stop) {
		if ((now - _emergency_stop_time) > (MAX_EMERGENCY_STOP_TIME_S * 1_s)) {
			// Manual reset required - check for explicit reset command
			// This would typically require a specific reset sequence
			if (isSystemHealthy()) {
				_emergency_stop_active = false;
				PX4_INFO("Emergency stop cleared");
			}
		}
	}
}

void WheelLoaderController::updateSubsystemHealth()
{
	hrt_abstime now = hrt_absolute_time();
	float health_timeout_us = _health_timeout.get() * 1_s;

	// Update boom health
	if (_boom_status_sub.updated()) {
		boom_status_s boom_status;

		if (_boom_status_sub.copy(&boom_status)) {
			_last_boom_status_time = now;

			// Determine boom health based on status fields
			if (boom_status.motor_fault || boom_status.encoder_fault) {
				_boom_health = HealthState::ERROR;
			} else if (boom_status.state == 4) {  // Error state
				_boom_health = HealthState::ERROR;
			} else if (boom_status.state == 2 || boom_status.state == 3) {  // Ready or Moving
				_boom_health = HealthState::HEALTHY;
			} else {
				_boom_health = HealthState::WARNING;  // Uninitialized or zeroing
			}
		}

	} else if ((now - _last_boom_status_time) > health_timeout_us) {
		_boom_health = HealthState::ERROR;
	}

	// Update bucket health
	if (_bucket_status_sub.updated()) {
		bucket_status_s bucket_status;

		if (_bucket_status_sub.copy(&bucket_status)) {
			_last_bucket_status_time = now;
			// Map bucket state to health state
			switch (bucket_status.state) {
			case 2: // ready
			case 3: // moving
				_bucket_health = HealthState::HEALTHY;
				break;
			case 1: // zeroing
				_bucket_health = HealthState::WARNING;
				break;
			case 4: // error
			default:
				_bucket_health = HealthState::ERROR;
				break;
			}
		}

	} else if ((now - _last_bucket_status_time) > health_timeout_us) {
		_bucket_health = HealthState::ERROR;
	}

	// Update steering health
	if (_steering_status_sub.updated()) {
		steering_status_s steering_status;

		if (_steering_status_sub.copy(&steering_status)) {
			_last_steering_status_time = now;
			_steering_health = steering_status.is_healthy ? HealthState::HEALTHY : HealthState::ERROR;
		}

	} else if ((now - _last_steering_status_time) > health_timeout_us) {
		_steering_health = HealthState::ERROR;
	}

	// Update wheel health - EKF2-style SubscriptionMultiArray
	uint8_t front_wheel_idx = static_cast<uint8_t>(_front_wheel_idx.get());
	uint8_t rear_wheel_idx = static_cast<uint8_t>(_rear_wheel_idx.get());

	// Check front wheel
	if (front_wheel_idx < _wheel_status_subs.size()) {
		wheel_status_s wheel_status;
		if (_wheel_status_subs[front_wheel_idx].updated() && _wheel_status_subs[front_wheel_idx].copy(&wheel_status)) {
			_last_wheel_status_time[0] = now;
			_front_wheel_health = wheel_status.controller_healthy ? HealthState::HEALTHY : HealthState::ERROR;
		} else if ((now - _last_wheel_status_time[0]) > health_timeout_us) {
			_front_wheel_health = HealthState::ERROR;
		}
	}

	// Check rear wheel
	if (rear_wheel_idx < _wheel_status_subs.size()) {
		wheel_status_s wheel_status;
		if (_wheel_status_subs[rear_wheel_idx].updated() && _wheel_status_subs[rear_wheel_idx].copy(&wheel_status)) {
			_last_wheel_status_time[1] = now;
			_rear_wheel_health = wheel_status.controller_healthy ? HealthState::HEALTHY : HealthState::ERROR;
		} else if ((now - _last_wheel_status_time[1]) > health_timeout_us) {
			_rear_wheel_health = HealthState::ERROR;
		}
	}
}

void WheelLoaderController::handleEmergencyStop()
{
	// Send emergency stop to all subsystems
	wheel_loader_command_s emergency_cmd{};
	emergency_cmd.timestamp = hrt_absolute_time();
	emergency_cmd.emergency_stop = true;

	generateSubsystemCommands(emergency_cmd);

	PX4_WARN("Emergency stop procedure executed");
}

bool WheelLoaderController::isSystemHealthy()
{
	HealthState overall_health = evaluateOverallHealth();
	return overall_health == HealthState::HEALTHY || overall_health == HealthState::WARNING;
}

WheelLoaderController::HealthState WheelLoaderController::evaluateOverallHealth()
{
	HealthState worst_health = HealthState::HEALTHY;

	// Find the worst health state among all subsystems
	HealthState subsystem_health[] = {
		_boom_health,
		_bucket_health,
		_steering_health,
		_front_wheel_health,
		_rear_wheel_health
	};

	for (HealthState health : subsystem_health) {
		if (health > worst_health) {
			worst_health = health;
		}
	}

	return worst_health;
}

void WheelLoaderController::updateParams()
{
	updateParams();

	// Update control rate if parameter changed
	float new_rate = _control_rate.get();

	if (fabsf(new_rate - CONTROL_RATE_HZ) > 0.1f && new_rate > 0.1f && new_rate <= 200.0f) {
		ScheduleClear();
		ScheduleOnInterval(static_cast<uint64_t>(1_s / new_rate));
	}
}

int WheelLoaderController::task_spawn(int argc, char *argv[])
{
	WheelLoaderController *instance = new WheelLoaderController();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;

		} else {
			PX4_ERR("Failed to initialize wheel loader controller");
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

void WheelLoaderController::processVlaCommand()
{
	if (_vla_command_sub.updated()) {
		vla_command_s vla_cmd;

		if (_vla_command_sub.copy(&vla_cmd)) {
			_current_vla_data = vla_cmd;
			_last_vla_time = hrt_absolute_time();
			_vla_valid = vla_cmd.valid_command && !vla_cmd.emergency_stop;

			// Convert VLA command to wheel loader command
			if (_vla_valid && _operation_mode == OperationMode::AUTO_MODE) {
				convertVlaToWheelLoaderCommand(vla_cmd, _vla_command);
				_vla_command.timestamp = vla_cmd.timestamp;
				_vla_command.command_source = wheel_loader_command_s::SOURCE_EXTERNAL;
			}
		}
	}

	// Check for VLA timeout
	if (_vla_valid && (hrt_absolute_time() - _last_vla_time) > (_vla_timeout.get() * 1e6f)) {
		_vla_valid = false;
		PX4_WARN("VLA command timeout - switching to manual mode");
		
		// Auto fallback to manual mode on VLA timeout
		if (_operation_mode == OperationMode::AUTO_MODE) {
			transitionToMode(OperationMode::MANUAL_MODE);
		}
	}
}

void WheelLoaderController::processModeSwitch()
{
	// Check parameter for operation mode change
	int requested_mode = _operation_mode_param.get();
	OperationMode target_mode = (requested_mode == 1) ? OperationMode::AUTO_MODE : OperationMode::MANUAL_MODE;

	// Handle mode transition request
	if (target_mode != _operation_mode && _control_state != ControlState::MODE_TRANSITION) {
		transitionToMode(target_mode);
	}

	// Handle ongoing mode transition
	if (_control_state == ControlState::MODE_TRANSITION) {
		handleModeTransition();
	}
}

void WheelLoaderController::transitionToMode(OperationMode new_mode)
{
	if (!isValidModeTransition(_operation_mode, new_mode)) {
		PX4_WARN("Invalid mode transition from %d to %d", (int)_operation_mode, (int)new_mode);
		return;
	}

	PX4_INFO("Transitioning from mode %d to mode %d", (int)_operation_mode, (int)new_mode);
	
	_previous_operation_mode = _operation_mode;
	_operation_mode = OperationMode::TRANSITION_MODE;
	_mode_transition_start_time = hrt_absolute_time();
	transitionToState(ControlState::MODE_TRANSITION);

	// Set target mode
	if (new_mode == OperationMode::AUTO_MODE) {
		_operation_mode = OperationMode::AUTO_MODE;
	} else {
		_operation_mode = OperationMode::MANUAL_MODE;
	}
}

bool WheelLoaderController::isValidModeTransition(OperationMode from, OperationMode to)
{
	// Manual mode can always override auto mode (safety requirement)
	if (to == OperationMode::MANUAL_MODE) {
		return true;
	}

	// Auto mode requires VLA to be enabled and valid
	if (to == OperationMode::AUTO_MODE) {
		return _vla_enable.get() && _vla_valid;
	}

	return false;
}

void WheelLoaderController::handleModeTransition()
{
	hrt_abstime now = hrt_absolute_time();
	float transition_time = (now - _mode_transition_start_time) / 1e6f;

	// Check if transition time exceeded
	if (transition_time > _mode_transition_time.get()) {
		// Complete transition
		if (_operation_mode == OperationMode::AUTO_MODE) {
			transitionToState(ControlState::AUTO_OPERATION);
			PX4_INFO("Transitioned to AUTO mode");
		} else {
			transitionToState(ControlState::MANUAL_OPERATION);
			PX4_INFO("Transitioned to MANUAL mode");
		}
	}
}

void WheelLoaderController::convertVlaToWheelLoaderCommand(const vla_command_s &vla_cmd, wheel_loader_command_s &wl_cmd)
{
	// Convert VLA outputs to wheel loader commands
	wl_cmd.timestamp = vla_cmd.timestamp;
	wl_cmd.command_source = wheel_loader_command_s::SOURCE_EXTERNAL;

	// Map normalized VLA commands to wheel loader controls
	wl_cmd.forward_speed = math::constrain(vla_cmd.forward_speed, -1.0f, 1.0f);
	wl_cmd.steering_angle = math::constrain(vla_cmd.steering_angle, -1.0f, 1.0f);

	// Hydraulic controls
	wl_cmd.boom_lift = math::constrain(vla_cmd.boom_lift, -1.0f, 1.0f);
	wl_cmd.bucket_tilt = math::constrain(vla_cmd.bucket_tilt, -1.0f, 1.0f);

	// Operation mode mapping
	switch (vla_cmd.operation_mode) {
		case vla_command_s::OPERATION_LOAD:
			wl_cmd.operation_mode = wheel_loader_command_s::MODE_LOAD;
			break;
		case vla_command_s::OPERATION_DUMP:
			wl_cmd.operation_mode = wheel_loader_command_s::MODE_DUMP;
			break;
		default:
			wl_cmd.operation_mode = wheel_loader_command_s::MODE_MANUAL;
			break;
	}

	// Emergency handling
	if (vla_cmd.emergency_stop) {
		wl_cmd.emergency_stop = true;
		wl_cmd.forward_speed = 0.0f;
		wl_cmd.steering_angle = 0.0f;
		wl_cmd.boom_lift = 0.0f;
		wl_cmd.bucket_tilt = 0.0f;
	}
}

void WheelLoaderController::processAutoLoadSequence()
{
	if (_current_vla_data.operation_mode != vla_command_s::OPERATION_LOAD) {
		return;
	}

	// Implement auto load sequence based on operation mode
	PX4_DEBUG("Auto load sequence active");
	// Simplified load sequence - actual implementation would depend on
	// specific wheel loader hydraulic system integration
}

void WheelLoaderController::processAutoDumpSequence()
{
	if (_current_vla_data.operation_mode != vla_command_s::OPERATION_DUMP) {
		return;
	}

	// Implement auto dump sequence based on operation mode
	PX4_DEBUG("Auto dump sequence active");
	// Simplified dump sequence - actual implementation would depend on
	// specific wheel loader hydraulic system integration
}

int WheelLoaderController::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Wheel loader controller module for coordinating subsystem operation, command arbitration,
state management, and safety oversight.

The module manages:
- Command arbitration between manual, autonomous, and external sources
- State machine for operational modes
- Safety monitoring and emergency procedures
- Coordination of boom, bucket, steering, and wheel subsystems

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("wheel_loader_controller", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int WheelLoaderController::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	if (!strcmp(argv[0], "status")) {
		return get_instance()->print_status();
	}

	return print_usage("unknown command");
}
