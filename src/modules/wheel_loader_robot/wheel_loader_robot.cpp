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

#include "wheel_loader_robot.hpp"

WheelLoaderRobot::WheelLoaderRobot() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	_loop_perf = perf_alloc(PC_ELAPSED, MODULE_NAME": cycle");
	_power_perf = perf_alloc(PC_ELAPSED, MODULE_NAME": power");
}

WheelLoaderRobot::~WheelLoaderRobot()
{
	perf_free(_loop_perf);
	perf_free(_power_perf);
}

bool WheelLoaderRobot::init()
{
	update_parameters();

	// Initialize power state
	_power_state.power_budget_w = _param_max_power.get();
	_power_state.mode = PowerMode::NORMAL;

	_state = SystemState::STANDBY;

	ScheduleOnInterval(CONTROL_PERIOD_US);

	return true;
}

void WheelLoaderRobot::Run()
{
	if (should_exit()) {
		ScheduleClear();
		return;
	}

	perf_begin(_loop_perf);

	// Update parameters if needed
	if (_parameter_update_sub.updated()) {
		update_parameters();
	}

	// Update inputs
	update_inputs();

	// Update power state
	if (hrt_elapsed_time(&_last_power_update) >= POWER_UPDATE_PERIOD_US) {
		perf_begin(_power_perf);
		update_power_state();
		perf_end(_power_perf);
		_last_power_update = hrt_absolute_time();
	}

	// Update system state
	update_system_state();

	// Execute commands if operating
	if (_state == SystemState::OPERATING || _state == SystemState::POWER_LIMITED) {
		execute_commands();
	}

	// Publish status
	publish_status();

	perf_end(_loop_perf);
}

void WheelLoaderRobot::update_inputs()
{
	// Process manual control input
	manual_control_setpoint_s manual;
	if (_manual_control_sub.update(&manual)) {
		_manual_input = process_manual_input(manual);
	}

	// Process VLA input if in autonomous mode
	if (_mode == OperationMode::AUTONOMOUS) {
		vla_command_s vla;
		if (_vla_command_sub.update(&vla)) {
			_vla_input = process_vla_input(vla);
		}
	}
}

void WheelLoaderRobot::update_power_state()
{
	// Update battery status
	battery_status_s battery;
	if (_battery_status_sub.update(&battery)) {
		_power_state.battery_voltage_v = battery.voltage_v;
		_power_state.battery_current_a = battery.current_a;
		_power_state.battery_remaining_pct = battery.remaining * 100.0f;
		_power_state.is_charging = battery.current_a < -0.1f;
	}

	// Update power consumption
	power_monitor_s power;
	if (_power_monitor_sub.update(&power)) {
		_power_state.power_consumption_w = power.voltage_v * power.current_a;
	}

	// Determine power mode
	_power_state.mode = determine_power_mode();

	// Calculate power budget
	calculate_power_budget();

	// Calculate efficiency factor
	_power_state.efficiency_factor = calculate_efficiency_factor();
}

WheelLoaderRobot::PowerMode WheelLoaderRobot::determine_power_mode()
{
	if (_power_state.is_critical()) {
		return PowerMode::CRITICAL;
	} else if (_power_state.is_low()) {
		return PowerMode::ECO;
	} else if (_power_state.battery_remaining_pct > 80.0f && !_power_state.is_charging) {
		return PowerMode::BOOST;
	} else {
		return PowerMode::NORMAL;
	}
}

void WheelLoaderRobot::calculate_power_budget()
{
	switch (_power_state.mode) {
	case PowerMode::CRITICAL:
		_power_state.power_budget_w = _param_max_power.get() * 0.3f;
		break;
	case PowerMode::ECO:
		_power_state.power_budget_w = _param_eco_power.get();
		break;
	case PowerMode::BOOST:
		_power_state.power_budget_w = _param_max_power.get() * 1.2f;
		break;
	default:
		_power_state.power_budget_w = _param_max_power.get();
		break;
	}
}

float WheelLoaderRobot::calculate_efficiency_factor()
{
	// Simple efficiency model based on battery voltage
	float nominal_voltage = 48.0f;  // Nominal 48V system
	float voltage_factor = _power_state.battery_voltage_v / nominal_voltage;

	// Efficiency decreases with low voltage
	return math::constrain(voltage_factor, 0.7f, 1.1f);
}

void WheelLoaderRobot::update_system_state()
{
	SystemState new_state = _state;

	// Update subsystem health
	update_subsystem_health();

	switch (_state) {
	case SystemState::INIT:
		if (_health.is_healthy()) {
			new_state = SystemState::STANDBY;
		}
		break;

	case SystemState::STANDBY:
		if (_manual_input.valid || (_vla_input.valid && _mode == OperationMode::AUTONOMOUS)) {
			if (_power_state.is_critical()) {
				new_state = SystemState::POWER_LIMITED;
			} else {
				new_state = SystemState::OPERATING;
			}
		} else if (check_emergency_conditions()) {
			new_state = SystemState::EMERGENCY;
		} else if (!_health.is_healthy()) {
			new_state = SystemState::FAULT;
		}
		break;

	case SystemState::OPERATING:
		if (check_emergency_conditions()) {
			new_state = SystemState::EMERGENCY;
		} else if (!_health.is_healthy()) {
			new_state = SystemState::FAULT;
		} else if (_power_state.is_critical()) {
			new_state = SystemState::POWER_LIMITED;
		} else if (!_manual_input.valid && !(_vla_input.valid && _mode == OperationMode::AUTONOMOUS)) {
			new_state = SystemState::STANDBY;
		}
		break;

	case SystemState::POWER_LIMITED:
		if (check_emergency_conditions()) {
			new_state = SystemState::EMERGENCY;
		} else if (!_health.is_healthy()) {
			new_state = SystemState::FAULT;
		} else if (!_power_state.is_critical() && (_manual_input.valid ||
			   (_vla_input.valid && _mode == OperationMode::AUTONOMOUS))) {
			new_state = SystemState::OPERATING;
		} else if (!_manual_input.valid && !(_vla_input.valid && _mode == OperationMode::AUTONOMOUS)) {
			new_state = SystemState::STANDBY;
		}
		break;

	case SystemState::EMERGENCY:
		// Requires manual reset
		if (!check_emergency_conditions() && _health.is_healthy()) {
			new_state = SystemState::STANDBY;
		}
		break;

	case SystemState::FAULT:
		if (_health.is_healthy()) {
			new_state = SystemState::STANDBY;
		}
		break;
	}

	if (new_state != _state && can_transition_to(new_state)) {
		transition_to(new_state);
	}
}

void WheelLoaderRobot::execute_commands()
{
	// Arbitrate between command sources
	_active_command = arbitrate_commands();

	// Validate command
	if (!validate_command(_active_command)) {
		PX4_WARN("Command validation failed");
		return;
	}

	// Apply power limits
	apply_power_limits(_active_command);

	// Apply general command limits
	apply_command_limits(_active_command);

	// Send commands to subsystems
	send_chassis_command(_active_command);
	send_boom_command(_active_command);
	send_bucket_command(_active_command);
}

wheel_loader_command_s WheelLoaderRobot::arbitrate_commands()
{
	wheel_loader_command_s cmd{};

	// Priority: Manual > Autonomous
	if (_manual_input.valid) {
		cmd = _manual_input.command;
	} else if (_vla_input.valid && _mode == OperationMode::AUTONOMOUS) {
		cmd = _vla_input.command;
	}

	return cmd;
}

void WheelLoaderRobot::apply_power_limits(wheel_loader_command_s &cmd)
{
	// Estimate power requirements
	_chassis_power_w = fabsf(cmd.linear_velocity) * _param_max_power.get() * 0.4f;
	_boom_power_w = fabsf(cmd.boom_velocity) * _param_max_power.get() * 0.3f;
	_bucket_power_w = fabsf(cmd.bucket_velocity) * _param_max_power.get() * 0.3f;

	_total_power_request_w = _chassis_power_w + _boom_power_w + _bucket_power_w;

	// Apply power budget limit
	if (_total_power_request_w > _power_state.power_budget_w) {
		float scale_factor = _power_state.power_budget_w / _total_power_request_w;

		// Prioritize chassis movement
		if (scale_factor < 0.7f) {
			cmd.boom_velocity *= 0.5f;
			cmd.bucket_velocity *= 0.5f;
		}

		cmd.linear_velocity *= scale_factor;
		cmd.angular_velocity *= scale_factor;
	}
}

void WheelLoaderRobot::send_chassis_command(const wheel_loader_command_s &cmd)
{
	chassis_command_s chassis_cmd{};
	chassis_cmd.timestamp = hrt_absolute_time();
	chassis_cmd.linear_velocity = cmd.linear_velocity;
	chassis_cmd.angular_velocity = cmd.angular_velocity;
	chassis_cmd.mode = chassis_command_s::MODE_VELOCITY;
	chassis_cmd.emergency_stop = (_state == SystemState::EMERGENCY);

	_chassis_cmd_pub.publish(chassis_cmd);
}

void WheelLoaderRobot::send_boom_command(const wheel_loader_command_s &cmd)
{
	boom_command_s boom_cmd{};
	boom_cmd.timestamp = hrt_absolute_time();
	boom_cmd.velocity = cmd.boom_velocity;
	boom_cmd.emergency_stop = (_state == SystemState::EMERGENCY);

	_boom_cmd_pub.publish(boom_cmd);
}

void WheelLoaderRobot::send_bucket_command(const wheel_loader_command_s &cmd)
{
	bucket_command_s bucket_cmd{};
	bucket_cmd.timestamp = hrt_absolute_time();
	bucket_cmd.velocity = cmd.bucket_velocity;
	bucket_cmd.emergency_stop = (_state == SystemState::EMERGENCY);

	_bucket_cmd_pub.publish(bucket_cmd);
}

void WheelLoaderRobot::publish_status()
{
	wheel_loader_status_s status{};
	status.timestamp = hrt_absolute_time();

	// System state
	status.state = static_cast<uint8_t>(_state);
	status.mode = static_cast<uint8_t>(_mode);

	// Power information
	status.battery_voltage = _power_state.battery_voltage_v;
	status.battery_current = _power_state.battery_current_a;
	status.battery_remaining = _power_state.battery_remaining_pct;
	status.power_consumption = _power_state.power_consumption_w;
	status.power_budget = _power_state.power_budget_w;
	status.power_mode = static_cast<uint8_t>(_power_state.mode);

	// Health status
	status.chassis_ok = _health.chassis_ok;
	status.boom_ok = _health.boom_ok;
	status.bucket_ok = _health.bucket_ok;
	status.battery_ok = _health.battery_ok;

	// Active command
	status.active_linear_velocity = _active_command.linear_velocity;
	status.active_angular_velocity = _active_command.angular_velocity;
	status.active_boom_velocity = _active_command.boom_velocity;
	status.active_bucket_velocity = _active_command.bucket_velocity;

	_status_pub.publish(status);
}

WheelLoaderRobot::CommandInput WheelLoaderRobot::process_manual_input(const manual_control_setpoint_s &manual)
{
	CommandInput input{};
	input.timestamp = hrt_absolute_time();
	input.valid = manual.valid;

	if (manual.valid) {
		// Map manual control to wheel loader commands
		input.command.linear_velocity = manual.x * _param_max_speed.get();
		input.command.angular_velocity = manual.r * M_PI_F;
		input.command.boom_velocity = manual.y * 0.5f;
		input.command.bucket_velocity = manual.z * 0.5f;
	}

	return input;
}

WheelLoaderRobot::CommandInput WheelLoaderRobot::process_vla_input(const vla_command_s &vla)
{
	CommandInput input{};
	input.timestamp = hrt_absolute_time();
	input.valid = vla.valid_output && !vla.emergency_stop;

	if (input.valid) {
		// Convert VLA bucket end effector setpoints to wheel loader commands
		// This requires inverse kinematics to determine vehicle position and boom/bucket angles

		// Simplified inverse kinematics for wheel loader
		// In a real implementation, this would use proper kinematic chains

		static float prev_bucket_x = 0.0f, prev_bucket_y = 0.0f, prev_bucket_z = 0.0f;
		static float prev_bucket_pitch = 0.0f;

		// Compute bucket position deltas
		float dx = vla.bucket_position_x - prev_bucket_x;
		float dy = vla.bucket_position_y - prev_bucket_y;
		float dz = vla.bucket_position_z - prev_bucket_z;
		float dpitch = vla.bucket_orientation_pitch - prev_bucket_pitch;

		// Convert bucket end effector motion to vehicle and joint motion
		float dt = 0.02f;  // 20ms control period

		// Vehicle motion: move base to position bucket appropriately
		// Simplified: assume bucket motion in X-Y requires vehicle movement
		float vehicle_distance = sqrtf(dx*dx + dy*dy);
		input.command.linear_velocity = vehicle_distance / dt;

		// Vehicle turning: align with bucket yaw target
		input.command.angular_velocity = (vla.bucket_orientation_yaw - atan2f(dy, dx)) / dt;

		// Boom motion: height changes primarily affect boom angle
		// Simplified mapping: vertical motion maps to boom velocity
		input.command.boom_velocity = dz / dt;

		// Bucket motion: pitch changes affect bucket angle
		// Bucket pitch controls digging/dumping action
		input.command.bucket_velocity = dpitch / dt;

		// Apply velocity limits for safety
		input.command.linear_velocity = math::constrain(input.command.linear_velocity, -2.0f, 2.0f);
		input.command.angular_velocity = math::constrain(input.command.angular_velocity, -M_PI_F/2, M_PI_F/2);
		input.command.boom_velocity = math::constrain(input.command.boom_velocity, -0.5f, 0.5f);
		input.command.bucket_velocity = math::constrain(input.command.bucket_velocity, -1.0f, 1.0f);

		// Update previous values for next iteration
		prev_bucket_x = vla.bucket_position_x;
		prev_bucket_y = vla.bucket_position_y;
		prev_bucket_z = vla.bucket_position_z;
		prev_bucket_pitch = vla.bucket_orientation_pitch;

		PX4_DEBUG("VLA bucket target: (%.2f, %.2f, %.2f) pitch=%.2f -> vel:(%.2f, %.2f, %.2f, %.2f)",
			  (double)vla.bucket_position_x, (double)vla.bucket_position_y, (double)vla.bucket_position_z,
			  (double)vla.bucket_orientation_pitch, (double)input.command.linear_velocity,
			  (double)input.command.angular_velocity, (double)input.command.boom_velocity,
			  (double)input.command.bucket_velocity);
	}

	return input;
}

bool WheelLoaderRobot::validate_command(const wheel_loader_command_s &cmd)
{
	// Check for NaN values
	if (!PX4_ISFINITE(cmd.linear_velocity) || !PX4_ISFINITE(cmd.angular_velocity) ||
	    !PX4_ISFINITE(cmd.boom_velocity) || !PX4_ISFINITE(cmd.bucket_velocity)) {
		return false;
	}

	// Check velocity limits
	if (fabsf(cmd.linear_velocity) > _param_max_speed.get() ||
	    fabsf(cmd.angular_velocity) > M_PI_F) {
		return false;
	}

	return true;
}

void WheelLoaderRobot::apply_command_limits(wheel_loader_command_s &cmd)
{
	// Apply velocity limits
	cmd.linear_velocity = math::constrain(cmd.linear_velocity,
					      -_param_max_speed.get(),
					      _param_max_speed.get());
	cmd.angular_velocity = math::constrain(cmd.angular_velocity, -M_PI_F, M_PI_F);
	cmd.boom_velocity = math::constrain(cmd.boom_velocity, -1.0f, 1.0f);
	cmd.bucket_velocity = math::constrain(cmd.bucket_velocity, -1.0f, 1.0f);
}

bool WheelLoaderRobot::check_emergency_conditions()
{
	// Check for emergency stop parameter
	if (_param_estop_enable.get() && _power_state.battery_voltage_v < 20.0f) {
		return true;
	}

	// Check for critical system failures
	if (!_health.battery_ok && _power_state.battery_remaining_pct < 5.0f) {
		return true;
	}

	return false;
}

void WheelLoaderRobot::update_subsystem_health()
{
	hrt_abstime now = hrt_absolute_time();

	// Update chassis health
	chassis_status_s chassis_status;
	if (_chassis_status_sub.update(&chassis_status)) {
		_health.chassis_ok = (chassis_status.health == chassis_status_s::HEALTH_OK);
		_health.last_update = now;
	}

	// Update boom health
	boom_status_s boom_status;
	if (_boom_status_sub.update(&boom_status)) {
		_health.boom_ok = boom_status.healthy;
	}

	// Update bucket health
	bucket_status_s bucket_status;
	if (_bucket_status_sub.update(&bucket_status)) {
		_health.bucket_ok = bucket_status.healthy;
	}

	// Update battery health
	_health.battery_ok = (_power_state.battery_voltage_v > _param_critical_battery.get());
}

void WheelLoaderRobot::transition_to(SystemState new_state)
{
	PX4_INFO("State transition: %d -> %d", static_cast<int>(_state), static_cast<int>(new_state));
	_state = new_state;
	_state_entry_time = hrt_absolute_time();
}

bool WheelLoaderRobot::can_transition_to(SystemState new_state) const
{
	// Define valid state transitions
	switch (_state) {
	case SystemState::INIT:
		return (new_state == SystemState::STANDBY || new_state == SystemState::FAULT);

	case SystemState::STANDBY:
		return true;  // Can transition to any state from standby

	case SystemState::OPERATING:
		return (new_state != SystemState::INIT);

	case SystemState::POWER_LIMITED:
		return (new_state != SystemState::INIT);

	case SystemState::EMERGENCY:
		return (new_state == SystemState::STANDBY || new_state == SystemState::FAULT);

	case SystemState::FAULT:
		return (new_state == SystemState::STANDBY || new_state == SystemState::EMERGENCY);
	}

	return false;
}

void WheelLoaderRobot::update_parameters()
{
	updateParams();
}

int WheelLoaderRobot::task_spawn(int argc, char *argv[])
{
	WheelLoaderRobot *instance = new WheelLoaderRobot();

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

int WheelLoaderRobot::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int WheelLoaderRobot::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Wheel loader robot central coordinator with integrated power management.

This module manages:
- Command arbitration between manual and autonomous control
- Battery-aware power management and optimization
- Subsystem coordination (chassis, boom, bucket)
- Safety monitoring and emergency response

The module prioritizes safety and power efficiency while providing
seamless operation across different power states.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("wheel_loader_robot", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}
