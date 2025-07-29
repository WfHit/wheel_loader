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

#include "wheel_loader_rc_input.hpp"

#include <px4_platform_common/log.h>
#include <lib/mathlib/mathlib.h>
#include <cstring>

WheelLoaderRcInput::WheelLoaderRcInput() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
}

WheelLoaderRcInput::~WheelLoaderRcInput()
{
	perf_free(_cycle_perf);
	perf_free(_rc_timeout_perf);
}

bool WheelLoaderRcInput::init()
{
	// Initialize command structure
	memset(&_current_command, 0, sizeof(_current_command));
	_current_command.command_source = wheel_loader_command_s::SOURCE_MANUAL_CONTROL;
	_current_command.drive_mode = wheel_loader_command_s::DRIVE_MODE_MANUAL;
	_current_command.hydraulic_mode = wheel_loader_command_s::HYDRAULIC_MODE_MANUAL;
	_current_command.enable_drivetrain = true;
	_current_command.enable_hydraulics = true;
	_current_command.traction_control_enabled = true;
	_current_command.stability_control_enabled = true;

	// Start scheduled work
	ScheduleOnInterval(RC_UPDATE_INTERVAL_US);

	return true;
}

void WheelLoaderRcInput::Run()
{
	perf_begin(_cycle_perf);

	// Check for parameter updates
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParameters();
	}

	// Only process if RC input is enabled
	if (!_param_rc_enable.get()) {
		perf_end(_cycle_perf);
		return;
	}

	// Process RC input
	processRcInput();

	// Process manual control input as backup
	processManualControlInput();

	// Check for timeout and handle failsafe
	const hrt_abstime now = hrt_absolute_time();
	if ((now - _last_valid_input_time) > (hrt_abstime)(_param_rc_timeout.get() * 1e6f)) {
		handleFailsafe();
	}

	// Publish wheel loader command if we have valid input
	if (!_failsafe_active) {
		publishWheelLoaderCommand();
	}

	perf_end(_cycle_perf);
}

void WheelLoaderRcInput::processRcInput()
{
	input_rc_s rc_input;
	if (_input_rc_sub.update(&rc_input)) {
		_last_rc_input_time = hrt_absolute_time();

		// Check if RC input is valid
		if (!isValidRcInput(rc_input)) {
			return;
		}

		_rc_input_available = true;
		_last_valid_input_time = _last_rc_input_time;

		// Check emergency stop first
		if (isEmergencyStopActive(rc_input)) {
			_current_command.emergency_stop = true;
			_failsafe_active = true;
			PX4_WARN("Emergency stop activated via RC");
			return;
		}

		_current_command.emergency_stop = false;
		_failsafe_active = false;

		// Map RC channels to wheel loader commands
		
		// Chassis velocity control (throttle channel)
		float throttle = mapRcChannel(rc_input.values[RC_CHANNEL_THROTTLE]);
		throttle = applyDeadzone(throttle, _param_rc_deadzone.get());
		float chassis_speed = throttle * _param_max_speed.get();
		
		// Steering control
		float steering = mapRcChannel(rc_input.values[RC_CHANNEL_STEERING]);
		steering = applyDeadzone(steering, _param_rc_deadzone.get());
		float steering_angle = steering * _param_max_steering.get();

		// Convert to axle speeds (front and rear axles controlled as units)
		float speed_factor = cosf(steering_angle * 0.5f); // Reduce speed when turning
		// Front axle - both wheels controlled as a unit
		_current_command.front_left_wheel_speed = chassis_speed * speed_factor;
		_current_command.front_right_wheel_speed = chassis_speed * speed_factor;
		// Rear axle - both wheels controlled as a unit with differential for steering
		_current_command.rear_left_wheel_speed = chassis_speed * (1.0f - steering);
		_current_command.rear_right_wheel_speed = chassis_speed * (1.0f + steering);

		// Boom control (angle/height based - position control)
		if (rc_input.channel_count > RC_CHANNEL_BOOM) {
			float boom_input = mapRcChannel(rc_input.values[RC_CHANNEL_BOOM]);
			boom_input = applyDeadzone(boom_input, _param_rc_deadzone.get());
			// Boom uses angle/height control (position-based)
			_current_command.boom_lift_cmd = boom_input * _param_max_boom_angle.get();
		}

		// Bucket control (velocity based - rate control)
		if (rc_input.channel_count > RC_CHANNEL_BUCKET) {
			float bucket_input = mapRcChannel(rc_input.values[RC_CHANNEL_BUCKET]);
			bucket_input = applyDeadzone(bucket_input, _param_rc_deadzone.get());
			// Bucket uses velocity control (rate-based)
			_current_command.bucket_angle_cmd = bucket_input * _param_max_bucket_velocity.get();
		}

		// Steering angle command
		_current_command.steering_angle_cmd = steering_angle;
		_current_command.articulation_angle_cmd = steering_angle * 0.5f; // Half for articulation

		// Apply safety limits
		applySafetyLimits(_current_command);

		// Update timestamp
		_current_command.timestamp = hrt_absolute_time();
	}
}

void WheelLoaderRcInput::processManualControlInput()
{
	// Use manual control setpoint as backup if RC not available
	if (_rc_input_available) {
		return;
	}

	manual_control_setpoint_s manual_control;
	if (_manual_control_sub.update(&manual_control)) {
		_last_manual_control_time = hrt_absolute_time();

		if (!manual_control.valid || manual_control.data_source != manual_control_setpoint_s::SOURCE_RC) {
			return;
		}

		_last_valid_input_time = _last_manual_control_time;
		_failsafe_active = false;

		// Map manual control to wheel loader commands
		float throttle = applyDeadzone(manual_control.throttle, _param_rc_deadzone.get());
		float roll = applyDeadzone(manual_control.roll, _param_rc_deadzone.get());
		
		float chassis_speed = throttle * _param_max_speed.get();
		float steering_angle = roll * _param_max_steering.get();

		// Convert to wheel speeds
		float speed_factor = cosf(steering_angle * 0.5f);
		_current_command.front_left_wheel_speed = chassis_speed * speed_factor;
		_current_command.front_right_wheel_speed = chassis_speed * speed_factor;
		_current_command.rear_left_wheel_speed = chassis_speed * (1.0f - roll);
		_current_command.rear_right_wheel_speed = chassis_speed * (1.0f + roll);

		_current_command.steering_angle_cmd = steering_angle;
		_current_command.articulation_angle_cmd = steering_angle * 0.5f;

		// Hydraulic controls from aux channels
		if (!isnan(manual_control.aux1)) {
			_current_command.boom_lift_cmd = applyDeadzone(manual_control.aux1, _param_rc_deadzone.get()) * _param_max_boom_angle.get();
		}
		if (!isnan(manual_control.aux2)) {
			_current_command.bucket_angle_cmd = applyDeadzone(manual_control.aux2, _param_rc_deadzone.get()) * _param_max_bucket_velocity.get();
		}

		applySafetyLimits(_current_command);
		_current_command.timestamp = hrt_absolute_time();
	}
}

void WheelLoaderRcInput::publishWheelLoaderCommand()
{
	_wheel_loader_command_pub.publish(_current_command);
}

float WheelLoaderRcInput::mapRcChannel(uint16_t raw_value, float min_out, float max_out)
{
	// Standard RC range is 1000-2000 microseconds
	constexpr float RC_MIN = 1000.0f;
	constexpr float RC_MAX = 2000.0f;
	constexpr float RC_CENTER = 1500.0f;

	// Handle invalid input
	if (raw_value < RC_MIN || raw_value > RC_MAX) {
		return 0.0f;
	}

	// Normalize to [-1, 1]
	float normalized = (static_cast<float>(raw_value) - RC_CENTER) / (RC_CENTER - RC_MIN);
	
	// Map to output range
	return math::constrain(normalized * (max_out - min_out) / 2.0f + (max_out + min_out) / 2.0f, min_out, max_out);
}

float WheelLoaderRcInput::applyDeadzone(float input, float deadzone)
{
	if (fabsf(input) < deadzone) {
		return 0.0f;
	}
	
	// Scale output to maintain full range outside deadzone
	float sign = (input >= 0.0f) ? 1.0f : -1.0f;
	return sign * (fabsf(input) - deadzone) / (1.0f - deadzone);
}

bool WheelLoaderRcInput::isValidRcInput(const input_rc_s &rc)
{
	// Check basic validity
	if (rc.rc_lost || rc.rc_failsafe || rc.channel_count < 4) {
		return false;
	}

	// Check signal age
	const hrt_abstime now = hrt_absolute_time();
	if ((now - rc.timestamp_last_signal) > (hrt_abstime)(RC_TIMEOUT_S * 1e6f)) {
		return false;
	}

	// Check essential channels are in valid range
	for (int i = 0; i < 4 && i < rc.channel_count; i++) {
		if (rc.values[i] < 800 || rc.values[i] > 2200) {
			return false;
		}
	}

	return true;
}

bool WheelLoaderRcInput::isEmergencyStopActive(const input_rc_s &rc)
{
	// Check emergency stop channel if available
	if (rc.channel_count > RC_CHANNEL_ESTOP) {
		// Emergency stop active if channel is in lower position
		return rc.values[RC_CHANNEL_ESTOP] < 1400;
	}
	
	return false;
}

void WheelLoaderRcInput::applySafetyLimits(wheel_loader_command_s &cmd)
{
	// Limit wheel speeds
	cmd.front_left_wheel_speed = math::constrain(cmd.front_left_wheel_speed, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);
	cmd.front_right_wheel_speed = math::constrain(cmd.front_right_wheel_speed, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);
	cmd.rear_left_wheel_speed = math::constrain(cmd.rear_left_wheel_speed, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);
	cmd.rear_right_wheel_speed = math::constrain(cmd.rear_right_wheel_speed, -MAX_WHEEL_SPEED, MAX_WHEEL_SPEED);

	// Limit steering angles
	cmd.steering_angle_cmd = math::constrain(cmd.steering_angle_cmd, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE);
	cmd.articulation_angle_cmd = math::constrain(cmd.articulation_angle_cmd, -MAX_STEERING_ANGLE, MAX_STEERING_ANGLE);

	// Limit hydraulic commands (boom: angle, bucket: velocity)
	cmd.boom_lift_cmd = math::constrain(cmd.boom_lift_cmd, -MAX_BOOM_ANGLE, MAX_BOOM_ANGLE);
	cmd.bucket_angle_cmd = math::constrain(cmd.bucket_angle_cmd, -MAX_BUCKET_VELOCITY, MAX_BUCKET_VELOCITY);

	// Set max limits
	cmd.max_vehicle_speed = _param_max_speed.get();
	cmd.max_slip_ratio = 0.2f; // 20% slip threshold
	cmd.traction_gain = 0.5f;  // Moderate traction control response
}

void WheelLoaderRcInput::handleFailsafe()
{
	if (!_failsafe_active) {
		PX4_WARN("RC input timeout - activating failsafe");
		perf_count(_rc_timeout_perf);
		_failsafe_active = true;
	}

	// Stop all motion
	memset(&_current_command, 0, sizeof(_current_command));
	_current_command.command_source = wheel_loader_command_s::SOURCE_MANUAL_CONTROL;
	_current_command.emergency_stop = true;
	_current_command.timestamp = hrt_absolute_time();
	
	publishWheelLoaderCommand();
}

bool WheelLoaderRcInput::isRcInputValid()
{
	const hrt_abstime now = hrt_absolute_time();
	return !_failsafe_active && 
	       ((now - _last_valid_input_time) < (hrt_abstime)(_param_rc_timeout.get() * 1e6f));
}

void WheelLoaderRcInput::updateParameters()
{
	updateParams();
}