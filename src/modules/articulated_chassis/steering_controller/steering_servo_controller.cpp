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

#include "steering_servo_controller.hpp"

SteeringServoController::SteeringServoController(ModuleParams *parent)
	: ModuleParams(parent)
{
	// Initialize position history for filtering
	for (int i = 0; i < FILTER_SIZE; i++) {
		_position_history[i] = 0.0f;
	}
}

void SteeringServoController::send_position_command(float position_rad, float velocity_rad_s)
{
	robotic_servo_command_s cmd{};
	cmd.timestamp = hrt_absolute_time();
	cmd.id = _servo_id.get();
	cmd.command_type = 0; // position control
	cmd.goal_position = position_rad;
	cmd.goal_velocity = velocity_rad_s;
	cmd.goal_current = _current_limit.get();
	cmd.torque_enable = true;

	_servo_command_pub.publish(cmd);
}

void SteeringServoController::update_feedback(float dt)
{
	robotic_servo_feedback_s feedback{};

	if (_servo_feedback_sub.update(&feedback)) {
		if (feedback.id == _servo_id.get()) {
			_servo_state.last_feedback_time = hrt_absolute_time();
			_servo_state.position_rad = feedback.position;
			_servo_state.velocity_rad_s = feedback.velocity;
			_servo_state.current_a = feedback.current;
			_servo_state.temperature_c = feedback.temperature;
			_servo_state.error_flags = feedback.error_flags;
			_servo_state.torque_enabled = feedback.torque_enabled;
			_servo_state.feedback_valid = true;

			// Update position filter
			update_position_filter(feedback.position);
		}
	}

	// Check for feedback timeout
	const uint64_t now = hrt_absolute_time();
	const uint64_t timeout_us = _feedback_timeout_ms.get() * 1000;

	if ((now - _servo_state.last_feedback_time) > timeout_us) {
		_servo_state.feedback_valid = false;
	}
}

bool SteeringServoController::is_healthy() const
{
	return _servo_state.feedback_valid && (_servo_state.error_flags == 0);
}

void SteeringServoController::update_position_filter(float position)
{
	_position_history[_history_index] = position;
	_history_index = (_history_index + 1) % FILTER_SIZE;

	// Calculate moving average
	float sum = 0.0f;

	for (int i = 0; i < FILTER_SIZE; i++) {
		sum += _position_history[i];
	}

	_filtered_position = sum / FILTER_SIZE;
}