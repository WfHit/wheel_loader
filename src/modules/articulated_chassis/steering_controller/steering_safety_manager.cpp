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

#include "steering_safety_manager.hpp"
#include "steering_servo_controller.hpp"
#include "steering_limit_sensor.hpp"

SteeringSafetyManager::SteeringSafetyManager(ModuleParams *parent)
	: ModuleParams(parent)
{
	// Initialize safety state
	_safety_state.safe_position_rad = 0.0f;
}

void SteeringSafetyManager::update_safety_state(const SteeringServoController &servo_controller,
		const SteeringLimitSensor &limit_sensor,
		float commanded_angle, float current_angle)
{
	if (!is_enabled()) {
		return;
	}

	// Update safety position from parameter
	_safety_state.safe_position_rad = _safety_position_rad.get();

	// Check various safety conditions
	check_emergency_stop();
	check_servo_faults(servo_controller);
	check_sensor_faults(limit_sensor);
	check_position_error(commanded_angle, current_angle);

	// Clear safety violations after timeout
	clear_safety_violations_if_timeout();
}

bool SteeringSafetyManager::is_safe() const
{
	if (!is_enabled()) {
		return true; // Consider safe if disabled
	}

	return !_safety_state.safety_violation &&
	       !_safety_state.position_limit_violation &&
	       !_safety_state.emergency_stop_active &&
	       !_safety_state.servo_fault &&
	       !_safety_state.sensor_fault;
}

void SteeringSafetyManager::handle_safety_violation(SteeringServoController &servo_controller)
{
	_safety_state.safety_violation = true;
	_safety_state.violation_count++;
	_safety_state.last_violation_time = hrt_absolute_time();

	// Command to safe position
	servo_controller.send_position_command(_safety_state.safe_position_rad, 0.0f);
}

void SteeringSafetyManager::check_emergency_stop()
{
	vehicle_status_s vehicle_status{};

	if (_vehicle_status_sub.update(&vehicle_status)) {
		_emergency_stop = (vehicle_status.nav_state == vehicle_status_s::NAVIGATION_STATE_TERMINATION);
	}

	if (_emergency_stop) {
		_safety_state.emergency_stop_active = true;
	} else {
		_safety_state.emergency_stop_active = false;
	}
}

void SteeringSafetyManager::check_servo_faults(const SteeringServoController &servo_controller)
{
	const auto &servo_state = servo_controller.get_state();

	if (servo_state.feedback_valid && (servo_state.error_flags != 0)) {
		_safety_state.servo_fault = true;
	} else {
		_safety_state.servo_fault = false;
	}
}

void SteeringSafetyManager::check_sensor_faults(const SteeringLimitSensor &limit_sensor)
{
	if (limit_sensor.are_enabled() && !limit_sensor.are_sensors_healthy()) {
		_safety_state.sensor_fault = true;
	} else {
		_safety_state.sensor_fault = false;
	}
}

void SteeringSafetyManager::check_position_error(float commanded_angle, float current_angle)
{
	float position_error = fabsf(commanded_angle - current_angle);

	if (position_error > _max_position_error.get()) {
		_safety_state.position_limit_violation = true;
	}
}

void SteeringSafetyManager::clear_safety_violations_if_timeout()
{
	const uint64_t now = hrt_absolute_time();
	const uint64_t timeout_us = _fault_timeout_ms.get() * 1000;

	if (_safety_state.safety_violation &&
	    (now - _safety_state.last_violation_time) > timeout_us) {
		if (_safety_state.violation_count < (uint32_t)_max_violations.get()) {
			_safety_state.safety_violation = false;
			_safety_state.position_limit_violation = false;
		}
	}
}