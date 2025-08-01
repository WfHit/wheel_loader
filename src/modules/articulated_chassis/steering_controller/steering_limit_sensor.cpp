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

#include "steering_limit_sensor.hpp"

SteeringLimitSensor::SteeringLimitSensor(ModuleParams *parent)
	: ModuleParams(parent)
{
	// Initialize instances to invalid values
	_limit_state.left_instance = 255;
	_limit_state.right_instance = 255;
}

void SteeringLimitSensor::update()
{
	if (!are_enabled()) {
		return;
	}

	// Update limit sensor instances from parameters if needed
	if (_limit_state.left_instance != _limit_left_instance.get()) {
		_limit_state.left_instance = _limit_left_instance.get();
	}

	if (_limit_state.right_instance != _limit_right_instance.get()) {
		_limit_state.right_instance = _limit_right_instance.get();
	}

	// Process sensor data
	limit_sensor_s sensor_data{};

	while (_limit_sensor_sub.update(&sensor_data)) {
		if (sensor_data.instance == _limit_state.left_instance) {
			_limit_state.left_last_update = hrt_absolute_time();
			_limit_state.left_limit_active = sensor_data.state;
			_limit_state.left_limit_healthy = !sensor_data.redundancy_fault;
		} else if (sensor_data.instance == _limit_state.right_instance) {
			_limit_state.right_last_update = hrt_absolute_time();
			_limit_state.right_limit_active = sensor_data.state;
			_limit_state.right_limit_healthy = !sensor_data.redundancy_fault;
		}
	}

	// Check for sensor timeouts
	check_sensor_timeouts();
}

bool SteeringLimitSensor::check_position_limits(float target_angle_rad, float max_angle_rad)
{
	if (!are_enabled()) {
		return true; // No limit checking if disabled
	}

	// Check if we're approaching a limit
	bool left_limit_approaching = (target_angle_rad < 0) &&
				       (_limit_state.left_limit_active ||
					(fabsf(target_angle_rad) > (max_angle_rad - _limit_margin_rad.get())));

	bool right_limit_approaching = (target_angle_rad > 0) &&
					(_limit_state.right_limit_active ||
					 (fabsf(target_angle_rad) > (max_angle_rad - _limit_margin_rad.get())));

	return !(left_limit_approaching || right_limit_approaching);
}

bool SteeringLimitSensor::are_sensors_healthy() const
{
	if (!are_enabled()) {
		return true; // Consider healthy if disabled
	}

	return _limit_state.left_limit_healthy && _limit_state.right_limit_healthy;
}

void SteeringLimitSensor::check_sensor_timeouts()
{
	const uint64_t now = hrt_absolute_time();
	const uint64_t sensor_timeout_us = _sensor_timeout_ms.get() * 1000;

	if ((now - _limit_state.left_last_update) > sensor_timeout_us) {
		_limit_state.left_limit_healthy = false;
	}

	if ((now - _limit_state.right_last_update) > sensor_timeout_us) {
		_limit_state.right_limit_healthy = false;
	}
}