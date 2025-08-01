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

#pragma once

#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/limit_sensor.h>
#include <drivers/drv_hrt.h>

/**
 * @brief Steering limit sensor monitoring
 *
 * Monitors steering limit sensors to prevent mechanical damage
 * and provides safety interlocks for steering commands.
 */
class SteeringLimitSensor : public ModuleParams
{
public:
	struct LimitSensorState {
		bool left_limit_active{false};
		bool right_limit_active{false};
		bool left_limit_healthy{false};
		bool right_limit_healthy{false};
		uint64_t left_last_update{0};
		uint64_t right_last_update{0};
		uint8_t left_instance{255};
		uint8_t right_instance{255};
	};

	SteeringLimitSensor(ModuleParams *parent);

	/**
	 * Update limit sensor state
	 */
	void update();

	/**
	 * Check if a target angle is within safe limits
	 * @param target_angle_rad Target steering angle
	 * @param max_angle_rad Maximum allowable steering angle
	 * @return true if target is safe, false if limit violation
	 */
	bool check_position_limits(float target_angle_rad, float max_angle_rad);

	/**
	 * Get current limit sensor state
	 */
	const LimitSensorState &get_state() const { return _limit_state; }

	/**
	 * Check if all limit sensors are healthy
	 */
	bool are_sensors_healthy() const;

	/**
	 * Check if limit sensors are enabled
	 */
	bool are_enabled() const { return _limit_sensors_enabled.get(); }

private:
	// uORB subscription
	uORB::Subscription _limit_sensor_sub{ORB_ID(limit_sensor)};

	// Limit sensor state
	LimitSensorState _limit_state;

	// Limit sensor parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::STEER_LIMIT_EN>) _limit_sensors_enabled,
		(ParamInt<px4::params::STEER_LT_LF_ID>) _limit_left_instance,
		(ParamInt<px4::params::STEER_LT_RT_ID>) _limit_right_instance,
		(ParamFloat<px4::params::STEER_LIMIT_MAR>) _limit_margin_rad,
		(ParamFloat<px4::params::STEER_SENS_TO>) _sensor_timeout_ms
	)

	void check_sensor_timeouts();
};