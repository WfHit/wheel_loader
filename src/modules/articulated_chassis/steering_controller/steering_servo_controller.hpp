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
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/robotic_servo_command.h>
#include <uORB/topics/robotic_servo_feedback.h>
#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>

/**
 * @brief ST3125 servo controller interface
 *
 * Handles communication with ST3125 steering servo including:
 * - Position and velocity commands
 * - Feedback processing and filtering
 * - Health monitoring and fault detection
 */
class SteeringServoController : public ModuleParams
{
public:
	struct ServoState {
		float position_rad{0.0f};
		float velocity_rad_s{0.0f};
		float current_a{0.0f};
		float temperature_c{0.0f};
		uint16_t error_flags{0};
		bool torque_enabled{false};
		bool feedback_valid{false};
		uint64_t last_feedback_time{0};
	};

	SteeringServoController(ModuleParams *parent);

	/**
	 * Send position command to servo
	 * @param position_rad Target position in radians
	 * @param velocity_rad_s Target velocity in rad/s (optional)
	 */
	void send_position_command(float position_rad, float velocity_rad_s = 0.0f);

	/**
	 * Process servo feedback and update state
	 * @param dt Time delta since last update
	 */
	void update_feedback(float dt);

	/**
	 * Get current servo state
	 */
	const ServoState &get_state() const { return _servo_state; }

	/**
	 * Get filtered position (moving average)
	 */
	float get_filtered_position() const { return _filtered_position; }

	/**
	 * Check if feedback is healthy
	 */
	bool is_healthy() const;

private:
	// uORB topics
	uORB::Publication<robotic_servo_command_s> _servo_command_pub{ORB_ID(robotic_servo_command)};
	uORB::Subscription _servo_feedback_sub{ORB_ID(robotic_servo_feedback)};

	// Servo state
	ServoState _servo_state;

	// Position filtering
	static constexpr int FILTER_SIZE = 3;
	float _position_history[FILTER_SIZE]{};
	int _history_index{0};
	float _filtered_position{0.0f};

	// ST3125 parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::STEER_ST3125_ID>) _servo_id,
		(ParamFloat<px4::params::STEER_ST3125_AN>) _max_angle,
		(ParamFloat<px4::params::STEER_ST3125_VL>) _max_velocity,
		(ParamFloat<px4::params::STEER_ST3125_CR>) _current_limit,
		(ParamFloat<px4::params::STEER_ST3125_DB>) _deadband,
		(ParamFloat<px4::params::STEER_FB_TO>) _feedback_timeout_ms
	)

	void update_position_filter(float position);
};