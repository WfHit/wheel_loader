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
#include <uORB/topics/vehicle_status.h>
#include <drivers/drv_hrt.h>

// Forward declarations
class SteeringServoController;
class SteeringLimitSensor;

/**
 * @brief Safety manager for steering control
 *
 * Monitors safety conditions and handles emergency situations:
 * - Emergency stop detection
 * - Servo fault monitoring
 * - Position error checking
 * - Safety violation tracking
 */
class SteeringSafetyManager : public ModuleParams
{
public:
	struct SafetyState {
		bool safety_violation{false};
		bool position_limit_violation{false};
		bool emergency_stop_active{false};
		bool servo_fault{false};
		bool sensor_fault{false};
		float safe_position_rad{0.0f};
		uint32_t violation_count{0};
		uint64_t last_violation_time{0};
	};

	SteeringSafetyManager(ModuleParams *parent);

	/**
	 * Update safety state
	 * @param servo_controller Reference to servo controller
	 * @param limit_sensor Reference to limit sensor
	 * @param commanded_angle Current commanded angle
	 * @param current_angle Current actual angle
	 */
	void update_safety_state(const SteeringServoController &servo_controller,
				  const SteeringLimitSensor &limit_sensor,
				  float commanded_angle, float current_angle);

	/**
	 * Check if system is in safe state
	 */
	bool is_safe() const;

	/**
	 * Handle safety violation (move to safe position)
	 * @param servo_controller Reference to servo controller for emergency command
	 */
	void handle_safety_violation(SteeringServoController &servo_controller);

	/**
	 * Get current safety state
	 */
	const SafetyState &get_state() const { return _safety_state; }

	/**
	 * Check if safety manager is enabled
	 */
	bool is_enabled() const { return _safety_manager_enabled.get(); }

private:
	// uORB subscription
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};

	// Safety state
	SafetyState _safety_state;

	// Emergency stop state
	bool _emergency_stop{false};

	// Safety parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::STEER_SAFETY_EN>) _safety_manager_enabled,
		(ParamFloat<px4::params::STEER_SAFE_POS>) _safety_position_rad,
		(ParamFloat<px4::params::STEER_MAX_POS>) _max_position_error,
		(ParamFloat<px4::params::STEER_FT_TO>) _fault_timeout_ms,
		(ParamInt<px4::params::STEER_MAX_VIOL>) _max_violations
	)

	void check_emergency_stop();
	void check_servo_faults(const SteeringServoController &servo_controller);
	void check_sensor_faults(const SteeringLimitSensor &limit_sensor);
	void check_position_error(float commanded_angle, float current_angle);
	void clear_safety_violations_if_timeout();
};