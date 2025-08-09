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

#pragma once

#include <px4_platform_common/module_params.h>
#include <drivers/drv_hrt.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/hbridge_command.h>
#include <uORB/topics/hbridge_status.h>

/**
 * @brief Actuator interface for boom motor control
 *
 * Manages H-bridge motor driver interface:
 * - Command publishing
 * - Status monitoring
 * - Fault detection
 * - Current limiting
 */
class BoomActuatorInterface : public ModuleParams
{
public:
	static constexpr hrt_abstime STATUS_TIMEOUT_US = 100000; // 100ms
	static constexpr uint32_t MAX_FAULT_COUNT = 5;

	struct ActuatorCommand {
		float duty_cycle;      // -1 to 1
		bool enable;          // Enable motor
		uint8_t mode;         // Control mode
	};

	struct ActuatorStatus {
		bool enabled;         // Motor enabled
		bool fault;          // Fault detected
		float current;       // Motor current (A)
		float voltage;       // Supply voltage (V)
		float temperature;   // Temperature (°C)
		hrt_abstime timestamp;
	};

	explicit BoomActuatorInterface(ModuleParams *parent);

	/**
	 * @brief Initialize actuator interface
	 * @param motor_instance H-bridge instance to use
	 * @return True if initialization successful
	 */
	bool initialize(int motor_instance);

	/**
	 * @brief Send actuator command
	 * @param command Command to send
	 * @return True if command sent successfully
	 */
	bool send_command(const ActuatorCommand &command);

	/**
	 * @brief Update actuator status
	 * @param status Output status structure
	 * @return True if new status available
	 */
	bool update_status(ActuatorStatus &status);

	/**
	 * @brief Emergency stop - disable motor immediately
	 */
	void emergency_stop();

	/**
	 * @brief Check if actuator is healthy
	 * @return True if actuator is operational
	 */
	bool is_healthy() const;

	/**
	 * @brief Get last command sent
	 * @return Last actuator command
	 */
	ActuatorCommand get_last_command() const { return _last_command; }

private:
	// Configuration
	int _motor_instance{-1};
	int _hbridge_selected{-1};
	bool _initialized{false};

	// State tracking
	ActuatorCommand _last_command{};
	ActuatorStatus _last_status{};
	hrt_abstime _last_command_time{0};
	hrt_abstime _last_status_time{0};

	// Health monitoring
	bool _is_healthy{false};
	uint32_t _fault_count{0};

	// uORB interface
	uORB::SubscriptionMultiArray<hbridge_status_s, 4> _hbridge_status_sub{ORB_ID::hbridge_status};
	orb_advert_t _hbridge_command_pub{nullptr};

	// Actuator parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::BOOM_MOTOR_IDX>) _param_motor_index,
		(ParamFloat<px4::params::BOOM_CUR_LIM>) _param_current_limit,
		(ParamFloat<px4::params::BOOM_DUTY_MAX>) _param_duty_max
	)

	/**
	 * @brief Select best H-bridge instance
	 * @return Selected instance index or -1 if none found
	 */
	int select_hbridge_instance();

	/**
	 * @brief Apply current limiting
	 * @param command Command to limit
	 * @param current Current motor current
	 * @return Limited command
	 */
	ActuatorCommand apply_current_limit(const ActuatorCommand &command, float current) const;
};
