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

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/input_rc.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/wheel_loader_command.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/parameter_update.h>

#include <lib/perf/perf_counter.h>
#include <drivers/drv_hrt.h>

using namespace time_literals;

// Module name for logging and performance counters
static constexpr const char *MODULE_NAME = "wheel_loader_rc_input";

/**
 * @brief Wheel Loader RC Input Module
 *
 * Converts RC/SBUS input signals to wheel loader commands for remote operation.
 * Handles channel mapping, safety checks, and failsafe conditions.
 */
class WheelLoaderRcInput : public ModuleBase<WheelLoaderRcInput>, 
                          public ModuleParams,
                          public px4::ScheduledWorkItem
{
public:
	WheelLoaderRcInput();
	~WheelLoaderRcInput() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	void Run() override;
	bool init();

private:
	// RC channel mapping constants
	static constexpr int RC_CHANNEL_THROTTLE = 2;      // Chassis forward/backward
	static constexpr int RC_CHANNEL_STEERING = 0;      // Chassis turn angle
	static constexpr int RC_CHANNEL_BOOM = 4;          // Boom height control
	static constexpr int RC_CHANNEL_BUCKET = 5;        // Bucket angle control
	static constexpr int RC_CHANNEL_ESTOP = 6;         // Emergency stop switch
	static constexpr int RC_CHANNEL_MODE = 7;          // Control mode switch

	// Timing constants
	static constexpr float RC_UPDATE_RATE_HZ = 50.0f;
	static constexpr uint64_t RC_UPDATE_INTERVAL_US = 1_s / RC_UPDATE_RATE_HZ;
	static constexpr float RC_TIMEOUT_S = 1.0f;
	static constexpr float RC_DEADZONE = 0.05f;

	// Value limits
	static constexpr float MAX_WHEEL_SPEED = 5.0f;     // rad/s
	static constexpr float MAX_STEERING_ANGLE = 0.7f;   // rad (40 degrees)
	static constexpr float MAX_BOOM_RATE = 0.5f;        // rad/s
	static constexpr float MAX_BUCKET_RATE = 1.0f;      // rad/s

	// Core processing functions
	void processRcInput();
	void processManualControlInput();
	void publishWheelLoaderCommand();
	void updateParameters();

	// RC input processing
	float mapRcChannel(uint16_t raw_value, float min_out = -1.0f, float max_out = 1.0f);
	float applyDeadzone(float input, float deadzone = RC_DEADZONE);
	bool isValidRcInput(const input_rc_s &rc);
	bool isEmergencyStopActive(const input_rc_s &rc);

	// Safety and validation
	void applySafetyLimits(wheel_loader_command_s &cmd);
	void handleFailsafe();
	bool isRcInputValid();

	// uORB subscriptions
	uORB::Subscription _input_rc_sub{ORB_ID(input_rc)};
	uORB::Subscription _manual_control_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};

	// uORB publications
	uORB::Publication<wheel_loader_command_s> _wheel_loader_command_pub{ORB_ID(wheel_loader_command)};

	// Current state
	wheel_loader_command_s _current_command{};
	hrt_abstime _last_rc_input_time{0};
	hrt_abstime _last_manual_control_time{0};
	hrt_abstime _last_valid_input_time{0};
	bool _failsafe_active{false};
	bool _rc_input_available{false};

	// Performance counters
	perf_counter_t _cycle_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _rc_timeout_perf{perf_alloc(PC_COUNT, MODULE_NAME": rc_timeouts")};

	// Parameters for RC channel mapping and limits
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::WL_RC_MAX_SPEED>) _param_max_speed,
		(ParamFloat<px4::params::WL_RC_MAX_STEER>) _param_max_steering,
		(ParamFloat<px4::params::WL_RC_MAX_BOOM>) _param_max_boom_rate,
		(ParamFloat<px4::params::WL_RC_MAX_BUCKET>) _param_max_bucket_rate,
		(ParamFloat<px4::params::WL_RC_DEADZONE>) _param_rc_deadzone,
		(ParamFloat<px4::params::WL_RC_TIMEOUT>) _param_rc_timeout,
		(ParamInt<px4::params::WL_RC_ENABLE>) _param_rc_enable
	)
};