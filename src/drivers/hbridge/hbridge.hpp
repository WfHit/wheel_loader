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

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/drv_motor_pwm.h>
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionCallback.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/hbridge_command.h>
#include <uORB/topics/hbridge_status.h>
#include <uORB/topics/limit_sensor.h>

using namespace time_literals;

/**
 * @brief H-Bridge motor driver with dual channels (left and right)
 *
 * Controls 2 H-bridge channels with PWM speed control and GPIO direction control.
 * Designed for DRV8701 H-bridge controllers.
 *
 * Channel mapping:
 * - Channel 0 = Left channel
 * - Channel 1 = Right channel
 */
class HBridge : public ModuleBase<HBridge>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	HBridge();
	~HBridge() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	/** @see ModuleBase::run_task() */
	void Run() override;

	bool init();

	static int test(int argc, char *argv[]);

private:
	// Channel definitions
	enum ChannelId : int {
		LEFT_CHANNEL = 0,
		RIGHT_CHANNEL = 1
	};

	// Maximum number of channels per H-bridge
	static constexpr int MAX_CHANNELS = 2;

	// Update rate
	static constexpr unsigned SCHEDULE_INTERVAL = 10_ms;

	struct channel_data_s {
		uint32_t dir_gpio{0};          // Direction control GPIO
		int pwm_channel{-1};           // PWM channel number
		uint32_t pwm_mask{0};          // PWM channel mask
		float current_duty_cycle{0.0f}; // Current duty cycle
		bool enabled{false};            // Channel enabled
		bool initialized{false};        // Channel initialized
		bool forward_limit_active{false}; // Forward direction limit sensor active
		bool reverse_limit_active{false}; // Reverse direction limit sensor active
		bool dir_reversed{false};      // Direction signal is reversed
	};

	// Methods
	void parameters_update();
	void process_commands();
	void process_limit_sensors();
	void publish_status();
	bool configure_channel(int channel);
	void set_channel_speed(int channel, float duty_cycle);
	void update_channel_direction(int channel, bool forward);
	bool check_limit_sensor_for_direction(int channel, bool forward);

	// Parameter getters
	int get_pwm_channel(int ch) const;
	int get_limit_sensor_function(int ch, bool forward) const;

	// Convenience methods for left/right channel access
	void set_left_channel_speed(float duty_cycle) { set_channel_speed(LEFT_CHANNEL, duty_cycle); }
	void set_right_channel_speed(float duty_cycle) { set_channel_speed(RIGHT_CHANNEL, duty_cycle); }
	int get_left_pwm_channel() const { return get_pwm_channel(LEFT_CHANNEL); }
	int get_right_pwm_channel() const { return get_pwm_channel(RIGHT_CHANNEL); }

	// Channel data
	channel_data_s _channels[MAX_CHANNELS];

	// Publications
	orb_advert_t _status_pub{nullptr};

	// Subscriptions
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _command_sub{ORB_ID(hbridge_command)};
	uORB::Subscription _limit_sensor_sub{ORB_ID(limit_sensor)};

	// Performance counters
	perf_counter_t _loop_perf;
	perf_counter_t _command_perf;

	// State
	bool _is_running{false};
	static bool _pwm_initialized;
	hrt_abstime _last_command_time{0};
	uint32_t _command_count{0};
	uint32_t _error_count{0};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::HBRIDGE_L_PWM>) _param_left_pwm,
		(ParamInt<px4::params::HBRIDGE_R_PWM>) _param_right_pwm,
		(ParamFloat<px4::params::HBRIDGE_PWM_FREQ>) _param_pwm_freq,
		(ParamInt<px4::params::HBRIDGE_L_DREV>) _param_left_dir_rev,
		(ParamInt<px4::params::HBRIDGE_R_DREV>) _param_right_dir_rev,
		(ParamInt<px4::params::HBRIDGE_L_FLIM>) _param_left_fwd_limit,
		(ParamInt<px4::params::HBRIDGE_L_RLIM>) _param_left_rev_limit,
		(ParamInt<px4::params::HBRIDGE_R_FLIM>) _param_right_fwd_limit,
		(ParamInt<px4::params::HBRIDGE_R_RLIM>) _param_right_rev_limit
	)
};
