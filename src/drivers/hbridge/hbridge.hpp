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

/**
 * @file hbridge.hpp
 *
 * Multi-instance H-Bridge motor driver
 */

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <px4_platform_common/px4_config.h>
#include <drivers/drv_hrt.h>
#include <drivers/drv_motor_pwm.h>
#include <lib/mathlib/mathlib.h>
#include <lib/perf/perf_counter.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/hbridge_command.h>
#include <uORB/topics/hbridge_status.h>
#include <uORB/topics/limit_sensor.h>
#include <board_config.h>
#include "hbridge_config.h"

using namespace time_literals;

// Module configuration
static constexpr char MODULE_NAME[] = "hbridge";
static constexpr uint8_t MAX_INSTANCES = 4;
static constexpr uint8_t MANAGER_INSTANCE = 255;

class HBridge : public ModuleBase<HBridge>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	HBridge(uint8_t instance);
	~HBridge() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;
	void Run() override;

	static int test(int argc, char *argv[]);

	// Multi-instance management
	static HBridge *_instances[MAX_INSTANCES];
	static px4::atomic<uint8_t> _num_instances;
	static HBridge *_manager_instance;
	static bool start_instance(int instance);
	static void stop_all_instances();

private:
	enum ChannelId : int {
		LEFT_CHANNEL = 0,
		RIGHT_CHANNEL = 1
	};

	static constexpr int MAX_CHANNELS = 2;
	static constexpr unsigned SCHEDULE_INTERVAL = 10_ms;

	struct channel_data_s {
		float current_duty_cycle{0.0f};
		bool enabled{false};
		bool initialized{false};
		bool forward_limit_active{false};
		bool reverse_limit_active{false};
	};

	// Instance data
	const uint8_t _instance;
	const hbridge_config_t *_board_config{nullptr};
	channel_data_s _channels[MAX_CHANNELS];

	// Publications and subscriptions
	uORB::PublicationMulti<hbridge_status_s> _status_pub{ORB_ID(hbridge_status)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _command_sub{ORB_ID(hbridge_command)};
	uORB::SubscriptionMultiArray<limit_sensor_s, 4> _limit_sensor_sub{ORB_ID::limit_sensor};

	// Performance counters
	perf_counter_t _loop_perf;
	perf_counter_t _command_perf;

	// State
	bool _is_running{false};
	static bool _pwm_initialized;
	hrt_abstime _last_command_time{0};
	uint32_t _command_count{0};
	uint32_t _error_count{0};

	// Methods
	void process_commands();
	void process_limit_sensors();
	void publish_status();
	bool configure_channels();
	void set_channel_speed(int channel, float duty_cycle);
	void update_channel_direction(int channel, bool forward);
	bool check_limit_sensor_for_direction(int channel, bool forward);
	int get_limit_sensor_function(int ch, bool forward) const;

	// Parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::HBRIDGE_L_FLIM>) _param_left_fwd_limit,
		(ParamInt<px4::params::HBRIDGE_L_RLIM>) _param_left_rev_limit,
		(ParamInt<px4::params::HBRIDGE_R_FLIM>) _param_right_fwd_limit,
		(ParamInt<px4::params::HBRIDGE_R_RLIM>) _param_right_rev_limit
	)
};
