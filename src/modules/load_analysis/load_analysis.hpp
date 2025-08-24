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
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/perf/perf_counter.h>

// uORB includes
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionMultiArray.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/hbridge_status.h>
#include <uORB/topics/load_lamp_command.h>
#include <uORB/topics/parameter_update.h>

using namespace time_literals;

/**
 * @brief Load Analysis Module for Wheel Loader
 *
 * Runs on main board (X7+) to analyze motor load from all H-bridge instances
 * across front and rear boards and generate load lamp commands.
 */
class LoadAnalysis : public ModuleBase<LoadAnalysis>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	LoadAnalysis();
	~LoadAnalysis() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	void Run() override;
	void updateParams() override;

	// Main processing functions
	void process_hbridge_status();
	void calculate_load();
	void publish_lamp_command();

	// Load calculation helpers
	float apply_smoothing(float new_load);
	uint8_t map_load_to_level(float load);
	uint32_t get_blink_interval(uint8_t load_level);

	// uORB subscriptions
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::SubscriptionMultiArray _hbridge_status_sub{ORB_ID(hbridge_status)};

	// uORB publications
	uORB::Publication<load_lamp_command_s> _load_lamp_command_pub{ORB_ID(load_lamp_command)};

	// Performance counters
	perf_counter_t _loop_perf{nullptr};
	perf_counter_t _load_calc_perf{nullptr};

	// State variables
	static constexpr int MAX_CHANNELS = 4;  // Support up to 4 H-bridge channels
	static constexpr int LOAD_HISTORY_SIZE = 10;

	float _channel_load_history[MAX_CHANNELS][LOAD_HISTORY_SIZE]{};
	int _channel_history_index[MAX_CHANNELS]{};
	float _channel_average_load[MAX_CHANNELS]{};
	bool _channel_has_data[MAX_CHANNELS]{};
	uint64_t _channel_last_update[MAX_CHANNELS]{};

	float _current_overall_load{0.0f};
	float _smoothed_load{0.0f};
	uint8_t _current_load_level{0};
	uint64_t _last_command_time{0};

	// Configuration
	static constexpr unsigned SCHEDULE_INTERVAL = 50000;  // 50ms = 20Hz update rate
	static constexpr uint64_t CHANNEL_TIMEOUT_US = 500000; // 500ms timeout for channel data

	// Parameters
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::LOAD_ANALYSIS_ALPHA>) _param_smoothing_alpha,
		(ParamFloat<px4::params::LOAD_THRESHOLD_LOW>) _param_threshold_low,
		(ParamFloat<px4::params::LOAD_THRESHOLD_MED>) _param_threshold_med,
		(ParamFloat<px4::params::LOAD_THRESHOLD_HIGH>) _param_threshold_high,
		(ParamInt<px4::params::LOAD_CMD_RATE_HZ>) _param_command_rate
	)
};
