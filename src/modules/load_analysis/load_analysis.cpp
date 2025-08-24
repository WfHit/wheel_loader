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

#include "load_analysis.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <drivers/drv_hrt.h>

LoadAnalysis::LoadAnalysis() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
	_loop_perf = perf_alloc(PC_ELAPSED, MODULE_NAME": cycle");
	_load_calc_perf = perf_alloc(PC_ELAPSED, MODULE_NAME": load_calc");
}

LoadAnalysis::~LoadAnalysis()
{
	perf_free(_loop_perf);
	perf_free(_load_calc_perf);
}

bool LoadAnalysis::init()
{
	// Initialize parameters
	updateParams();

	// Start the work queue
	ScheduleOnInterval(SCHEDULE_INTERVAL);

	return true;
}

void LoadAnalysis::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Update parameters
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}

	// Process H-bridge status from all boards
	process_hbridge_status();

	// Calculate overall load
	calculate_load();

	// Publish lamp command at configured rate
	uint64_t now = hrt_absolute_time();
	uint64_t cmd_interval = 1000000 / _param_command_rate.get(); // Convert Hz to us

	if (now - _last_command_time >= cmd_interval) {
		publish_lamp_command();
		_last_command_time = now;
	}

	perf_end(_loop_perf);
}

void LoadAnalysis::process_hbridge_status()
{
	uint64_t now = hrt_absolute_time();
	hbridge_status_s status;

	// Process all hbridge status updates
	for (auto &sub : _hbridge_status_sub) {
		if (sub.update(&status)) {
			// Ensure we have a valid instance within our tracking range
			if (status.instance < MAX_CHANNELS) {
				float current_duty = fabsf(status.duty_cycle);

				// Add current duty cycle to this channel's history
				_channel_load_history[status.instance][_channel_history_index[status.instance]] = current_duty;
				_channel_history_index[status.instance] = (_channel_history_index[status.instance] + 1) % LOAD_HISTORY_SIZE;
				_channel_has_data[status.instance] = true;
				_channel_last_update[status.instance] = now;

				// Calculate average for this channel
				float channel_sum = 0.0f;
				for (int j = 0; j < LOAD_HISTORY_SIZE; j++) {
					channel_sum += _channel_load_history[status.instance][j];
				}
				_channel_average_load[status.instance] = channel_sum / LOAD_HISTORY_SIZE;
			}
		}
	}

	// Check for timed-out channels
	for (int i = 0; i < MAX_CHANNELS; i++) {
		if (_channel_has_data[i] && (now - _channel_last_update[i] > CHANNEL_TIMEOUT_US)) {
			_channel_has_data[i] = false;
			_channel_average_load[i] = 0.0f;
		}
	}
}

void LoadAnalysis::calculate_load()
{
	perf_begin(_load_calc_perf);

	// Calculate overall average from all channels with data
	float total_average_load = 0.0f;
	int active_channels = 0;

	for (int i = 0; i < MAX_CHANNELS; i++) {
		if (_channel_has_data[i]) {
			total_average_load += _channel_average_load[i];
			active_channels++;
		}
	}

	// Calculate overall average from channel averages
	if (active_channels > 0) {
		_current_overall_load = total_average_load / active_channels;
	} else {
		_current_overall_load = 0.0f;
	}

	// Apply smoothing
	_smoothed_load = apply_smoothing(_current_overall_load);

	// Map to load level
	_current_load_level = map_load_to_level(_smoothed_load);

	perf_end(_load_calc_perf);
}

float LoadAnalysis::apply_smoothing(float new_load)
{
	// Exponential moving average
	float alpha = _param_smoothing_alpha.get();
	return alpha * new_load + (1.0f - alpha) * _smoothed_load;
}

uint8_t LoadAnalysis::map_load_to_level(float load)
{
	if (load < 0.1f) {
		return load_lamp_command_s::LOAD_VERY_LOW;
	} else if (load < 0.2f) {
		return load_lamp_command_s::LOAD_LOW;
	} else if (load < _param_threshold_low.get()) {
		return load_lamp_command_s::LOAD_MED_LOW;
	} else if (load < _param_threshold_med.get()) {
		return load_lamp_command_s::LOAD_MEDIUM;
	} else if (load < _param_threshold_high.get()) {
		return load_lamp_command_s::LOAD_HIGH;
	} else {
		return load_lamp_command_s::LOAD_VERY_HIGH;
	}
}

uint32_t LoadAnalysis::get_blink_interval(uint8_t load_level)
{
	switch (load_level) {
		case load_lamp_command_s::LOAD_VERY_LOW:
			return 2000000; // 0.5 Hz
		case load_lamp_command_s::LOAD_LOW:
			return 1000000; // 1 Hz
		case load_lamp_command_s::LOAD_MED_LOW:
			return 500000;  // 2 Hz
		case load_lamp_command_s::LOAD_MEDIUM:
			return 200000;  // 5 Hz
		case load_lamp_command_s::LOAD_HIGH:
			return 100000;  // 10 Hz
		case load_lamp_command_s::LOAD_VERY_HIGH:
			return 50000;   // 20 Hz
		case load_lamp_command_s::LOAD_OFF:
		default:
			return 0;       // Off
	}
}

void LoadAnalysis::publish_lamp_command()
{
	load_lamp_command_s cmd{};
	cmd.timestamp = hrt_absolute_time();
	cmd.load_level = _current_load_level;
	cmd.load_value = _smoothed_load;
	cmd.blink_interval_us = get_blink_interval(_current_load_level);

	_load_lamp_command_pub.publish(cmd);
}

void LoadAnalysis::updateParams()
{
	ModuleParams::updateParams();
}

int LoadAnalysis::print_status()
{
	PX4_INFO("Load Analysis Module");
	PX4_INFO("  Current overall load: %.3f", (double)_current_overall_load);
	PX4_INFO("  Smoothed load: %.3f", (double)_smoothed_load);
	PX4_INFO("  Current load level: %u", _current_load_level);
	PX4_INFO("  Command rate: %d Hz", _param_command_rate.get());

	// Show individual channel averages
	PX4_INFO("  Channel load averages:");
	for (int i = 0; i < MAX_CHANNELS; i++) {
		if (_channel_has_data[i]) {
			PX4_INFO("    Channel %d: %.3f", i, (double)_channel_average_load[i]);
		}
	}

	perf_print_counter(_loop_perf);
	perf_print_counter(_load_calc_perf);
	return 0;
}

int LoadAnalysis::task_spawn(int argc, char *argv[])
{
	LoadAnalysis *instance = new LoadAnalysis();

	if (instance) {
		_object.store(instance);
		_task_id = px4_task_spawn_cmd("load_analysis",
					      SCHED_DEFAULT,
					      SCHED_PRIORITY_DEFAULT,
					      1400,
					      (px4_main_t)&run_trampoline,
					      (char *const *)argv);

		if (_task_id < 0) {
			delete instance;
			_object.store(nullptr);
			_task_id = -1;
			return PX4_ERROR;
		}

		return PX4_OK;
	}

	PX4_ERR("alloc failed");
	return PX4_ERROR;
}

int LoadAnalysis::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int LoadAnalysis::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Load analysis module that runs on the main board (X7+) to analyze motor load
from all H-bridge instances across front and rear boards via distributed uORB.

Processes hbridge_status from all boards, calculates overall load, and generates
load_lamp_command messages sent to the rear board for lamp control.

### Architecture
Front/Rear H-Bridges → uORB Proxy → Main Board Load Analysis → Load Commands → Rear Board

### Implementation
- Subscribes to hbridge_status from all H-bridge instances
- Calculates weighted average load across all active channels
- Applies smoothing to prevent rapid lamp changes
- Maps load values to discrete lamp command levels
- Publishes load_lamp_command at configurable rate

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("load_analysis", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int load_analysis_main(int argc, char *argv[])
{
	return LoadAnalysis::main(argc, argv);
}
