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

#include "load_lamp_controller.h"
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <board_config.h>

#ifndef BOARD_HAS_LOAD_LAMP
#warning "BOARD_HAS_LOAD_LAMP not defined - load lamp controller module will be disabled"
#endif

LoadLampController::LoadLampController() : ModuleParams(nullptr)
{
}

LoadLampController::~LoadLampController()
{
	perf_free(_loop_perf);
	perf_free(_load_update_perf);
}

void LoadLampController::parameters_update()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}
}

void LoadLampController::update_load()
{
	perf_begin(_load_update_perf);

	hbridge_status_s status;

	if (_hbridge_status_sub.update(&status)) {
		float total_average_load = 0.0f;
		int active_channels = 0;

		// Update each channel's load history and calculate its average
		for (int i = 0; i < MAX_CHANNELS && i < (int)sizeof(status.channel_enabled); i++) {
			if (status.channel_enabled[i]) {
				float current_duty = fabsf(status.channel_duty_cycle[i]);

				// Add current duty cycle to this channel's history
				_channel_load_history[i][_channel_history_index[i]] = current_duty;
				_channel_history_index[i] = (_channel_history_index[i] + 1) % LOAD_HISTORY_SIZE;
				_channel_has_data[i] = true;

				// Calculate average for this channel
				float channel_sum = 0.0f;
				for (int j = 0; j < LOAD_HISTORY_SIZE; j++) {
					channel_sum += _channel_load_history[i][j];
				}
				_channel_average_load[i] = channel_sum / LOAD_HISTORY_SIZE;

				// Add this channel's average to the total
				total_average_load += _channel_average_load[i];
				active_channels++;
			} else {
				_channel_has_data[i] = false;
				_channel_average_load[i] = 0.0f;
			}
		}

		// Calculate overall average from channel averages
		if (active_channels > 0) {
			_current_load = total_average_load / active_channels;
		} else {
			_current_load = 0.0f;
		}
	}

	perf_end(_load_update_perf);
}

void LoadLampController::update_blink_rate(float load)
{
	// Map load to blink frequency based on thresholds
	if (load < _param_threshold_low.get()) {
		_blink_interval_us = 500000; // 2 Hz (slow)

	} else if (load < _param_threshold_med.get()) {
		_blink_interval_us = 200000; // 5 Hz (medium)

	} else if (load < _param_threshold_high.get()) {
		_blink_interval_us = 100000; // 10 Hz (fast)

	} else {
		_blink_interval_us = 50000;  // 20 Hz (very fast)
	}
}

void LoadLampController::toggle_lamps()
{
	_lamps_on = !_lamps_on;
	set_lamps(_lamps_on);
}

void LoadLampController::set_lamps(bool on)
{
#ifdef BOARD_HAS_LOAD_LAMP
	LOAD_LAMP_LEFT(on);
	LOAD_LAMP_RIGHT(on);
#endif
}

void LoadLampController::set_test_blink_rate(uint32_t interval_us)
{
	_blink_interval_us = interval_us;
}

void LoadLampController::run()
{
#ifdef BOARD_HAS_LOAD_LAMP
	// Initialize load lamp GPIOs
	px4_arch_configgpio(GPIO_LOAD_LAMP_LEFT);
	px4_arch_configgpio(GPIO_LOAD_LAMP_RIGHT);
	px4_arch_configgpio(GPIO_LOAD_LAMP_GND);

	// Set ground pin to 0
	px4_arch_gpiowrite(GPIO_LOAD_LAMP_GND, 0);

	// Start with lamps off
	set_lamps(false);

	PX4_INFO("Load lamp controller started:");
	PX4_INFO("  Lamp 1 GPIO (PA4): configured");
	PX4_INFO("  Lamp 2 GPIO (PC1): configured");
	PX4_INFO("  Ground GPIO (PC0): configured");
#else
	PX4_WARN("Load lamp GPIO not available on this board - module disabled");
#endif

	while (!should_exit()) {
		perf_begin(_loop_perf);

		parameters_update();

		update_load();
		update_blink_rate(_current_load);

		// Handle blinking
		uint64_t now = hrt_absolute_time();

		if (now - _last_toggle >= _blink_interval_us) {
			toggle_lamps();
			_last_toggle = now;
		}

		perf_end(_loop_perf);
		px4_usleep(20000); // 50Hz update rate
	}

#ifdef BOARD_HAS_LOAD_LAMP
	// Shutdown sequence - turn off all lamps
	set_lamps(false);
	PX4_INFO("Load lamp controller shutdown complete");
#endif
}

int LoadLampController::print_status()
{
	PX4_INFO("Load Lamp Controller");
	PX4_INFO("  Overall motor load: %.2f", (double)_current_load);
	PX4_INFO("  Blink interval: %lu us", _blink_interval_us);
	PX4_INFO("  Load thresholds: %.2f / %.2f / %.2f",
		 (double)_param_threshold_low.get(),
		 (double)_param_threshold_med.get(),
		 (double)_param_threshold_high.get());

	// Show individual channel averages
	PX4_INFO("  Channel load averages:");
	for (int i = 0; i < MAX_CHANNELS; i++) {
		if (_channel_has_data[i]) {
			PX4_INFO("    Channel %d: %.3f", i, (double)_channel_average_load[i]);
		}
	}

#ifdef BOARD_HAS_LOAD_LAMP
	PX4_INFO("  GPIO Configuration:");
	PX4_INFO("    Left Lamp (PA4): configured");
	PX4_INFO("    Right Lamp (PC1): configured");
	PX4_INFO("    Ground (PC0): configured");
#else
	PX4_INFO("  GPIO available: no (board not supported)");
#endif
	perf_print_counter(_loop_perf);
	perf_print_counter(_load_update_perf);
	return 0;
}

int LoadLampController::task_spawn(int argc, char *argv[])
{
	LoadLampController *instance = new LoadLampController();

	if (instance) {
		_object.store(instance);
		_task_id = px4_task_spawn_cmd("load_lamp_controller",
					      SCHED_DEFAULT,
					      SCHED_PRIORITY_DEFAULT - 10,
					      1200,
					      (px4_main_t)&run_trampoline,
					      (char *const *)argv);

		if (_task_id < 0) {
			PX4_ERR("task start failed");
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

int LoadLampController::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		print_usage("module not running");
		return 1;
	}

	if (!strcmp(argv[0], "test")) {
		if (argc < 2) {
			PX4_WARN("Usage: load_lamp_controller test <load_value>");
			PX4_INFO("Test with motor load value (0.0 - 1.0):");
			PX4_INFO("  0.0 - 0.3: Slow blink (2 Hz)");
			PX4_INFO("  0.3 - 0.6: Medium blink (5 Hz)");
			PX4_INFO("  0.6 - 0.8: Fast blink (10 Hz)");
			PX4_INFO("  0.8 - 1.0: Very fast blink (20 Hz)");
			PX4_INFO("Example: load_lamp_controller test 0.5");
			return 1;
		}

		float test_load = atof(argv[1]);

		if (test_load < 0.0f || test_load > 1.0f) {
			PX4_WARN("Invalid load value: %.2f. Valid range: 0.0-1.0", (double)test_load);
			return 1;
		}

		LoadLampController *instance = get_instance();

		if (instance) {
			// Calculate blink rate based on load thresholds (same logic as update_blink_rate)
			uint32_t blink_interval;
			const char* rate_description;

			if (test_load < instance->_param_threshold_low.get()) {
				blink_interval = 500000; // 2 Hz
				rate_description = "Slow blink (2 Hz)";
			} else if (test_load < instance->_param_threshold_med.get()) {
				blink_interval = 200000; // 5 Hz
				rate_description = "Medium blink (5 Hz)";
			} else if (test_load < instance->_param_threshold_high.get()) {
				blink_interval = 100000; // 10 Hz
				rate_description = "Fast blink (10 Hz)";
			} else {
				blink_interval = 50000; // 20 Hz
				rate_description = "Very fast blink (20 Hz)";
			}

			instance->set_test_blink_rate(blink_interval);
			PX4_INFO("Test load: %.2f -> %s", (double)test_load, rate_description);
			return 0;
		}

		return 1;
	}

	return print_usage("unknown command");
}

int LoadLampController::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Load lamp controller that monitors motor load and adjusts lamp blink rate accordingly.

The module subscribes to hbridge_status topic to get motor load information
and controls two lamps (PA4, PC1) with PC0 as ground.

Blink rates based on motor load:
- 0.0-0.3: Slow blink (2 Hz)
- 0.3-0.6: Medium blink (5 Hz)
- 0.6-0.8: Fast blink (10 Hz)
- 0.8-1.0: Very fast blink (20 Hz)

### Examples
To start the module:
$ load_lamp_controller start

To test with different motor load values:
$ load_lamp_controller test 0.1    # Test low load (slow blink)
$ load_lamp_controller test 0.5    # Test medium load (medium blink)
$ load_lamp_controller test 0.9    # Test high load (very fast blink)

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("load_lamp_controller", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("test <load>", "Test with motor load value (0.0-1.0)");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

int LoadLampController::run_trampoline(int argc, char *argv[])
{
	LoadLampController *instance = get_instance();

	if (instance) {
		instance->run();
	}

	return 0;
}

extern "C" __EXPORT int load_lamp_controller_main(int argc, char *argv[])
{
	return LoadLampController::main(argc, argv);
}
