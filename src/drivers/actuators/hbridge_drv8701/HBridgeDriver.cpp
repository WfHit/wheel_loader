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

#include "HBridgeDriver.hpp"
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

HBridgeDriver::HBridgeDriver() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default)
{
}

HBridgeDriver::~HBridgeDriver()
{
	perf_free(_loop_perf);

	delete _channel0;
	delete _channel1;
	delete _enable_manager;
}

int HBridgeDriver::init()
{
	// Load parameters
	updateParams();

	// Update configuration from parameters
	_config.pwm_frequency = _param_pwm_freq.get();
	_config.update_rate_hz = _param_update_rate.get();

	// Initialize enable manager
	_enable_manager = new EnableManager();

	if (!_enable_manager || !_enable_manager->init(_param_gpio_enable.get())) {
		PX4_ERR("Enable manager initialization failed");
		return PX4_ERROR;
	}

	// Initialize channel 0
	HBridgeChannel<0>::Config ch0_config = {
		.pwm_channel = static_cast<uint8_t>(_param_pwm_ch0.get()),
		.pwm_freq = _config.pwm_frequency,
		.gpio_dir = static_cast<uint32_t>(_param_gpio_dir0.get()),
		.gpio_fault = static_cast<uint32_t>(_param_gpio_fault0.get())
	};

	_channel0 = new HBridgeChannel<0>("hbridge_ch0");

	if (!_channel0 || !_channel0->init(ch0_config)) {
		PX4_ERR("Channel 0 initialization failed");
		return PX4_ERROR;
	}

	// Initialize channel 1
	HBridgeChannel<1>::Config ch1_config = {
		.pwm_channel = static_cast<uint8_t>(_param_pwm_ch1.get()),
		.pwm_freq = _config.pwm_frequency,
		.gpio_dir = static_cast<uint32_t>(_param_gpio_dir1.get()),
		.gpio_fault = static_cast<uint32_t>(_param_gpio_fault1.get())
	};

	_channel1 = new HBridgeChannel<1>("hbridge_ch1");

	if (!_channel1 || !_channel1->init(ch1_config)) {
		PX4_ERR("Channel 1 initialization failed");
		return PX4_ERROR;
	}

	// Start the work queue
	ScheduleOnInterval(1000000 / _config.update_rate_hz); // Convert Hz to microseconds

	_initialized = true;
	PX4_INFO("H-Bridge driver initialized successfully");
	return PX4_OK;
}

void HBridgeDriver::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Update parameters if changed
	updateParams();

	// Update channels
	if (_channel0) {
		_channel0->update();
	}

	if (_channel1) {
		_channel1->update();
	}

	// Update enable manager
	if (_enable_manager) {
		bool ch0_request = (_channel0 != nullptr);
		bool ch1_request = (_channel1 != nullptr);
		uint8_t ch0_state = _channel0 ? _channel0->get_state() : 0;
		uint8_t ch1_state = _channel1 ? _channel1->get_state() : 0;

		_enable_manager->update(ch0_request, ch1_request, ch0_state, ch1_state);
	}

	perf_end(_loop_perf);
}

int HBridgeDriver::task_spawn(int argc, char *argv[])
{
	HBridgeDriver *instance = new HBridgeDriver();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init() == PX4_OK) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

int HBridgeDriver::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		int ret = HBridgeDriver::task_spawn(argc, argv);

		if (ret) {
			return ret;
		}
	}

	return print_usage("unknown command");
}

int HBridgeDriver::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
H-Bridge DRV8701 driver for dual-channel motor control.

This driver provides independent control of two motor channels using DRV8701 H-Bridge chips.
Each channel can be controlled independently with different PWM duty cycles and directions.

The driver supports:
- Independent PWM control for each channel
- Direction control via GPIO
- Fault monitoring via GPIO polling
- Shared enable pin management
- Emergency stop functionality

### Examples
Start the driver:
$ hbridge_drv8701 start

Stop the driver:
$ hbridge_drv8701 stop

Show driver status:
$ hbridge_drv8701 status
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("hbridge_drv8701", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_COMMAND("status");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}
