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

#include "driver_lamp_controller.h"
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <board_config.h>

#ifndef BOARD_HAS_DRIVER_LAMP
#warning "BOARD_HAS_DRIVER_LAMP not defined - driver lamp controller module will be disabled"
#endif

DriverLampController::DriverLampController() : ModuleParams(nullptr)
{
}

DriverLampController::~DriverLampController()
{
	perf_free(_loop_perf);
	perf_free(_mode_update_perf);
}

void DriverLampController::parameters_update()
{
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		updateParams();
	}
}

void DriverLampController::update_lamp_mode()
{
	perf_begin(_mode_update_perf);

	// Skip normal mode logic if in test mode
	if (_test_mode_active) {
		_current_mode = _test_mode;
		perf_end(_mode_update_perf);
		return;
	}

	// Normal operation - check vehicle status
	vehicle_status_s status;

	if (_vehicle_status_sub.update(&status)) {
		// Check for reverse gear (simplified - you may need different logic)
		if (status.nav_state == vehicle_status_s::NAVIGATION_STATE_AUTO_RTL) {
			_current_mode = LampMode::REVERSE;
			perf_end(_mode_update_perf);
			return;
		}
	}

	// Check control mode for turn signals
	vehicle_control_mode_s control_mode;

	if (_vehicle_control_mode_sub.update(&control_mode)) {
		// This is simplified - you'd need actual turn signal logic
		// For now, using manual control flags as example
		if (control_mode.flag_control_manual_enabled) {
			// TODO: Add actual turn signal logic here
			// _current_mode = LampMode::LEFT_TURN; // Example
		}
	}

	// Default to OFF if no specific mode detected
	_current_mode = LampMode::OFF;

	perf_end(_mode_update_perf);
}

uint32_t DriverLampController::get_turn_signal_interval_us() const
{
	// Convert Hz to microseconds
	float blink_rate = _param_blink_rate.get();

	if (blink_rate <= 0.0f) {
		blink_rate = 1.5f; // Default fallback
	}

	return static_cast<uint32_t>(1000000.0f / (2.0f * blink_rate)); // Half period for on/off
}

void DriverLampController::process_lamp_state()
{
	uint64_t now = hrt_absolute_time();

	switch (_current_mode) {
	case LampMode::OFF:
		set_lamps(false, false);
		break;

	case LampMode::REVERSE:
		set_lamps(true, true);
		break;

	case LampMode::LEFT_TURN: {
		uint32_t interval = get_turn_signal_interval_us();

		if (now - _last_toggle >= interval) {
			_left_lamp_on = !_left_lamp_on;
			set_lamps(_left_lamp_on, false);
			_last_toggle = now;
		}
	}
	break;

	case LampMode::RIGHT_TURN: {
		uint32_t interval = get_turn_signal_interval_us();

		if (now - _last_toggle >= interval) {
			_right_lamp_on = !_right_lamp_on;
			set_lamps(false, _right_lamp_on);
			_last_toggle = now;
		}
	}
	break;

	case LampMode::HAZARD:
		process_hazard_pattern();
		break;
	}
}

void DriverLampController::process_hazard_pattern()
{
	uint64_t now = hrt_absolute_time();
	uint64_t elapsed = now - _hazard_timer;

	switch (_hazard_state) {
	case HazardState::FIRST_SHORT:
		set_lamps(true, true);

		if (elapsed >= HAZARD_SHORT_ON_US) {
			_hazard_state = HazardState::FIRST_SHORT_OFF;
			_hazard_timer = now;
		}

		break;

	case HazardState::FIRST_SHORT_OFF:
		set_lamps(false, false);

		if (elapsed >= HAZARD_SHORT_OFF_US) {
			_hazard_state = HazardState::SECOND_SHORT;
			_hazard_timer = now;
		}

		break;

	case HazardState::SECOND_SHORT:
		set_lamps(true, true);

		if (elapsed >= HAZARD_SHORT_ON_US) {
			_hazard_state = HazardState::SECOND_SHORT_OFF;
			_hazard_timer = now;
		}

		break;

	case HazardState::SECOND_SHORT_OFF:
		set_lamps(false, false);

		if (elapsed >= HAZARD_SHORT_OFF_US) {
			_hazard_state = HazardState::LONG;
			_hazard_timer = now;
		}

		break;

	case HazardState::LONG:
		set_lamps(true, true);

		if (elapsed >= HAZARD_LONG_ON_US) {
			_hazard_state = HazardState::LONG_OFF;
			_hazard_timer = now;
		}

		break;

	case HazardState::LONG_OFF:
		set_lamps(false, false);

		if (elapsed >= HAZARD_LONG_OFF_US) {
			_hazard_state = HazardState::FIRST_SHORT;
			_hazard_timer = now;
		}

		break;
	}
}

void DriverLampController::set_lamps(bool left, bool right)
{
#ifdef BOARD_HAS_DRIVER_LAMP
	DRIVER_LAMP_LEFT(left);
	DRIVER_LAMP_RIGHT(right);
#endif
}

void DriverLampController::run()
{
#ifdef BOARD_HAS_DRIVER_LAMP
	// Initialize driver lamp GPIOs
	px4_arch_configgpio(GPIO_DRIVER_LAMP_LEFT);
	px4_arch_configgpio(GPIO_DRIVER_LAMP_RIGHT);
	px4_arch_configgpio(GPIO_DRIVER_LAMP_GND);

	// Set ground pin to 0
	px4_arch_gpiowrite(GPIO_DRIVER_LAMP_GND, 0);

	// Start with lamps off
	set_lamps(false, false);

	PX4_INFO("Driver lamp controller started:");
	PX4_INFO("  Left lamp GPIO (PB2): configured");
	PX4_INFO("  Right lamp GPIO (PB4): configured");
	PX4_INFO("  Ground GPIO (PB3): configured");
#else
	PX4_WARN("Driver lamp GPIO not available on this board - module disabled");
#endif

	// Initialize hazard timer
	_hazard_timer = hrt_absolute_time();

	while (!should_exit()) {
		perf_begin(_loop_perf);

		parameters_update();

		update_lamp_mode();
		process_lamp_state();

		perf_end(_loop_perf);
		px4_usleep(20000); // 50Hz update rate
	}

#ifdef BOARD_HAS_DRIVER_LAMP
	// Shutdown sequence - turn off all lamps
	set_lamps(false, false);
	PX4_INFO("Driver lamp controller shutdown complete");
#endif
}

int DriverLampController::print_status()
{
	PX4_INFO("Driver Lamp Controller");
	PX4_INFO("  Current mode: %d", static_cast<int>(_current_mode));
	PX4_INFO("  Test mode: %s", _test_mode_active ? "active" : "inactive");

	if (_test_mode_active) {
		PX4_INFO("  Test mode type: %d", static_cast<int>(_test_mode));
	}

	PX4_INFO("  Blink rate: %.1f Hz", (double)_param_blink_rate.get());
	PX4_INFO("  Left lamp: %s", _left_lamp_on ? "on" : "off");
	PX4_INFO("  Right lamp: %s", _right_lamp_on ? "on" : "off");
#ifdef BOARD_HAS_DRIVER_LAMP
	PX4_INFO("  GPIO Configuration:");
	PX4_INFO("    Left lamp (PB2): configured");
	PX4_INFO("    Right lamp (PB4): configured");
	PX4_INFO("    Ground (PB3): configured");
#else
	PX4_INFO("  GPIO available: no (board not supported)");
#endif
	perf_print_counter(_loop_perf);
	perf_print_counter(_mode_update_perf);
	return 0;
}

int DriverLampController::task_spawn(int argc, char *argv[])
{
	DriverLampController *instance = new DriverLampController();

	if (instance) {
		_object.store(instance);
		_task_id = px4_task_spawn_cmd("driver_lamp_controller",
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

int DriverLampController::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		PX4_ERR("not running");
		return PX4_ERROR;
	}

	DriverLampController *instance = get_instance();

	if (!instance) {
		PX4_ERR("instance not found");
		return PX4_ERROR;
	}

	if (argc >= 2 && strcmp(argv[0], "test") == 0) {
		if (argc < 2) {
			PX4_INFO("Usage: driver_lamp_controller test <mode>");
			PX4_INFO("Available modes: off, left, right, reverse, hazard");
			return PX4_OK;
		}

		const char *mode = argv[1];

		if (strcmp(mode, "off") == 0) {
			instance->test_mode(LampMode::OFF);
			PX4_INFO("Testing: OFF mode");
		} else if (strcmp(mode, "left") == 0) {
			instance->test_mode(LampMode::LEFT_TURN);
			PX4_INFO("Testing: LEFT TURN mode");
		} else if (strcmp(mode, "right") == 0) {
			instance->test_mode(LampMode::RIGHT_TURN);
			PX4_INFO("Testing: RIGHT TURN mode");
		} else if (strcmp(mode, "reverse") == 0) {
			instance->test_mode(LampMode::REVERSE);
			PX4_INFO("Testing: REVERSE mode");
		} else if (strcmp(mode, "hazard") == 0) {
			instance->test_mode(LampMode::HAZARD);
			PX4_INFO("Testing: HAZARD mode");
		} else if (strcmp(mode, "normal") == 0) {
			instance->test_mode_disable();
			PX4_INFO("Returning to normal operation");
		} else {
			PX4_ERR("Unknown test mode: %s", mode);
			PX4_INFO("Available modes: off, left, right, reverse, hazard, normal");
			return PX4_ERROR;
		}

		return PX4_OK;
	}

	return print_usage("unknown command");
}

int DriverLampController::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver lamp controller that manages turn signals and reverse lamps.

The module subscribes to vehicle control and status topics to control
two lamps (PB2, PB4) with PB3 as ground.

Lamp modes:
- OFF: Both lamps off
- REVERSE: Both lamps solid on
- LEFT_TURN: Left lamp blinks at configured rate
- RIGHT_TURN: Right lamp blinks at configured rate
- HAZARD: Both lamps blink in short-short-long pattern

Hazard pattern: 100ms on, 100ms off, 100ms on, 100ms off, 300ms on, 400ms off, repeat

### Examples
To start the module:
$ driver_lamp_controller start

To test different lamp modes:
$ driver_lamp_controller test off      # Test OFF mode
$ driver_lamp_controller test left     # Test LEFT TURN mode
$ driver_lamp_controller test right    # Test RIGHT TURN mode
$ driver_lamp_controller test reverse  # Test REVERSE mode
$ driver_lamp_controller test hazard   # Test HAZARD mode
$ driver_lamp_controller test normal   # Return to normal operation

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("driver_lamp_controller", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND_DESCR("test", "Test lamp modes");
	PRINT_MODULE_USAGE_ARG("off|left|right|reverse|hazard|normal", "Test mode", false);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

void DriverLampController::test_mode(LampMode mode)
{
	_test_mode_active = true;
	_test_mode = mode;
	_current_mode = mode;

	// Reset timing variables for the new mode
	_last_toggle = hrt_absolute_time();
	_hazard_timer = hrt_absolute_time();
	_hazard_state = HazardState::FIRST_SHORT;
	_left_lamp_on = false;
	_right_lamp_on = false;
}

void DriverLampController::test_mode_disable()
{
	_test_mode_active = false;
	_current_mode = LampMode::OFF;
	set_lamps(false, false);
}

int DriverLampController::run_trampoline(int argc, char *argv[])
{
	DriverLampController *instance = get_instance();

	if (instance) {
		instance->run();
	}

	return 0;
}

extern "C" __EXPORT int driver_lamp_controller_main(int argc, char *argv[])
{
	return DriverLampController::main(argc, argv);
}
