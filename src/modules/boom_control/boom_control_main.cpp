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

#include "boom_control.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

int BoomControl::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Boom control module for wheel loader.

Controls the boom angle using AS5600 sensor feedback and DC motor with H-bridge (DRV8701).
Includes proper kinematics transformation, S-curve motion planning, and PID position control.

### Implementation
The module uses triangle-based kinematics to convert between boom angles and actuator lengths.
The AS5600 magnetic encoder provides position feedback at the actuator pivot point.
A PID controller with S-curve trajectory generation provides smooth and precise motion.

### Examples
To set boom to carry position:
$ boom_control preset carry

To start calibration:
$ boom_control calibrate

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("boom_control", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_COMMENT("Optional parameters:");
	PRINT_MODULE_USAGE_COMMAND_DESCR("preset", "Set boom to preset position (ground|carry|max)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("calibrate", "Start automatic AS5600 calibration");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int BoomControl::custom_command(int argc, char *argv[])
{
	if (!is_running()) {
		print_usage("not running");
		return 1;
	}

	if (!strcmp(argv[0], "preset")) {
		if (argc < 2) {
			PX4_ERR("usage: boom_control preset <ground|carry|max>");
			return 1;
		}

		BoomPreset preset;
		if (!strcmp(argv[1], "ground")) {
			preset = BoomPreset::GROUND;
		} else if (!strcmp(argv[1], "carry")) {
			preset = BoomPreset::CARRY;
		} else if (!strcmp(argv[1], "max")) {
			preset = BoomPreset::MAX_HEIGHT;
		} else {
			PX4_ERR("Unknown preset: %s", argv[1]);
			return 1;
		}

		get_instance()->set_target_position(preset);
		return 0;
	}

	if (!strcmp(argv[0], "calibrate")) {
		PX4_INFO("Starting boom auto-calibration...");
		get_instance()->start_auto_calibration();
		return 0;
	}

	return print_usage("unknown command");
}

int BoomControl::task_spawn(int argc, char *argv[])
{
	BoomControl *instance = new BoomControl();

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
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

extern "C" __EXPORT int boom_control_main(int argc, char *argv[])
{
	return BoomControl::main(argc, argv);
}
