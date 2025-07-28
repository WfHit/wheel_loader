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

#include "wheel_loader_rc_input.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/cli.h>

extern "C" __EXPORT int wheel_loader_rc_input_main(int argc, char *argv[]);

int WheelLoaderRcInput::task_spawn(int argc, char *argv[])
{
	WheelLoaderRcInput *instance = new WheelLoaderRcInput();

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

int WheelLoaderRcInput::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int WheelLoaderRcInput::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Wheel Loader RC Input module for processing SBUS/RC commands.

This module converts RC transmitter input (SBUS protocol) into wheel loader 
control commands. It handles channel mapping for chassis movement, boom/bucket 
hydraulic controls, and safety features including emergency stop and failsafe.

### Implementation
The module subscribes to input_rc and manual_control_setpoint topics and publishes
wheel_loader_command messages with SOURCE_MANUAL_CONTROL. It provides:

- RC channel mapping to wheel loader functions
- Safety deadzone and limit processing  
- Failsafe handling for RC signal loss
- Emergency stop functionality
- Traction control integration

### Examples
Start the wheel loader RC input:
$ wheel_loader_rc_input start

Check status:
$ wheel_loader_rc_input status

Stop the module:
$ wheel_loader_rc_input stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("wheel_loader_rc_input", "system");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int wheel_loader_rc_input_main(int argc, char *argv[])
{
	return WheelLoaderRcInput::main(argc, argv);
}