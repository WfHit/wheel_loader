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

#include "chassis_trajectory_follower.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>

using namespace wheel_loader;

extern "C" __EXPORT int chassis_trajectory_follower_main(int argc, char *argv[]);

/**
 * Chassis Trajectory Follower implementation
 */
int ChassisTrajectoryFollower::task_spawn(int argc, char *argv[])
{
	ChassisTrajectoryFollower *instance = new ChassisTrajectoryFollower();

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

ChassisTrajectoryFollower *ChassisTrajectoryFollower::instantiate(int argc, char *argv[])
{
	return new ChassisTrajectoryFollower();
}

int ChassisTrajectoryFollower::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int ChassisTrajectoryFollower::print_usage(const char *reason)
{
	if (reason) {
		PX4_ERR("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Chassis trajectory follower with MPC control and coordination.
Runs at 50Hz for smooth motion control with predictive capabilities.

### Implementation
Uses TinyMPC implementation for chassis trajectory following.
State vector: [x, y, theta, v] (position, heading, velocity)
Control vector: [a, delta] (acceleration, steering angle)
Uses bicycle model for prediction with embedded MPC solver.

### Examples
Start the chassis trajectory follower:
$ chassis_trajectory_follower start

Stop the chassis trajectory follower:
$ chassis_trajectory_follower stop

Check status:
$ chassis_trajectory_follower status
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("chassis_trajectory_follower", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

/**
 * Main chassis trajectory follower module entry point
 */
int chassis_trajectory_follower_main(int argc, char *argv[])
{
	return ChassisTrajectoryFollower::main(argc, argv);
}
