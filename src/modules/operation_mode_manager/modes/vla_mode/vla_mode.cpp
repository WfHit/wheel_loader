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

#include "vla_mode.hpp"
#include <lib/mathlib/mathlib.h>

using namespace wheel_loader;

bool VlaMode::init()
{
	// Initialize trajectory decomposer
	trajectory_decomposer.init(3.0f, 0.7f);  // 3m max reach, 70% coordination

	// Initialize setpoints
	chassis_setpoint = {};
	bucket_setpoint = {};

	return true;
}

bool VlaMode::activate()
{
	active = true;
	last_trajectory_update = hrt_absolute_time();
	return true;
}

void VlaMode::deactivate()
{
	active = false;

	// Reset setpoints to safe values
	chassis_setpoint = {};
	bucket_setpoint = {};
	vla_trajectory = {};
}

void VlaMode::update(float dt)
{
	if (!active) {
		return;
	}

	// Check trajectory timeout
	if (!is_trajectory_valid()) {
		// Invalid or old trajectory - stop motion
		chassis_setpoint.valid = false;
		bucket_setpoint.valid = false;
		return;
	}

	// Decompose VLA trajectory into chassis and bucket components
	decompose_trajectory();
}

void VlaMode::set_vla_trajectory(const VlaTrajectoryPoint &trajectory)
{
	vla_trajectory = trajectory;
	last_trajectory_update = hrt_absolute_time();
}

void VlaMode::update_vehicle_state(const Vector3f &position, float yaw)
{
	current_position = position;
	current_yaw = yaw;
}

bool VlaMode::is_trajectory_valid() const
{
	const hrt_abstime now = hrt_absolute_time();

	// Check if trajectory is valid and not too old
	return vla_trajectory.valid &&
	       ((now - last_trajectory_update) < TRAJECTORY_TIMEOUT);
}

bool VlaMode::decompose_trajectory()
{
	if (!vla_trajectory.valid) {
		return false;
	}

	// Use trajectory decomposer to split VLA command
	return trajectory_decomposer.decompose(
		vla_trajectory,
		current_position,
		current_yaw,
		chassis_setpoint,
		bucket_setpoint
	);
}

ChassisTrajectorySetpoint VlaMode::get_chassis_setpoint() const
{
	return chassis_setpoint;
}

BucketTrajectorySetpoint VlaMode::get_bucket_setpoint() const
{
	return bucket_setpoint;
}
