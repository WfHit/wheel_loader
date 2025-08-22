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

#include "manual_mode.hpp"
#include <lib/mathlib/mathlib.h>

using namespace wheel_loader;

bool ManualMode::init()
{
	// Initialize setpoints
	chassis_setpoint = {};
	bucket_setpoint = {};

	return true;
}

bool ManualMode::activate()
{
	active = true;
	return true;
}

void ManualMode::deactivate()
{
	active = false;

	// Reset setpoints to safe values
	chassis_setpoint = {};
	bucket_setpoint = {};
}

void ManualMode::update(float dt)
{
	if (!active) {
		return;
	}

	// Generate trajectories from manual inputs
	generate_chassis_trajectory();
	generate_bucket_trajectory();
}

void ManualMode::set_manual_inputs(const ManualControlInputs &inputs)
{
	manual_inputs = inputs;
}

void ManualMode::update_vehicle_state(const Vector3f &position, float yaw)
{
	current_position = position;
	current_yaw = yaw;
}

void ManualMode::generate_chassis_trajectory()
{
	const hrt_abstime now = hrt_absolute_time();

	// Generate velocity command from RC input
	float target_velocity = manual_inputs.chassis_velocity * velocity_scale;
	float target_turn_angle = manual_inputs.chassis_turn_angle * turn_angle_scale;

	// Calculate target position (simple integration for trajectory preview)
	Vector3f target_position = current_position;
	if (fabsf(target_velocity) > 0.1f) {
		// Predict position 1 second ahead
		const float preview_time = 1.0f;
		target_position(0) += target_velocity * cosf(current_yaw) * preview_time;
		target_position(1) += target_velocity * sinf(current_yaw) * preview_time;
	}

	// Calculate target yaw (assume simple steering model)
	float target_yaw = current_yaw;
	if (fabsf(target_turn_angle) > 0.1f && fabsf(target_velocity) > 0.1f) {
		// Simple bicycle model for yaw rate
		const float wheelbase = 2.5f;  // Vehicle parameter
		float yaw_rate = target_velocity * tanf(target_turn_angle) / wheelbase;
		target_yaw = current_yaw + yaw_rate * 1.0f;  // 1 second preview
	}

	// Set chassis trajectory setpoint
	chassis_setpoint.position = target_position;
	chassis_setpoint.yaw = target_yaw;
	chassis_setpoint.velocity = Vector3f(
		target_velocity * cosf(current_yaw),
		target_velocity * sinf(current_yaw),
		0.0f
	);
	chassis_setpoint.yaw_rate = fabsf(target_velocity) > 0.1f ?
		target_velocity * tanf(target_turn_angle) / 2.5f : 0.0f;
	chassis_setpoint.timestamp = now;
	chassis_setpoint.valid = true;
}

void ManualMode::generate_bucket_trajectory()
{
	const hrt_abstime now = hrt_absolute_time();

	// Generate bucket motion from RC inputs
	float boom_velocity_cmd = manual_inputs.boom_lift_velocity * boom_velocity_scale;
	float bucket_angle_cmd = manual_inputs.bucket_angle * bucket_angle_scale;

	// Calculate bucket position relative to current chassis position
	// This is a simplified kinematic model
	const float boom_length = 2.5f;   // Robot parameter
	const float boom_height = 1.2f;   // Height of boom pivot

	// Simple forward kinematics (assuming boom moves vertically)
	Vector3f bucket_position = current_position;
	bucket_position(0) += boom_length * cosf(current_yaw);  // Forward from chassis
	bucket_position(1) += boom_length * sinf(current_yaw);  // Lateral from chassis
	bucket_position(2) = boom_height;  // Height (would be controlled by boom)

	// Bucket orientation (simple model)
	Quatf bucket_orientation = Quatf(AxisAnglef(Vector3f(0, 1, 0), bucket_angle_cmd));

	// Bucket velocity
	Vector3f bucket_velocity;
	bucket_velocity(0) = 0.0f;  // No lateral motion in manual mode
	bucket_velocity(1) = 0.0f;
	bucket_velocity(2) = boom_velocity_cmd;  // Vertical motion from boom

	// Set bucket trajectory setpoint
	bucket_setpoint.position = bucket_position;
	bucket_setpoint.orientation = bucket_orientation;
	bucket_setpoint.velocity = bucket_velocity;
	bucket_setpoint.angular_velocity.zero();  // Simple model
	bucket_setpoint.timestamp = now;
	bucket_setpoint.valid = true;
}

ChassisTrajectorySetpoint ManualMode::get_chassis_setpoint() const
{
	return chassis_setpoint;
}

BucketTrajectorySetpoint ManualMode::get_bucket_setpoint() const
{
	return bucket_setpoint;
}
