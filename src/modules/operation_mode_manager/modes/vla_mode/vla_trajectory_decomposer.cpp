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

#include "vla_trajectory_decomposer.hpp"
#include <lib/mathlib/mathlib.h>

using namespace wheel_loader;

void VlaTrajectoryDecomposer::init(float max_reach, float coordination_factor)
{
	this->max_reach = max_reach;
	this->coordination_factor = math::constrain(coordination_factor, 0.0f, 1.0f);
}

bool VlaTrajectoryDecomposer::decompose(const VlaTrajectoryPoint &vla_point,
                                        const Vector3f &current_chassis_pos,
                                        float current_chassis_yaw,
                                        ChassisTrajectorySetpoint &chassis_trajectory,
                                        BucketTrajectorySetpoint &bucket_trajectory)
{
	// Calculate how much the chassis should contribute to the motion
	Vector3f chassis_contribution = calculate_chassis_contribution(
		vla_point.bucket_position, current_chassis_pos);

	// Generate chassis trajectory
	generate_chassis_trajectory(vla_point, current_chassis_pos, current_chassis_yaw,
	                            chassis_contribution, chassis_trajectory);

	// Generate bucket trajectory (directly from VLA command)
	generate_bucket_trajectory(vla_point, bucket_trajectory);

	// Validate the trajectories
	return validate_trajectories(chassis_trajectory, bucket_trajectory);
}

Vector3f VlaTrajectoryDecomposer::calculate_chassis_contribution(
	const Vector3f &bucket_target_world,
	const Vector3f &current_chassis_pos)
{
	// Calculate distance from chassis to bucket target
	Vector3f chassis_to_bucket = bucket_target_world - current_chassis_pos;
	chassis_to_bucket(2) = 0.0f;  // Only consider horizontal distance

	float reach_distance = chassis_to_bucket.norm();

	// If bucket target is far, chassis should move to help
	if (reach_distance > max_reach * 0.6f) {
		// Calculate how much chassis should move
		float assistance_factor = math::constrain(
			(reach_distance - max_reach * 0.5f) / (max_reach * 0.5f),
			0.0f, 1.0f
		);

		// Direction chassis should move
		Vector3f assistance_direction = chassis_to_bucket.normalized();

		// Calculate chassis contribution
		float contribution_magnitude = coordination_factor * assistance_factor *
		                               (reach_distance - max_reach * 0.5f);

		return assistance_direction * contribution_magnitude;
	}

	return Vector3f{};  // No assistance needed
}

void VlaTrajectoryDecomposer::generate_chassis_trajectory(
	const VlaTrajectoryPoint &vla_point,
	const Vector3f &current_chassis_pos,
	float current_chassis_yaw,
	const Vector3f &chassis_contribution,
	ChassisTrajectorySetpoint &chassis_trajectory)
{
	const hrt_abstime now = hrt_absolute_time();

	// Calculate target chassis position
	Vector3f target_chassis_pos = current_chassis_pos + chassis_contribution;

	// Calculate target chassis orientation to face bucket target
	Vector3f bucket_direction = vla_point.bucket_position - target_chassis_pos;
	bucket_direction(2) = 0.0f;  // Only horizontal

	float target_chassis_yaw = current_chassis_yaw;
	if (bucket_direction.norm() > 0.1f) {
		target_chassis_yaw = atan2f(bucket_direction(1), bucket_direction(0));
	}

	// Calculate chassis velocity (simple proportional to position error)
	Vector3f position_error = target_chassis_pos - current_chassis_pos;
	Vector3f chassis_velocity = position_error * 2.0f;  // P gain of 2.0

	// Limit chassis velocity
	float velocity_norm = chassis_velocity.norm();
	if (velocity_norm > max_chassis_velocity) {
		chassis_velocity = chassis_velocity * (max_chassis_velocity / velocity_norm);
	}

	// Calculate yaw rate
	float yaw_error = matrix::wrap_pi(target_chassis_yaw - current_chassis_yaw);
	float chassis_yaw_rate = yaw_error * 2.0f;  // P gain of 2.0
	chassis_yaw_rate = math::constrain(chassis_yaw_rate, -max_chassis_turn_rate, max_chassis_turn_rate);

	// Set chassis trajectory
	chassis_trajectory.position = target_chassis_pos;
	chassis_trajectory.yaw = target_chassis_yaw;
	chassis_trajectory.velocity = chassis_velocity;
	chassis_trajectory.yaw_rate = chassis_yaw_rate;
	chassis_trajectory.timestamp = now;
	chassis_trajectory.valid = true;
}

void VlaTrajectoryDecomposer::generate_bucket_trajectory(
	const VlaTrajectoryPoint &vla_point,
	BucketTrajectorySetpoint &bucket_trajectory)
{
	// Bucket trajectory is directly from VLA command (world frame)
	bucket_trajectory.position = vla_point.bucket_position;
	bucket_trajectory.orientation = vla_point.bucket_orientation;
	bucket_trajectory.velocity = vla_point.bucket_velocity;
	bucket_trajectory.angular_velocity = vla_point.bucket_angular_velocity;
	bucket_trajectory.timestamp = vla_point.timestamp;
	bucket_trajectory.valid = vla_point.valid;

	// Apply velocity limits
	float velocity_norm = bucket_trajectory.velocity.norm();
	if (velocity_norm > max_bucket_velocity) {
		bucket_trajectory.velocity = bucket_trajectory.velocity * (max_bucket_velocity / velocity_norm);
	}

	float angular_velocity_norm = bucket_trajectory.angular_velocity.norm();
	if (angular_velocity_norm > max_bucket_angular_rate) {
		bucket_trajectory.angular_velocity = bucket_trajectory.angular_velocity *
		                                     (max_bucket_angular_rate / angular_velocity_norm);
	}
}

bool VlaTrajectoryDecomposer::validate_trajectories(
	const ChassisTrajectorySetpoint &chassis_trajectory,
	const BucketTrajectorySetpoint &bucket_trajectory)
{
	// Check if bucket is within reach from chassis position
	Vector3f chassis_to_bucket = bucket_trajectory.position - chassis_trajectory.position;
	chassis_to_bucket(2) = 0.0f;  // Only horizontal distance

	float reach_distance = chassis_to_bucket.norm();
	if (reach_distance > max_reach) {
		return false;  // Bucket too far from chassis
	}

	// Check velocity limits
	if (chassis_trajectory.velocity.norm() > max_chassis_velocity) {
		return false;
	}

	if (bucket_trajectory.velocity.norm() > max_bucket_velocity) {
		return false;
	}

	// Check angular rate limits
	if (fabsf(chassis_trajectory.yaw_rate) > max_chassis_turn_rate) {
		return false;
	}

	if (bucket_trajectory.angular_velocity.norm() > max_bucket_angular_rate) {
		return false;
	}

	return true;
}
