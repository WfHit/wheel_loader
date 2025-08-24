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

#pragma once

#include <drivers/drv_hrt.h>
#include <matrix/matrix/math.hpp>
#include <lib/mathlib/mathlib.h>

using namespace matrix;

namespace wheel_loader
{

/**
 * Chassis trajectory setpoint in WORLD coordinates
 * Published by operation modes, subscribed by chassis trajectory follower
 */
struct ChassisTrajectorySetpoint {
	Vector3f position;           // Position in world frame (m)
	float yaw;                   // Yaw angle in world frame (rad)
	Vector3f velocity;           // Linear velocity in world frame (m/s)
	float yaw_rate;              // Yaw rate (rad/s)
	hrt_abstime timestamp;       // Timestamp
	bool valid{false};           // Validity flag
};

/**
 * End effector trajectory setpoint in WORLD coordinates
 * Published by operation modes, subscribed by end effector trajectory follower
 */
struct EndEffectorTrajectorySetpoint {
	Vector3f position;           // End effector position in world frame (m)
	Quatf orientation;           // End effector orientation in world frame
	Vector3f velocity;           // End effector linear velocity in world frame (m/s)
	Vector3f angular_velocity;   // End effector angular velocity in world frame (rad/s)
	hrt_abstime timestamp;       // Timestamp
	bool valid{false};           // Validity flag
};

/**
 * VLA trajectory point - Complete 6DOF end effector pose in WORLD frame
 * This is the input from the Vision-Language-Action model
 */
struct VlaTrajectoryPoint {
	Vector3f end_effector_position;    // End effector position in world frame (m)
	Quatf end_effector_orientation;    // End effector orientation in world frame
	Vector3f end_effector_velocity;    // End effector linear velocity in world frame (m/s)
	Vector3f end_effector_angular_velocity; // End effector angular velocity in world frame (rad/s)
	hrt_abstime timestamp;       // Timestamp
	bool valid{false};           // Validity flag
};

/**
 * Manual control inputs from RC
 */
struct ManualControlInputs {
	float chassis_velocity;      // Forward/backward velocity command (-1 to 1)
	float chassis_turn_angle;    // Turn angle command (-1 to 1)
	float boom_lift_velocity;    // Boom lift velocity command (-1 to 1)
	float end_effector_angle;    // End effector angle command (-1 to 1)
	bool mode_switch;            // Mode switch state
	hrt_abstime timestamp;       // Timestamp
};

/**
 * Chassis control command
 * Published by chassis trajectory follower to wheel/steering modules
 */
struct ChassisControlCommand {
	float velocity;              // Forward/backward velocity (m/s)
	float steering_angle;        // Steering angle (rad)
	hrt_abstime timestamp;       // Timestamp
	bool valid{false};           // Validity flag
};

/**
 * End effector control command
 * Published by end effector trajectory follower to boom/end effector control modules
 */
struct EndEffectorControlCommand {
	Vector3f position;           // Target end effector position (m)
	Quatf orientation;           // Target end effector orientation
	Vector3f velocity;           // Target end effector velocity (m/s)
	Vector3f angular_velocity;   // Target end effector angular velocity (rad/s)
	hrt_abstime timestamp;       // Timestamp
	bool valid{false};           // Validity flag
};

} // namespace wheel_loader
