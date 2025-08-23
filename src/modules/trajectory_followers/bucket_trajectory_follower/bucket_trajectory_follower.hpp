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

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/hysteresis/hysteresis.h>
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/bucket_trajectory_setpoint.h>
#include <uORB/topics/boom_status.h>
#include <uORB/topics/bucket_status.h>
#include <uORB/topics/boom_command.h>
#include <uORB/topics/bucket_command.h>
#include <lib/perf/perf_counter.h>
#include <matrix/matrix.hpp>
#include <mathlib/mathlib.h>

namespace wheel_loader
{

/**
 * @brief Bucket trajectory follower for wheel loader
 *
 * This module converts bucket tip trajectory setpoints into separate boom and bucket
 * joint commands using inverse kinematics. It uses the existing BoomKinematics and
 * BucketKinematics classes to maintain proper coordinate system separation:
 *
 * - Boom: Angle relative to ground/horizontal (BoomCommand)
 * - Bucket: Angle relative to boom (BucketCommand with coordinate_frame=1)
 *
 * Key coordinate system understanding:
 * - Bucket can ONLY rotate relative to boom physically
 * - "Ground reference" in BucketCommand is a control mode, not physical capability
 * - This module ALWAYS publishes bucket commands in boom reference frame
 */
class BucketTrajectoryFollower : public ModuleBase<BucketTrajectoryFollower>, public ModuleParams,
				  public px4::ScheduledWorkItem
{
public:
	BucketTrajectoryFollower();
	~BucketTrajectoryFollower() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	void Run() override;

	// Update subscriptions and process trajectory setpoint
	void update_subscriptions();
	void update_trajectory_following();

	// Inverse kinematics: bucket tip position → boom angle (to ground) + bucket angle (to boom)
	bool compute_joint_angles(const matrix::Vector3f &bucket_tip_position,
				  float &boom_angle_ground,
				  float &bucket_angle_boom);

	// Forward kinematics: joint angles → bucket tip position (for validation)
	matrix::Vector3f compute_bucket_tip_position(float boom_angle_ground, float bucket_angle_boom);

	// uORB subscriptions
	uORB::Subscription _bucket_trajectory_setpoint_sub{ORB_ID(bucket_trajectory_setpoint)};
	uORB::Subscription _boom_status_sub{ORB_ID(boom_status)};
	uORB::Subscription _bucket_status_sub{ORB_ID(bucket_status)};
	uORB::Subscription _vehicle_local_position_sub{ORB_ID(vehicle_local_position)};

	// uORB publications - separate commands for boom and bucket
	uORB::Publication<boom_command_s> _boom_command_pub{ORB_ID(boom_command)};
	uORB::Publication<bucket_command_s> _bucket_command_pub{ORB_ID(bucket_command)};

	// Parameters
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::BT_FOLLOW_RATE>) _param_follow_rate,
		(ParamFloat<px4::params::BT_POSITION_TOL>) _param_position_tolerance,
		(ParamFloat<px4::params::BT_MAX_BOOM_VEL>) _param_max_boom_velocity,
		(ParamFloat<px4::params::BT_MAX_BUCKET_VEL>) _param_max_bucket_velocity,
		(ParamFloat<px4::params::BT_SMOOTHING_FACTOR>) _param_smoothing_factor
	)

	// Performance counters
	perf_counter_t _loop_perf;
	perf_counter_t _kinematics_perf;

	// Control state
	bool _trajectory_active{false};
	matrix::Vector3f _last_setpoint_position{};
	hrt_abstime _last_update_time{0};

	// Safety limits and validation
	static constexpr float MIN_BOOM_ANGLE = -20.0f * M_PI_F / 180.0f; // -20 degrees
	static constexpr float MAX_BOOM_ANGLE = 60.0f * M_PI_F / 180.0f;  // 60 degrees
	static constexpr float MIN_BUCKET_ANGLE = -90.0f * M_PI_F / 180.0f; // -90 degrees (relative to boom)
	static constexpr float MAX_BUCKET_ANGLE = 90.0f * M_PI_F / 180.0f;  // 90 degrees (relative to boom)
};

} // namespace wheel_loader
