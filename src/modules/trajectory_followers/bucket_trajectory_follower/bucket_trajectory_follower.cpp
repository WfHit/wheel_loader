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

#include "bucket_trajectory_follower.hpp"
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduleNow.hpp>

using namespace matrix;

BucketTrajectoryFollower::BucketTrajectoryFollower() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_kinematics_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": kinematics"))
{
}

BucketTrajectoryFollower::~BucketTrajectoryFollower()
{
	perf_free(_loop_perf);
	perf_free(_kinematics_perf);
}

int BucketTrajectoryFollower::task_spawn(int argc, char *argv[])
{
	BucketTrajectoryFollower *instance = new BucketTrajectoryFollower();

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

bool BucketTrajectoryFollower::init()
{
	ScheduleOnInterval(50_ms); // 20Hz control loop for smooth trajectory following
	return true;
}

void BucketTrajectoryFollower::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Update all subscriptions
	update_subscriptions();

	// Process trajectory following
	update_trajectory_following();

	perf_end(_loop_perf);
}

void BucketTrajectoryFollower::update_subscriptions()
{
	// Update trajectory setpoint
	bucket_trajectory_setpoint_s trajectory_setpoint;
	if (_bucket_trajectory_setpoint_sub.update(&trajectory_setpoint)) {
		// New trajectory received - reset any smoothing
		_trajectory_active = true;
		_last_setpoint_position = Vector3f(trajectory_setpoint.x, trajectory_setpoint.y, trajectory_setpoint.z);
		_last_update_time = hrt_absolute_time();
	}

	// Update status subscriptions for current state feedback
	boom_status_s boom_status;
	_boom_status_sub.update(&boom_status);

	bucket_status_s bucket_status;
	_bucket_status_sub.update(&bucket_status);

	vehicle_local_position_s vehicle_pos;
	_vehicle_local_position_sub.update(&vehicle_pos);
}

void BucketTrajectoryFollower::update_trajectory_following()
{
	if (!_trajectory_active) {
		return;
	}

	perf_begin(_kinematics_perf);

	// Target bucket tip position (from trajectory setpoint)
	Vector3f target_position = _last_setpoint_position;

	// Compute inverse kinematics: bucket tip position → joint angles
	float boom_angle_ground, bucket_angle_boom;
	bool solution_valid = compute_joint_angles(target_position, boom_angle_ground, bucket_angle_boom);

	if (solution_valid) {
		// Validate joint limits
		boom_angle_ground = math::constrain(boom_angle_ground, MIN_BOOM_ANGLE, MAX_BOOM_ANGLE);
		bucket_angle_boom = math::constrain(bucket_angle_boom, MIN_BUCKET_ANGLE, MAX_BUCKET_ANGLE);

		// Publish boom command (angle relative to ground)
		boom_command_s boom_cmd{};
		boom_cmd.timestamp = hrt_absolute_time();
		boom_cmd.lift_angle_cmd = boom_angle_ground;
		boom_cmd.lift_velocity_cmd = 0.0f; // Simple position control for now
		boom_cmd.control_mode = boom_command_s::CONTROL_MODE_POSITION;
		_boom_command_pub.publish(boom_cmd);

		// Publish bucket command (angle relative to boom - coordinate_frame=1)
		bucket_command_s bucket_cmd{};
		bucket_cmd.timestamp = hrt_absolute_time();
		bucket_cmd.target_angle = bucket_angle_boom;
		bucket_cmd.coordinate_frame = 1; // CRITICAL: Always boom reference frame!
		bucket_cmd.max_velocity = _param_max_bucket_velocity.get();
		bucket_cmd.control_mode = bucket_command_s::CONTROL_MODE_POSITION;
		_bucket_command_pub.publish(bucket_cmd);

	} else {
		// Invalid kinematic solution - stop trajectory following
		PX4_WARN("Invalid kinematic solution - stopping trajectory");
		_trajectory_active = false;
	}

	perf_end(_kinematics_perf);
}

bool BucketTrajectoryFollower::compute_joint_angles(const Vector3f &bucket_tip_position,
						    float &boom_angle_ground,
						    float &bucket_angle_boom)
{
	// This is where we implement the inverse kinematics using the existing
	// BoomKinematics and BucketKinematics classes
	
	// Step 1: Extract boom and bucket parameters from kinematics classes
	// (This would need to access the kinematic parameters from the existing classes)
	
	// For now, implement a simplified 2D inverse kinematics
	// Assuming boom pivot at origin, bucket attached to boom end
	
	// Boom length (would come from BoomKinematics)
	const float boom_length = 3.0f; // meters - should come from kinematics
	
	// Bucket length (would come from BucketKinematics)  
	const float bucket_length = 1.5f; // meters - should come from kinematics
	
	// 2D inverse kinematics in vertical plane (x=forward, z=up)
	float target_x = bucket_tip_position(0);
	float target_z = bucket_tip_position(2);
	
	// Distance from boom pivot to target
	float target_distance = sqrtf(target_x * target_x + target_z * target_z);
	
	// Check if target is reachable
	float max_reach = boom_length + bucket_length;
	float min_reach = fabsf(boom_length - bucket_length);
	
	if (target_distance > max_reach || target_distance < min_reach) {
		return false; // Unreachable target
	}
	
	// Law of cosines to find boom angle
	float boom_tip_to_target = bucket_length;
	float cos_boom_angle = (boom_length * boom_length + target_distance * target_distance - 
			       boom_tip_to_target * boom_tip_to_target) / 
			      (2.0f * boom_length * target_distance);
	
	if (cos_boom_angle < -1.0f || cos_boom_angle > 1.0f) {
		return false; // Invalid solution
	}
	
	float boom_to_target_angle = acosf(cos_boom_angle);
	float target_elevation_angle = atan2f(target_z, target_x);
	
	// Boom angle relative to ground (horizontal)
	boom_angle_ground = target_elevation_angle - boom_to_target_angle;
	
	// Law of cosines to find bucket angle relative to boom
	float cos_bucket_angle = (boom_length * boom_length + bucket_length * bucket_length - 
				 target_distance * target_distance) / 
				(2.0f * boom_length * bucket_length);
	
	if (cos_bucket_angle < -1.0f || cos_bucket_angle > 1.0f) {
		return false; // Invalid solution
	}
	
	// Bucket angle relative to boom (interior angle)
	bucket_angle_boom = M_PI_F - acosf(cos_bucket_angle);
	
	return true;
}

Vector3f BucketTrajectoryFollower::compute_bucket_tip_position(float boom_angle_ground, float bucket_angle_boom)
{
	// Forward kinematics for validation
	const float boom_length = 3.0f; // Should come from BoomKinematics
	const float bucket_length = 1.5f; // Should come from BucketKinematics
	
	// Boom tip position
	float boom_tip_x = boom_length * cosf(boom_angle_ground);
	float boom_tip_z = boom_length * sinf(boom_angle_ground);
	
	// Bucket angle in ground frame
	float bucket_angle_ground = boom_angle_ground + bucket_angle_boom;
	
	// Bucket tip position
	float bucket_tip_x = boom_tip_x + bucket_length * cosf(bucket_angle_ground);
	float bucket_tip_z = boom_tip_z + bucket_length * sinf(bucket_angle_ground);
	
	return Vector3f(bucket_tip_x, 0.0f, bucket_tip_z);
}

int BucketTrajectoryFollower::print_status()
{
	PX4_INFO("Bucket Trajectory Follower Status:");
	PX4_INFO("  Trajectory active: %s", _trajectory_active ? "YES" : "NO");
	PX4_INFO("  Last setpoint: [%.2f, %.2f, %.2f]", 
		 (double)_last_setpoint_position(0),
		 (double)_last_setpoint_position(1), 
		 (double)_last_setpoint_position(2));
	
	perf_print_counter(_loop_perf);
	perf_print_counter(_kinematics_perf);
	
	return 0;
}

int BucketTrajectoryFollower::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int BucketTrajectoryFollower::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Bucket trajectory follower for wheel loader. Converts bucket tip trajectory setpoints
into separate boom and bucket joint commands using inverse kinematics.

Key coordinate system understanding:
- Boom: Angle relative to ground/horizontal (BoomCommand)  
- Bucket: Angle relative to boom (BucketCommand with coordinate_frame=1)
- Bucket can ONLY rotate relative to boom physically

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("bucket_trajectory_follower", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int bucket_trajectory_follower_main(int argc, char *argv[])
{
	return BucketTrajectoryFollower::main(argc, argv);
}
