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

#include "end_effector_trajectory_follower.hpp"
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduleNow.hpp>

using namespace matrix;

EndEffectorTrajectoryFollower::EndEffectorTrajectoryFollower() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
	_kinematics_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": kinematics"))
{
}

EndEffectorTrajectoryFollower::~EndEffectorTrajectoryFollower()
{
	perf_free(_loop_perf);
	perf_free(_kinematics_perf);
}

int EndEffectorTrajectoryFollower::task_spawn(int argc, char *argv[])
{
	EndEffectorTrajectoryFollower *instance = new EndEffectorTrajectoryFollower();

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

bool EndEffectorTrajectoryFollower::init()
{
	ScheduleOnInterval(50_ms); // 20Hz control loop for smooth trajectory following
	return true;
}

void EndEffectorTrajectoryFollower::Run()
{
	if (should_exit()) {
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);

	// Update all subscriptions
	update_subscriptions();

	// Generate trajectories for boom and bucket controllers
	update_trajectory_generation();

	perf_end(_loop_perf);
}

void EndEffectorTrajectoryFollower::update_subscriptions()
{
	// Update end effector trajectory setpoint
	end_effector_trajectory_setpoint_s ee_setpoint;
	if (_end_effector_trajectory_setpoint_sub.update(&ee_setpoint)) {
		// New end effector trajectory received (bucket position from system perspective)
		_trajectory_active = true;
		_last_end_effector_position = Vector3f(ee_setpoint.x, ee_setpoint.y, ee_setpoint.z);
		_last_end_effector_orientation = Quatf(ee_setpoint.q[0], ee_setpoint.q[1], ee_setpoint.q[2], ee_setpoint.q[3]);
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

void EndEffectorTrajectoryFollower::update_trajectory_generation()
{
	if (!_trajectory_active) {
		return;
	}

	perf_begin(_kinematics_perf);

	// Target end effector position and orientation (bucket tip from system perspective)
	Vector3f target_position = _last_end_effector_position;
	Quatf target_orientation = _last_end_effector_orientation;

	// Compute inverse kinematics: end effector pose → boom + bucket joint angles
	// End effector = bucket viewed from system perspective (global position/orientation)
	// Output = boom + bucket angles for local joint control
	float boom_angle_chassis, bucket_angle_boom;
	bool solution_valid = compute_joint_angles(target_position, target_orientation,
						   boom_angle_chassis, bucket_angle_boom);

	if (solution_valid) {
		// Apply safety limits
		boom_angle_chassis = math::constrain(boom_angle_chassis, MIN_BOOM_ANGLE, MAX_BOOM_ANGLE);
		bucket_angle_boom = math::constrain(bucket_angle_boom, MIN_BUCKET_ANGLE, MAX_BUCKET_ANGLE);

		// Compute velocities for smooth motion
		float dt = (hrt_absolute_time() - _last_update_time) * 1e-6f; // Convert to seconds
		float boom_velocity = 0.0f;
		float bucket_velocity = 0.0f;

		if (dt > 0.001f && dt < 0.1f) { // Valid time step
			boom_velocity = (boom_angle_chassis - _last_boom_angle) / dt;
			bucket_velocity = (bucket_angle_boom - _last_bucket_angle) / dt;

			// Limit velocities
			boom_velocity = math::constrain(boom_velocity, -_param_max_boom_velocity.get(), _param_max_boom_velocity.get());
			bucket_velocity = math::constrain(bucket_velocity, -_param_max_bucket_velocity.get(), _param_max_bucket_velocity.get());
		}

		// Generate trajectory setpoints for controllers
		generate_boom_trajectory(boom_angle_chassis, boom_velocity);
		generate_bucket_trajectory(bucket_angle_boom, bucket_velocity);

		// Update state for next iteration
		_last_boom_angle = boom_angle_chassis;
		_last_bucket_angle = bucket_angle_boom;

	} else {
		// Invalid kinematic solution - stop trajectory generation
		PX4_WARN("Invalid kinematic solution - stopping trajectory generation");
		_trajectory_active = false;
	}

	perf_end(_kinematics_perf);
}

bool EndEffectorTrajectoryFollower::compute_joint_angles(const Vector3f &end_effector_position_chassis,
						    const Quatf &end_effector_orientation_chassis,
						    float &boom_angle_chassis,
						    float &bucket_angle_boom)
{
	// This implements inverse kinematics for the boom-bucket kinematic chain
	// Key concept: End effector = bucket (same physical object, different control perspectives)
	// Input: End effector position/orientation in chassis frame (bucket from system perspective)
	// Output: Boom angle (chassis→boom) + Bucket angle (boom→bucket) for joint controllers

	// For simplified 2D kinematics in the vertical plane (x=forward, z=up from chassis)
	float target_x = end_effector_position_chassis(0);
	float target_z = end_effector_position_chassis(2) - BOOM_PIVOT_HEIGHT; // Relative to boom pivot

	// Distance from boom pivot to target
	float target_distance = sqrtf(target_x * target_x + target_z * target_z);

	// Check if target is reachable
	float max_reach = BOOM_LENGTH + BUCKET_LENGTH;
	float min_reach = fabsf(BOOM_LENGTH - BUCKET_LENGTH);

	if (target_distance > max_reach || target_distance < min_reach) {
		PX4_DEBUG("Target unreachable: distance=%.2f, min=%.2f, max=%.2f",
			  (double)target_distance, (double)min_reach, (double)max_reach);
		return false; // Unreachable target
	}

	// Law of cosines to find boom angle
	float cos_boom_angle = (BOOM_LENGTH * BOOM_LENGTH + target_distance * target_distance -
			       BUCKET_LENGTH * BUCKET_LENGTH) /
			      (2.0f * BOOM_LENGTH * target_distance);

	if (cos_boom_angle < -1.0f || cos_boom_angle > 1.0f) {
		return false; // Invalid solution
	}

	float boom_to_target_angle = acosf(cos_boom_angle);
	float target_elevation_angle = atan2f(target_z, target_x);

	// Boom angle relative to chassis horizontal
	boom_angle_chassis = target_elevation_angle - boom_to_target_angle;

	// Law of cosines to find bucket angle relative to boom
	float cos_bucket_angle = (BOOM_LENGTH * BOOM_LENGTH + BUCKET_LENGTH * BUCKET_LENGTH -
				 target_distance * target_distance) /
				(2.0f * BOOM_LENGTH * BUCKET_LENGTH);

	if (cos_bucket_angle < -1.0f || cos_bucket_angle > 1.0f) {
		return false; // Invalid solution
	}

	// Bucket angle relative to boom (interior angle)
	bucket_angle_boom = M_PI_F - acosf(cos_bucket_angle);

	return true;
}

void EndEffectorTrajectoryFollower::generate_boom_trajectory(float target_boom_angle, float boom_velocity)
{
	boom_trajectory_setpoint_s boom_traj{};
	boom_traj.timestamp = hrt_absolute_time();
	boom_traj.angle_setpoint = target_boom_angle;
	boom_traj.angular_velocity_setpoint = boom_velocity;
	boom_traj.coordinate_frame = 0; // Chassis frame
	boom_traj.control_mode = boom_trajectory_setpoint_s::CONTROL_MODE_POSITION;

	_boom_trajectory_pub.publish(boom_traj);
}

void EndEffectorTrajectoryFollower::generate_bucket_trajectory(float target_bucket_angle, float bucket_velocity)
{
	bucket_trajectory_setpoint_s bucket_traj{};
	bucket_traj.timestamp = hrt_absolute_time();
	bucket_traj.angle_setpoint = target_bucket_angle;
	bucket_traj.angular_velocity_setpoint = bucket_velocity;
	bucket_traj.coordinate_frame = 1; // Boom frame (relative to boom)
	bucket_traj.control_mode = bucket_trajectory_setpoint_s::CONTROL_MODE_POSITION;

	_bucket_trajectory_pub.publish(bucket_traj);
}

Vector3f EndEffectorTrajectoryFollower::compute_end_effector_position(float boom_angle_chassis, float bucket_angle_boom)
{
	// Forward kinematics for validation
	// Boom tip position in chassis frame
	float boom_tip_x = BOOM_LENGTH * cosf(boom_angle_chassis);
	float boom_tip_z = BOOM_LENGTH * sinf(boom_angle_chassis) + BOOM_PIVOT_HEIGHT;

	// Bucket angle in chassis frame
	float bucket_angle_chassis = boom_angle_chassis + bucket_angle_boom;

	// End effector position in chassis frame
	float ee_x = boom_tip_x + BUCKET_LENGTH * cosf(bucket_angle_chassis);
	float ee_z = boom_tip_z + BUCKET_LENGTH * sinf(bucket_angle_chassis);

	return Vector3f(ee_x, 0.0f, ee_z);
}

matrix::Vector3f EndEffectorTrajectoryFollower::transform_world_to_chassis(const matrix::Vector3f &position_world,
									   const matrix::Vector3f &chassis_position,
									   const matrix::Quatf &chassis_orientation)
{
	// Transform from world coordinates to chassis coordinates
	Vector3f relative_position = position_world - chassis_position;
	return chassis_orientation.inversed() * relative_position;
}int EndEffectorTrajectoryFollower::print_status()
{
	PX4_INFO("End Effector Trajectory Follower Status:");
	PX4_INFO("  Trajectory active: %s", _trajectory_active ? "YES" : "NO");
	PX4_INFO("  Last end effector position: [%.2f, %.2f, %.2f]",
		 (double)_last_end_effector_position(0),
		 (double)_last_end_effector_position(1),
		 (double)_last_end_effector_position(2));
	PX4_INFO("  Last boom angle: %.2f deg", (double)(_last_boom_angle * 180.0f / M_PI_F));
	PX4_INFO("  Last bucket angle: %.2f deg", (double)(_last_bucket_angle * 180.0f / M_PI_F));

	perf_print_counter(_loop_perf);
	perf_print_counter(_kinematics_perf);

	return 0;
}

int EndEffectorTrajectoryFollower::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int EndEffectorTrajectoryFollower::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s
", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
End effector trajectory follower for wheel loader. Takes end effector trajectory setpoints
(final position/orientation relative to chassis) and generates boom and bucket trajectory
setpoints using inverse kinematics.

Architecture:
- Input: End effector trajectory setpoint in chassis/world coordinates
- Processing: Inverse kinematics to compute boom + bucket joint angles
- Output: Boom trajectory setpoint + Bucket trajectory setpoint
- Controllers: Boom controller + Bucket controller execute the trajectories

Key coordinate transformations:
- End effector position: chassis/world frame → boom angle (chassis→boom) + bucket angle (boom→bucket)
- Boom trajectory: angle relative to chassis horizontal
- Bucket trajectory: angle relative to boom

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("end_effector_trajectory_follower", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

int EndEffectorTrajectoryFollower::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int EndEffectorTrajectoryFollower::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s
", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
End effector trajectory follower for wheel loader. Takes end effector trajectory setpoints
(bucket position/orientation from system perspective) and generates boom and bucket trajectory
setpoints using inverse kinematics.

Key Concept:
- End effector = bucket from system-wide perspective (global positioning)
- Bucket = same physical object from joint control perspective (local angles)

Architecture:
- Input: End effector trajectory setpoint in chassis/world coordinates
- Processing: Inverse kinematics to compute boom + bucket joint angles
- Output: Boom trajectory setpoint + Bucket trajectory setpoint
- Controllers: Boom controller + Bucket controller execute local joint control

Control Philosophy:
- This module thinks globally about end effector positioning
- Boom/bucket controllers think locally about joint angles
- Same physical tool, different control perspectives

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("end_effector_trajectory_follower", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int end_effector_trajectory_follower_main(int argc, char *argv[])
{
	return EndEffectorTrajectoryFollower::main(argc, argv);
}
