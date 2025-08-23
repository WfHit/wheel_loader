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
#include <uORB/Subscription.hpp>
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/bucket_trajectory_setpoint.h>
#include <uORB/topics/bucket_control_command.h>
#include <uORB/topics/boom_position.h>
#include <uORB/topics/bucket_position.h>
#include <uORB/topics/load_sensor.h>
#include <lib/matrix/matrix/math.hpp>
#include <lib/perf/perf_counter.h>
#include <lib/pid/pid.h>
#include <lib/hysteresis/hysteresis.h>

#include "trajectory_types.hpp"

namespace wheel_loader
{

/**
 * @class BucketTrajectoryFollower
 * @brief Trajectory follower for bucket (boom + bucket) motion using 6-DOF PID control
 *
 * This class implements trajectory following for the bucket assembly (boom and bucket)
 * using separate PID controllers for each degree of freedom. The controller takes
 * BucketTrajectorySetpoint in world coordinates and outputs bucket actuator commands.
 *
 * Features:
 * - 6-DOF PID control (boom x,y,z + bucket roll,pitch,yaw)
 * - Safety constraints and joint limits
 * - Gravity compensation
 * - Load handling considerations
 * - Emergency stop capability
 */
class BucketTrajectoryFollower : public ModuleBase<BucketTrajectoryFollower>,
                                public ModuleParams,
                                public px4::ScheduledWorkItem
{
public:
	BucketTrajectoryFollower();
	~BucketTrajectoryFollower() override;

	bool init() override;

	int print_status() override;

	static int task_spawn(int argc, char *argv[]);

	static BucketTrajectoryFollower *instantiate(int argc, char *argv[]);

	static int custom_command(int argc, char *argv[]);

	static int print_usage(const char *reason = nullptr);

private:
	static constexpr int RUN_INTERVAL = 20000; // 50Hz in microseconds
	static constexpr hrt_abstime SETPOINT_TIMEOUT = 500000; // 0.5 seconds

	void Run() override;

	/**
	 * Update bucket state from sensors
	 */
	void update_bucket_state();

	/**
	 * Process incoming trajectory setpoint
	 */
	void process_trajectory_setpoint();

	/**
	 * Run 6-DOF PID controllers
	 */
	void run_pid_controllers(float dt);

	/**
	 * Apply safety constraints and joint limits
	 */
	void apply_safety_constraints();

	/**
	 * Apply gravity compensation
	 */
	void apply_gravity_compensation();

	/**
	 * Apply load handling compensation
	 */
	void apply_load_compensation();

	/**
	 * Convert PID outputs to actuator commands
	 */
	void convert_pid_to_commands();

	/**
	 * Publish bucket control commands
	 */
	void publish_control_commands();

	/**
	 * Reset all controllers
	 */
	void reset_controllers();

	/**
	 * Check joint limits
	 */
	bool check_joint_limits(const matrix::Vector3f &position, const matrix::Vector3f &orientation);

	// uORB subscriptions
	uORB::Subscription bucket_trajectory_setpoint_sub{ORB_ID(bucket_trajectory_setpoint)};
	uORB::Subscription boom_position_sub{ORB_ID(boom_position)};
	uORB::Subscription bucket_position_sub{ORB_ID(bucket_position)};
	uORB::Subscription load_sensor_sub{ORB_ID(load_sensor)};

	// uORB publications
	uORB::Publication<bucket_control_command_s> bucket_control_pub{ORB_ID(bucket_control_command)};

	// Current state
	bucket_trajectory_setpoint_s current_setpoint{};
	boom_position_s current_boom_position{};
	bucket_position_s current_bucket_position{};
	load_sensor_s current_load{};

	// Current bucket state in world coordinates
	matrix::Vector3f current_position{};    // World position of bucket tip
	matrix::Vector3f current_orientation{}; // Roll, pitch, yaw
	matrix::Vector3f current_velocity{};    // Linear velocity
	matrix::Vector3f current_angular_velocity{}; // Angular velocity

	// PID controllers for 6-DOF control
	PID_t pid_x{};      // X position controller
	PID_t pid_y{};      // Y position controller
	PID_t pid_z{};      // Z position controller
	PID_t pid_roll{};   // Roll controller
	PID_t pid_pitch{};  // Pitch controller
	PID_t pid_yaw{};    // Yaw controller

	// Control outputs
	matrix::Vector3f position_output{}; // Position control output (forces)
	matrix::Vector3f orientation_output{}; // Orientation control output (torques)

	// Bucket actuator commands
	float boom_extension_cmd{0.0f};
	float boom_lift_cmd{0.0f};
	float bucket_tilt_cmd{0.0f};
	float bucket_roll_cmd{0.0f};
	float bucket_pitch_cmd{0.0f};
	float bucket_yaw_cmd{0.0f};

	// Safety and status
	bool emergency_stop{false};
	bool joint_limits_violated{false};
	systemlib::Hysteresis setpoint_timeout_hysteresis{false};
	hrt_abstime last_setpoint{0};
	hrt_abstime last_update{0};

	// Load handling
	float current_load_weight{0.0f};
	matrix::Vector3f load_center_of_mass{};

	// Performance counters
	perf_counter_t loop_perf{nullptr};
	perf_counter_t pid_perf{nullptr};
	perf_counter_t kinematics_perf{nullptr};

	// Parameters
	DEFINE_PARAMETERS(
		// Position PID gains
		(ParamFloat<px4::params::OMM_BUCKET_PID_PX_P>) param_pid_p_x_p,
		(ParamFloat<px4::params::OMM_BUCKET_PID_PX_I>) param_pid_p_x_i,
		(ParamFloat<px4::params::OMM_BUCKET_PID_PX_D>) param_pid_p_x_d,

		(ParamFloat<px4::params::OMM_BUCKET_PID_PY_P>) param_pid_p_y_p,
		(ParamFloat<px4::params::OMM_BUCKET_PID_PY_I>) param_pid_p_y_i,
		(ParamFloat<px4::params::OMM_BUCKET_PID_PY_D>) param_pid_p_y_d,

		(ParamFloat<px4::params::OMM_BUCKET_PID_PZ_P>) param_pid_p_z_p,
		(ParamFloat<px4::params::OMM_BUCKET_PID_PZ_I>) param_pid_p_z_i,
		(ParamFloat<px4::params::OMM_BUCKET_PID_PZ_D>) param_pid_p_z_d,

		// Orientation PID gains
		(ParamFloat<px4::params::OMM_BUCKET_PID_R_P>) param_pid_roll_p,
		(ParamFloat<px4::params::OMM_BUCKET_PID_R_I>) param_pid_roll_i,
		(ParamFloat<px4::params::OMM_BUCKET_PID_R_D>) param_pid_roll_d,

		(ParamFloat<px4::params::OMM_BUCKET_PID_P_P>) param_pid_pitch_p,
		(ParamFloat<px4::params::OMM_BUCKET_PID_P_I>) param_pid_pitch_i,
		(ParamFloat<px4::params::OMM_BUCKET_PID_P_D>) param_pid_pitch_d,

		(ParamFloat<px4::params::OMM_BUCKET_PID_Y_P>) param_pid_yaw_p,
		(ParamFloat<px4::params::OMM_BUCKET_PID_Y_I>) param_pid_yaw_i,
		(ParamFloat<px4::params::OMM_BUCKET_PID_Y_D>) param_pid_yaw_d,

		// Safety constraints
		(ParamFloat<px4::params::OMM_BUCKET_MAX_VEL>) param_max_velocity,
		(ParamFloat<px4::params::OMM_BUCKET_MAX_ACCEL>) param_max_acceleration,
		(ParamFloat<px4::params::OMM_BUCKET_MAX_FORCE>) param_max_force,

		// Joint limits
		(ParamFloat<px4::params::OMM_BOOM_EXT_MIN>) param_boom_ext_min,
		(ParamFloat<px4::params::OMM_BOOM_EXT_MAX>) param_boom_ext_max,
		(ParamFloat<px4::params::OMM_BOOM_LIFT_MIN>) param_boom_lift_min,
		(ParamFloat<px4::params::OMM_BOOM_LIFT_MAX>) param_boom_lift_max,
		(ParamFloat<px4::params::OMM_BUCKET_TILT_MIN>) param_bucket_tilt_min,
		(ParamFloat<px4::params::OMM_BUCKET_TILT_MAX>) param_bucket_tilt_max,

		// Gravity compensation
		(ParamFloat<px4::params::OMM_BUCKET_GRAVITY_COMP>) param_gravity_comp_gain,
		(ParamFloat<px4::params::OMM_LOAD_COMP_GAIN>) param_load_comp_gain,

		// Bucket kinematics
		(ParamFloat<px4::params::OMM_BOOM_LENGTH>) param_boom_length,
		(ParamFloat<px4::params::OMM_BUCKET_LENGTH>) param_bucket_length,
		(ParamFloat<px4::params::OMM_BOOM_OFFSET_X>) param_boom_offset_x,
		(ParamFloat<px4::params::OMM_BOOM_OFFSET_Y>) param_boom_offset_y,
		(ParamFloat<px4::params::OMM_BOOM_OFFSET_Z>) param_boom_offset_z
	)
};

} // namespace wheel_loader
