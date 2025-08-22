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
#include <uORB/topics/chassis_trajectory_setpoint.h>
#include <uORB/topics/bucket_trajectory_setpoint.h>
#include <uORB/topics/chassis_control_command.h>
#include <lib/perf/perf_counter.h>

#include "trajectory_types.hpp"
#include "chassis_mpc_controller.hpp"

namespace wheel_loader
{

/**
 * Chassis trajectory follower with TinyMPC control and coordination
 * Runs at 50Hz for smooth motion control with predictive capabilities
 */
class ChassisTrajectoryFollower : public ModuleBase<ChassisTrajectoryFollower>,
                                  public ModuleParams,
                                  public px4::ScheduledWorkItem
{
public:
	ChassisTrajectoryFollower();
	~ChassisTrajectoryFollower();

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);
	static ChassisTrajectoryFollower *instantiate(int argc, char *argv[]);
	int print_status() override;

	bool init();

private:
	void Run() override;

	/**
	 * Update current vehicle state
	 */
	void update_vehicle_state();

	/**
	 * Process trajectory setpoint with coordination
	 */
	void process_trajectory_setpoint();

	/**
	 * Apply coordination with bucket motion
	 */
	void apply_coordination();

	/**
	 * Run MPC controller for optimal trajectory following
	 */
	void run_mpc_controller(float dt);

	/**
	 * Convert MPC output to actuator commands
	 */
	void convert_mpc_to_commands(const matrix::Vector<float, 2> &mpc_output);

	/**
	 * Apply safety constraints and limits
	 */
	void apply_safety_constraints();

	/**
	 * Publish control commands to actuator modules
	 */
	void publish_control_commands();

	/**
	 * Reset controllers and state
	 */
	void reset_controllers();

	// Parameters for MPC control
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::CTF_POS_P>) param_position_weight,
		(ParamFloat<px4::params::CTF_HDG_P>) param_heading_weight,
		(ParamFloat<px4::params::CTF_VEL_P>) param_velocity_weight,
		(ParamFloat<px4::params::CTF_ACC_R>) param_acceleration_weight,
		(ParamFloat<px4::params::CTF_STR_R>) param_steering_weight,
		(ParamFloat<px4::params::CTF_MAX_VEL>) param_max_velocity,
		(ParamFloat<px4::params::CTF_MAX_ACC>) param_max_acceleration,
		(ParamFloat<px4::params::CTF_MAX_STR>) param_max_steering,
		(ParamFloat<px4::params::CTF_COORD_EN>) param_coordination_enabled,
		(ParamFloat<px4::params::CTF_COORD_K>) param_coordination_gain,
		(ParamFloat<px4::params::CTF_WHEEL_B>) param_wheelbase
	)

	// Subscriptions
	uORB::Subscription chassis_trajectory_setpoint_sub{ORB_ID(chassis_trajectory_setpoint)};
	uORB::Subscription vehicle_local_position_sub{ORB_ID(vehicle_local_position)};
	uORB::Subscription bucket_trajectory_setpoint_sub{ORB_ID(bucket_trajectory_setpoint)}; // For coordination

	// Publications
	uORB::Publication<chassis_control_command_s> chassis_control_pub{ORB_ID(chassis_control_command)};

	// MPC Controller
	ChassisMPCController mpc_controller;

	// State
	chassis_trajectory_setpoint_s current_setpoint{};
	bucket_trajectory_setpoint_s bucket_setpoint{};  // For coordination
	vehicle_local_position_s current_position{};

	// Vehicle state vector for MPC [x, y, theta, v]
	matrix::Vector<float, 4> vehicle_state{};
	float current_velocity{0.0f};

	// MPC output
	matrix::Vector<float, 2> mpc_output{};  // [acceleration, steering]

	// Control outputs
	float velocity_command{0.0f};
	float steering_command{0.0f};

	// Coordination state
	float coordination_factor{0.0f};
	matrix::Vector3f bucket_influence{};

	// Safety
	bool emergency_stop{false};
	systemlib::Hysteresis setpoint_timeout_hysteresis{false};

	// Performance monitoring
	perf_counter_t loop_perf{nullptr};
	perf_counter_t mpc_perf{nullptr};
	perf_counter_t coordination_perf{nullptr};

	// Timing
	hrt_abstime last_update{0};
	hrt_abstime last_setpoint{0};
	static constexpr uint32_t RUN_INTERVAL{20_ms}; // 50Hz control loop
	static constexpr hrt_abstime SETPOINT_TIMEOUT{500_ms};
};

} // namespace wheel_loader
