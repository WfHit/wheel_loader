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

#include "chassis_trajectory_follower.hpp"
#include <px4_platform_common/getopt.h>
#include <lib/mathlib/mathlib.h>

using namespace wheel_loader;
using namespace matrix;

ChassisTrajectoryFollower::ChassisTrajectoryFollower() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	// Initialize hysteresis for setpoint timeout
	setpoint_timeout_hysteresis.set_hysteresis_time_from(false, SETPOINT_TIMEOUT);
}

ChassisTrajectoryFollower::~ChassisTrajectoryFollower()
{
	perf_free(loop_perf);
	perf_free(mpc_perf);
	perf_free(coordination_perf);
}

bool ChassisTrajectoryFollower::init()
{
	// Schedule at 50Hz
	ScheduleOnInterval(RUN_INTERVAL);

	// Initialize performance counters
	loop_perf = perf_alloc(PC_ELAPSED, MODULE_NAME": cycle");
	mpc_perf = perf_alloc(PC_ELAPSED, MODULE_NAME": mpc");
	coordination_perf = perf_alloc(PC_ELAPSED, MODULE_NAME": coordination");

	// Initialize MPC controller
	mpc_controller.init(param_wheelbase.get(), param_max_velocity.get(), param_max_steering.get());

	// Set MPC weights
	Vector<float, 4> state_weights;
	state_weights(0) = param_position_weight.get();   // x position
	state_weights(1) = param_position_weight.get();   // y position
	state_weights(2) = param_heading_weight.get();    // heading
	state_weights(3) = param_velocity_weight.get();   // velocity

	Vector<float, 2> control_weights;
	control_weights(0) = param_acceleration_weight.get();  // acceleration
	control_weights(1) = param_steering_weight.get();      // steering

	mpc_controller.set_weights(state_weights, control_weights);

	return true;
}

void ChassisTrajectoryFollower::Run()
{
	perf_begin(loop_perf);

	// Update vehicle state
	update_vehicle_state();

	// Process trajectory setpoint
	process_trajectory_setpoint();

	// Check for setpoint timeout
	const hrt_abstime now = hrt_absolute_time();
	setpoint_timeout_hysteresis.set_state_and_update(
		(now - last_setpoint) > SETPOINT_TIMEOUT, now);

	if (setpoint_timeout_hysteresis.get_state()) {
		// No valid setpoint - emergency stop
		emergency_stop = true;
		reset_controllers();
	} else {
		emergency_stop = false;

		// Calculate time delta
		const float dt = math::constrain((now - last_update) * 1e-6f, 0.001f, 0.1f);
		last_update = now;

		// Apply coordination with bucket if enabled
		if (param_coordination_enabled.get() > 0.5f) {
			perf_begin(coordination_perf);
			apply_coordination();
			perf_end(coordination_perf);
		}

		// Run MPC controller
		perf_begin(mpc_perf);
		run_mpc_controller(dt);
		perf_end(mpc_perf);

		// Convert MPC output to actuator commands
		convert_mpc_to_commands(mpc_output);

		// Apply safety constraints
		apply_safety_constraints();
	}

	// Publish control commands
	publish_control_commands();

	perf_end(loop_perf);
}

void ChassisTrajectoryFollower::update_vehicle_state()
{
	// Update current position
	if (vehicle_local_position_sub.updated()) {
		vehicle_local_position_sub.copy(&current_position);

		// Update vehicle state vector for MPC
		vehicle_state(0) = current_position.x;        // x position
		vehicle_state(1) = current_position.y;        // y position
		vehicle_state(2) = current_position.heading;  // heading

		// Calculate velocity magnitude
		current_velocity = sqrtf(current_position.vx * current_position.vx +
		                        current_position.vy * current_position.vy);
		vehicle_state(3) = current_velocity;          // velocity
	}
}

void ChassisTrajectoryFollower::process_trajectory_setpoint()
{
	// Update trajectory setpoint
	if (chassis_trajectory_setpoint_sub.updated()) {
		chassis_trajectory_setpoint_sub.copy(&current_setpoint);
		last_setpoint = hrt_absolute_time();
	}
}

void ChassisTrajectoryFollower::apply_coordination()
{
	// Get bucket setpoint for coordination
	if (bucket_trajectory_setpoint_sub.updated()) {
		bucket_trajectory_setpoint_sub.copy(&bucket_setpoint);
	}

	if (!bucket_setpoint.valid) {
		coordination_factor = 0.0f;
		return;
	}

	// Calculate bucket influence on chassis motion
	Vector3f bucket_pos(bucket_setpoint.x_position, bucket_setpoint.y_position, bucket_setpoint.z_position);
	Vector2f chassis_pos(current_position.x, current_position.y);

	// If bucket is reaching far, chassis should help by moving closer
	Vector2f bucket_direction = bucket_pos.xy() - chassis_pos;
	float reach_distance = bucket_direction.norm();
	const float max_reach = 3.0f;  // Robot specific parameter

	if (reach_distance > max_reach * 0.6f) {
		// Calculate coordination factor
		coordination_factor = math::constrain(
			(reach_distance - max_reach * 0.5f) / (max_reach * 0.5f),
			0.0f, 1.0f
		);

		// Modify setpoint to assist bucket motion
		Vector2f assistance_vector = bucket_direction.normalized() *
		                            coordination_factor * param_coordination_gain.get();

		// Adjust chassis target position
		current_setpoint.x_position += assistance_vector(0);
		current_setpoint.y_position += assistance_vector(1);

		// Adjust chassis target heading to face bucket
		float bucket_heading = atan2f(bucket_direction(1), bucket_direction(0));
		float heading_blend = coordination_factor * 0.3f;  // Gentle heading adjustment
		current_setpoint.yaw = (1.0f - heading_blend) * current_setpoint.yaw +
		                       heading_blend * bucket_heading;
	} else {
		coordination_factor = 0.0f;
	}
}

void ChassisTrajectoryFollower::run_mpc_controller(float dt)
{
	if (!current_setpoint.valid) {
		mpc_output.setZero();
		return;
	}

	// Run MPC solver
	mpc_output = mpc_controller.solve(vehicle_state, current_setpoint, dt);
}

void ChassisTrajectoryFollower::convert_mpc_to_commands(const Vector<float, 2> &mpc_output)
{
	// MPC output: [acceleration, steering_angle]
	float acceleration_cmd = mpc_output(0);
	float steering_angle_cmd = mpc_output(1);

	// Convert acceleration to velocity command (simple integration)
	velocity_command = current_velocity + acceleration_cmd * 0.02f;  // 50Hz * 0.02s = dt

	// Direct steering command
	steering_command = steering_angle_cmd;
}

void ChassisTrajectoryFollower::apply_safety_constraints()
{
	// Apply velocity constraints
	velocity_command = math::constrain(
		velocity_command,
		-param_max_velocity.get(),
		param_max_velocity.get()
	);

	// Apply steering constraints
	steering_command = math::constrain(
		steering_command,
		-param_max_steering.get(),
		param_max_steering.get()
	);

	// Reduce velocity during sharp turns (kinematic constraint)
	const float wheelbase = param_wheelbase.get();
	const float min_turn_radius = 1.0f;  // Minimum safe turn radius

	if (fabsf(steering_command) > 0.1f) {
		float turn_radius = wheelbase / tanf(fabsf(steering_command));
		if (turn_radius < min_turn_radius) {
			// Limit velocity for tight turns
			float max_vel_for_turn = min_turn_radius * fabsf(steering_command) / wheelbase;
			velocity_command = math::constrain(velocity_command,
			                                  -max_vel_for_turn, max_vel_for_turn);
		}
	}

	// Emergency stop override
	if (emergency_stop) {
		velocity_command = 0.0f;
		steering_command = 0.0f;
	}
}

void ChassisTrajectoryFollower::publish_control_commands()
{
	chassis_control_command_s cmd{};
	cmd.velocity = velocity_command;
	cmd.steering_angle = steering_command;
	cmd.timestamp = hrt_absolute_time();
	cmd.valid = !emergency_stop && current_setpoint.valid;

	chassis_control_pub.publish(cmd);
}

void ChassisTrajectoryFollower::reset_controllers()
{
	velocity_command = 0.0f;
	steering_command = 0.0f;
	mpc_output.setZero();
	coordination_factor = 0.0f;
}

int ChassisTrajectoryFollower::print_status()
{
	PX4_INFO("Chassis Trajectory Follower Status:");
	PX4_INFO("  Emergency Stop: %s", emergency_stop ? "YES" : "NO");
	PX4_INFO("  Setpoint Valid: %s", current_setpoint.valid ? "YES" : "NO");
	PX4_INFO("  Velocity Command: %.2f m/s", (double)velocity_command);
	PX4_INFO("  Steering Command: %.2f rad", (double)steering_command);
	PX4_INFO("  Coordination Factor: %.2f", (double)coordination_factor);
	PX4_INFO("  Current Velocity: %.2f m/s", (double)current_velocity);

	return 0;
}

// Module framework functions would go here...
