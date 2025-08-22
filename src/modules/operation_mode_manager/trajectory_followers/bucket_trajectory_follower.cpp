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
#include <px4_platform_common/getopt.h>
#include <lib/mathlib/mathlib.h>

using namespace wheel_loader;
using namespace matrix;

BucketTrajectoryFollower::BucketTrajectoryFollower() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::rate_ctrl)
{
	// Initialize hysteresis for setpoint timeout
	setpoint_timeout_hysteresis.set_hysteresis_time_from(false, SETPOINT_TIMEOUT);
}

BucketTrajectoryFollower::~BucketTrajectoryFollower()
{
	perf_free(loop_perf);
	perf_free(pid_perf);
	perf_free(kinematics_perf);
}

bool BucketTrajectoryFollower::init()
{
	// Schedule at 50Hz
	ScheduleOnInterval(RUN_INTERVAL);

	// Initialize performance counters
	loop_perf = perf_alloc(PC_ELAPSED, MODULE_NAME": cycle");
	pid_perf = perf_alloc(PC_ELAPSED, MODULE_NAME": pid");
	kinematics_perf = perf_alloc(PC_ELAPSED, MODULE_NAME": kinematics");

	// Initialize PID controllers

	// Position controllers (X, Y, Z)
	pid_init(&pid_x, PID_MODE_DERIVATIV_CALC, 0.02f);
	pid_set_parameters(&pid_x,
		param_pid_p_x_p.get(), param_pid_p_x_i.get(), param_pid_p_x_d.get(),
		param_max_force.get(), -param_max_force.get());

	pid_init(&pid_y, PID_MODE_DERIVATIV_CALC, 0.02f);
	pid_set_parameters(&pid_y,
		param_pid_p_y_p.get(), param_pid_p_y_i.get(), param_pid_p_y_d.get(),
		param_max_force.get(), -param_max_force.get());

	pid_init(&pid_z, PID_MODE_DERIVATIV_CALC, 0.02f);
	pid_set_parameters(&pid_z,
		param_pid_p_z_p.get(), param_pid_p_z_i.get(), param_pid_p_z_d.get(),
		param_max_force.get(), -param_max_force.get());

	// Orientation controllers (Roll, Pitch, Yaw)
	pid_init(&pid_roll, PID_MODE_DERIVATIV_CALC, 0.02f);
	pid_set_parameters(&pid_roll,
		param_pid_roll_p.get(), param_pid_roll_i.get(), param_pid_roll_d.get(),
		1.0f, -1.0f);  // Normalized torque output

	pid_init(&pid_pitch, PID_MODE_DERIVATIV_CALC, 0.02f);
	pid_set_parameters(&pid_pitch,
		param_pid_pitch_p.get(), param_pid_pitch_i.get(), param_pid_pitch_d.get(),
		1.0f, -1.0f);

	pid_init(&pid_yaw, PID_MODE_DERIVATIV_CALC, 0.02f);
	pid_set_parameters(&pid_yaw,
		param_pid_yaw_p.get(), param_pid_yaw_i.get(), param_pid_yaw_d.get(),
		1.0f, -1.0f);

	return true;
}

void BucketTrajectoryFollower::Run()
{
	perf_begin(loop_perf);

	// Update bucket state
	update_bucket_state();

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

		// Run PID controllers
		perf_begin(pid_perf);
		run_pid_controllers(dt);
		perf_end(pid_perf);

		// Apply gravity compensation
		apply_gravity_compensation();

		// Apply load compensation
		apply_load_compensation();

		// Convert PID outputs to actuator commands
		perf_begin(kinematics_perf);
		convert_pid_to_commands();
		perf_end(kinematics_perf);

		// Apply safety constraints
		apply_safety_constraints();
	}

	// Publish control commands
	publish_control_commands();

	perf_end(loop_perf);
}

void BucketTrajectoryFollower::update_bucket_state()
{
	// Update boom position
	if (boom_position_sub.updated()) {
		boom_position_sub.copy(&current_boom_position);
	}

	// Update bucket position
	if (bucket_position_sub.updated()) {
		bucket_position_sub.copy(&current_bucket_position);
	}

	// Update load sensor
	if (load_sensor_sub.updated()) {
		load_sensor_sub.copy(&current_load);
		current_load_weight = current_load.weight;
		load_center_of_mass = Vector3f(current_load.center_of_mass);
	}

	// Calculate current bucket position and orientation in world coordinates
	// This requires forward kinematics calculation

	// Boom parameters
	float boom_extension = current_boom_position.extension;
	float boom_lift_angle = current_boom_position.lift_angle;
	Vector3f boom_offset(param_boom_offset_x.get(),
	                    param_boom_offset_y.get(),
	                    param_boom_offset_z.get());

	// Bucket parameters
	float bucket_tilt = current_bucket_position.tilt_angle;
	float bucket_roll = current_bucket_position.roll_angle;
	float bucket_pitch = current_bucket_position.pitch_angle;

	// Forward kinematics calculation
	// Boom tip position (simplified - assumes boom extends horizontally then lifts)
	Vector3f boom_tip;
	boom_tip(0) = boom_offset(0) + boom_extension * cosf(boom_lift_angle);
	boom_tip(1) = boom_offset(1);
	boom_tip(2) = boom_offset(2) + boom_extension * sinf(boom_lift_angle);

	// Bucket tip position (bucket extends from boom tip)
	float bucket_length = param_bucket_length.get();
	Vector3f bucket_direction;
	bucket_direction(0) = cosf(boom_lift_angle + bucket_tilt);
	bucket_direction(1) = 0.0f;  // Simplified - no lateral bucket motion
	bucket_direction(2) = sinf(boom_lift_angle + bucket_tilt);

	current_position = boom_tip + bucket_direction * bucket_length;

	// Bucket orientation
	current_orientation(0) = bucket_roll;
	current_orientation(1) = bucket_pitch;
	current_orientation(2) = boom_lift_angle + bucket_tilt;  // Effective yaw
}

void BucketTrajectoryFollower::process_trajectory_setpoint()
{
	// Update trajectory setpoint
	if (bucket_trajectory_setpoint_sub.updated()) {
		bucket_trajectory_setpoint_sub.copy(&current_setpoint);
		last_setpoint = hrt_absolute_time();
	}
}

void BucketTrajectoryFollower::run_pid_controllers(float dt)
{
	if (!current_setpoint.valid) {
		position_output.setZero();
		orientation_output.setZero();
		return;
	}

	// Calculate position errors
	Vector3f position_error = current_setpoint.position - current_position;
	Vector3f orientation_error = current_setpoint.orientation - current_orientation;

	// Wrap yaw error to [-π, π]
	orientation_error(2) = wrap_pi(orientation_error(2));

	// Run position PID controllers
	position_output(0) = pid_calculate(&pid_x, position_error(0), 0.0f, dt);
	position_output(1) = pid_calculate(&pid_y, position_error(1), 0.0f, dt);
	position_output(2) = pid_calculate(&pid_z, position_error(2), 0.0f, dt);

	// Run orientation PID controllers
	orientation_output(0) = pid_calculate(&pid_roll, orientation_error(0), 0.0f, dt);
	orientation_output(1) = pid_calculate(&pid_pitch, orientation_error(1), 0.0f, dt);
	orientation_output(2) = pid_calculate(&pid_yaw, orientation_error(2), 0.0f, dt);
}

void BucketTrajectoryFollower::apply_gravity_compensation()
{
	// Apply gravity compensation to Z axis
	const float gravity_compensation = param_gravity_comp_gain.get() * 9.81f;
	position_output(2) += gravity_compensation;
}

void BucketTrajectoryFollower::apply_load_compensation()
{
	if (current_load_weight > 0.1f) {  // Minimum detectable load
		// Compensate for load weight
		float load_compensation = current_load_weight * param_load_comp_gain.get();
		position_output(2) += load_compensation;

		// Adjust for center of mass offset
		Vector3f com_offset = load_center_of_mass;
		position_output += com_offset * (load_compensation * 0.1f);  // Small adjustment
	}
}

void BucketTrajectoryFollower::convert_pid_to_commands()
{
	// This is a simplified inverse kinematics solution
	// In practice, this would require a more sophisticated kinematic solver

	// Extract forces and torques
	Vector3f forces = position_output;
	Vector3f torques = orientation_output;

	// Convert forces to joint commands using simplified Jacobian
	// Boom extension command (X direction primarily)
	boom_extension_cmd = forces(0) * 0.01f;  // Simple scaling

	// Boom lift command (Z direction primarily)
	boom_lift_cmd = forces(2) * 0.01f;

	// Bucket tilt command (affects Z position and pitch)
	bucket_tilt_cmd = (forces(2) * 0.005f) + (torques(1) * 0.1f);

	// Bucket orientation commands
	bucket_roll_cmd = torques(0) * 0.1f;
	bucket_pitch_cmd = torques(1) * 0.1f;
	bucket_yaw_cmd = torques(2) * 0.1f;
}

void BucketTrajectoryFollower::apply_safety_constraints()
{
	// Check joint limits
	Vector3f test_position = current_position;
	Vector3f test_orientation = current_orientation;
	joint_limits_violated = !check_joint_limits(test_position, test_orientation);

	// Apply joint limit constraints
	boom_extension_cmd = math::constrain(boom_extension_cmd, -1.0f, 1.0f);
	boom_lift_cmd = math::constrain(boom_lift_cmd, -1.0f, 1.0f);
	bucket_tilt_cmd = math::constrain(bucket_tilt_cmd, -1.0f, 1.0f);
	bucket_roll_cmd = math::constrain(bucket_roll_cmd, -1.0f, 1.0f);
	bucket_pitch_cmd = math::constrain(bucket_pitch_cmd, -1.0f, 1.0f);
	bucket_yaw_cmd = math::constrain(bucket_yaw_cmd, -1.0f, 1.0f);

	// Emergency stop override
	if (emergency_stop || joint_limits_violated) {
		boom_extension_cmd = 0.0f;
		boom_lift_cmd = 0.0f;
		bucket_tilt_cmd = 0.0f;
		bucket_roll_cmd = 0.0f;
		bucket_pitch_cmd = 0.0f;
		bucket_yaw_cmd = 0.0f;
	}
}

bool BucketTrajectoryFollower::check_joint_limits(const Vector3f &position, const Vector3f &orientation)
{
	// Check boom extension limits
	float boom_ext = current_boom_position.extension;
	if (boom_ext < param_boom_ext_min.get() || boom_ext > param_boom_ext_max.get()) {
		return false;
	}

	// Check boom lift limits
	float boom_lift = current_boom_position.lift_angle;
	if (boom_lift < param_boom_lift_min.get() || boom_lift > param_boom_lift_max.get()) {
		return false;
	}

	// Check bucket tilt limits
	float bucket_tilt = current_bucket_position.tilt_angle;
	if (bucket_tilt < param_bucket_tilt_min.get() || bucket_tilt > param_bucket_tilt_max.get()) {
		return false;
	}

	return true;
}

void BucketTrajectoryFollower::publish_control_commands()
{
	BucketControlCommand cmd{};
	cmd.boom_extension = boom_extension_cmd;
	cmd.boom_lift = boom_lift_cmd;
	cmd.bucket_tilt = bucket_tilt_cmd;
	cmd.bucket_roll = bucket_roll_cmd;
	cmd.bucket_pitch = bucket_pitch_cmd;
	cmd.bucket_yaw = bucket_yaw_cmd;
	cmd.timestamp = hrt_absolute_time();
	cmd.valid = !emergency_stop && !joint_limits_violated && current_setpoint.valid;

	bucket_control_pub.publish(cmd);
}

void BucketTrajectoryFollower::reset_controllers()
{
	// Reset PID controllers
	pid_reset_integral(&pid_x);
	pid_reset_integral(&pid_y);
	pid_reset_integral(&pid_z);
	pid_reset_integral(&pid_roll);
	pid_reset_integral(&pid_pitch);
	pid_reset_integral(&pid_yaw);

	// Reset outputs
	position_output.setZero();
	orientation_output.setZero();

	// Reset commands
	boom_extension_cmd = 0.0f;
	boom_lift_cmd = 0.0f;
	bucket_tilt_cmd = 0.0f;
	bucket_roll_cmd = 0.0f;
	bucket_pitch_cmd = 0.0f;
	bucket_yaw_cmd = 0.0f;
}

int BucketTrajectoryFollower::print_status()
{
	PX4_INFO("Bucket Trajectory Follower Status:");
	PX4_INFO("  Emergency Stop: %s", emergency_stop ? "YES" : "NO");
	PX4_INFO("  Joint Limits Violated: %s", joint_limits_violated ? "YES" : "NO");
	PX4_INFO("  Setpoint Valid: %s", current_setpoint.valid ? "YES" : "NO");
	PX4_INFO("  Current Position: (%.2f, %.2f, %.2f)",
		(double)current_position(0), (double)current_position(1), (double)current_position(2));
	PX4_INFO("  Target Position: (%.2f, %.2f, %.2f)",
		(double)current_setpoint.position(0), (double)current_setpoint.position(1), (double)current_setpoint.position(2));
	PX4_INFO("  Load Weight: %.2f kg", (double)current_load_weight);

	return 0;
}

// Module framework functions would go here...
