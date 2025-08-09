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

#include "boom_kinematics.hpp"
#include <px4_platform_common/log.h>
#include <mathlib/mathlib.h>

BoomKinematics::BoomKinematics(ModuleParams *parent) :
	ModuleParams(parent)
{
	// Initialize configuration
	update_configuration();
}

float BoomKinematics::actuator_to_boom_angle(float actuator_length) const
{
	// Forward kinematics: Given actuator length, find boom angle
	// Using triangle formed by boom_pivot, actuator_mount, and actuator_boom_joint

	float pivot_to_mount_dist = _config.actuator_base_to_pivot_length;  // Use parameter value
	float actuator_to_pivot_dist = _config.actuator_joint_to_pivot_length;      // Distance from actuator to pivot
	float mount_to_joint_dist = actuator_length;                      // Actuator length

	// Use law of cosines to find angle at boom pivot
	float angle_at_pivot = law_of_cosines_angle(pivot_to_mount_dist, actuator_to_pivot_dist, mount_to_joint_dist);

	// Use cached angle of pivot-to-mount line from horizontal
	float mount_angle = _config.actuator_base_to_pivot_angle;

	// Calculate angle of pivot-to-actuator-joint line from horizontal
	float actuator_joint_angle = mount_angle + angle_at_pivot;

	// Calculate boom angle (pivot-to-bucket line from horizontal)
	// Add the fixed angle between actuator joint and bucket joint
	float boom_angle = actuator_joint_angle + _config.actuator_joint_to_boom_diff_angle;

	return boom_angle;
}

float BoomKinematics::boom_angle_to_actuator(float boom_angle) const
{
	// Inverse kinematics: Given boom angle, find required actuator length

	// Calculate angle of actuator joint line from horizontal
	float actuator_joint_angle = boom_angle - _config.actuator_joint_to_boom_diff_angle;

	// Use cached angle of actuator mount line from horizontal
	float mount_angle = _config.actuator_base_to_pivot_angle;

	// Calculate angle at boom pivot
	float angle_at_pivot = actuator_joint_angle - mount_angle;

	float pivot_to_mount_dist = _config.actuator_base_to_pivot_length;  // Use parameter value
	float actuator_to_pivot_dist = _config.actuator_joint_to_pivot_length;

	// Use law of cosines to find actuator length
	float actuator_length = law_of_cosines_side(pivot_to_mount_dist, actuator_to_pivot_dist, angle_at_pivot);

	return actuator_length;
}

float BoomKinematics::encoder_to_actuator_length(float encoder_angle) const
{
	// Convert AS5600 encoder angle to actuator length
	// The AS5600 returns absolute angle, so relationship with geometric angle OAB is just an offset

	// Apply offset calibration to get the actual geometric angle OAB
	// encoder_angle_at_min corresponds to the encoder reading when actuator is at minimum length
	float angle_oab = encoder_angle - _config.encoder_angle_at_min;

	// Convert geometric angle OAB to actuator length using law of cosines
	float pivot_to_mount_dist = _config.actuator_base_to_pivot_length;  // OA
	float actuator_to_pivot_dist = _config.actuator_joint_to_pivot_length;  // OB

	// Use law of cosines: AB = sqrt(OA² + OB² - 2*OA*OB*cos(angle_OAB))
	float actuator_length = law_of_cosines_side(pivot_to_mount_dist, actuator_to_pivot_dist, angle_oab);

	return actuator_length;
}

float BoomKinematics::actuator_length_to_encoder(float actuator_length) const
{
	// Convert actuator length to boom angle using inverse kinematics
	float boom_angle = actuator_to_boom_angle(actuator_length);

	// Calculate encoder angle: Direct offset relationship with AS5600
	float encoder_angle = math::degrees(boom_angle) + _config.encoder_angle_at_min;

	// Wrap to 0-360 degrees for encoder
	encoder_angle = fmodf(encoder_angle + 360.0f, 360.0f);

	return encoder_angle;
}

void BoomKinematics::calculate_bucket_position(float boom_angle, float &x_pos, float &z_pos) const
{
	// Calculate bucket position from boom angle
	float boom_length_dist = _config.boom_length;

	x_pos = boom_length_dist * cosf(boom_angle);
	z_pos = boom_length_dist * sinf(boom_angle);
}

float BoomKinematics::calculate_bucket_height(float boom_angle) const
{
	// Calculate bucket height from ground
	float bucket_z_from_pivot = _config.boom_length * sinf(boom_angle);
	float bucket_height = _config.pivot_height_from_ground + bucket_z_from_pivot;

	return bucket_height;
}

float BoomKinematics::calculate_mechanical_advantage(float boom_angle) const
{
	// Calculate mechanical advantage at current boom position

	// Calculate moment arms
	float pivot_to_mount_dist = _config.actuator_base_to_pivot_length;  // Use parameter value
	float actuator_joint_angle = boom_angle - _config.actuator_joint_to_boom_diff_angle;
	float mount_angle = _config.actuator_base_to_pivot_angle;  // Use parameter value
	float angle_at_pivot = actuator_joint_angle - mount_angle;

	// Actuator moment arm
	float actuator_moment_arm = pivot_to_mount_dist * sinf(angle_at_pivot);

	// Boom moment arm (distance to load application point)
	float boom_moment_arm = _config.boom_length * sinf(boom_angle);

	// Mechanical advantage
	float mechanical_advantage = actuator_moment_arm / boom_moment_arm;

	return fabsf(mechanical_advantage);
}

BoomKinematics::KinematicState BoomKinematics::get_kinematic_state(float actuator_length) const
{
	KinematicState state = {};

	// Calculate boom angle
	state.boom_angle = actuator_to_boom_angle(actuator_length);
	state.actuator_length = actuator_length;

	// Calculate bucket position
	float x_pos, z_pos;
	calculate_bucket_position(state.boom_angle, x_pos, z_pos);
	state.bucket_reach = x_pos;
	state.bucket_height = calculate_bucket_height(state.boom_angle);

	// Calculate mechanical advantage
	state.mechanical_advantage = calculate_mechanical_advantage(state.boom_angle);

	// Validate position
	state.is_valid = is_position_valid(state.boom_angle);

	return state;
}

BoomKinematics::KinematicState BoomKinematics::get_kinematic_state_from_encoder(float encoder_angle) const
{
	float actuator_length = encoder_to_actuator_length(encoder_angle);
	return get_kinematic_state(actuator_length);
}

bool BoomKinematics::is_position_valid(float boom_angle) const
{
	// Check for basic triangle validity
	float required_actuator_length = boom_angle_to_actuator(boom_angle);

	// Check for physical constraints (triangle inequality)
	float pivot_to_mount_dist = _config.actuator_base_to_pivot_length;  // Use parameter value
	float actuator_to_pivot_dist = _config.actuator_joint_to_pivot_length;
	float mount_to_joint_dist = required_actuator_length;

	if (mount_to_joint_dist > (pivot_to_mount_dist + actuator_to_pivot_dist) ||
	    mount_to_joint_dist < fabsf(pivot_to_mount_dist - actuator_to_pivot_dist)) {
		return false;
	}

	return true;
}

void BoomKinematics::update_configuration()
{
	// Load configuration parameters using DEFINE_PARAMETERS approach
	_config.boom_length = _param_boom_length.get();
	_config.actuator_joint_to_pivot_length = _param_actuator_joint_to_pivot_length.get();
	_config.actuator_base_to_pivot_length = _param_actuator_base_to_pivot_length.get();
	_config.actuator_joint_to_boom_end_length = _param_actuator_joint_to_boom_end_length.get();
	_config.actuator_length_at_zero = _param_actuator_zero_length.get();
	_config.pivot_height_from_ground = _param_pivot_height.get();
	_config.encoder_angle_at_min = _param_encoder_min_angle.get();
	_config.encoder_angle_at_max = _param_encoder_max_angle.get();
	_config.encoder_angle_at_zero = _param_encoder_zero_angle.get();

	// Calculate computed values
	_config.actuator_joint_to_boom_diff_angle = calculate_actuator_joint_to_boom_diff_angle();

	// Calculate actuator_base_to_pivot_angle using pivot angle at zero length plus boom differential angle
	_config.actuator_base_to_pivot_angle = calculate_actuator_base_to_pivot_angle();
}

float BoomKinematics::calculate_actuator_base_to_pivot_angle()
{
	// Calculate actuator_base_to_pivot_angle as:
	// pivot angle when actuator_length_at_zero + actuator_joint_to_boom_diff_angle

	// First calculate the pivot angle in the actuator triangle when actuator is at zero length
	float pivot_angle_at_zero = calculate_pivot_angle_in_actuator_triangle(_config.actuator_length_at_zero);

	// Add the actuator joint to boom differential angle
	float actuator_base_to_pivot_angle = pivot_angle_at_zero + _config.actuator_joint_to_boom_diff_angle;

	return actuator_base_to_pivot_angle;  // Return in radians
}

bool BoomKinematics::validate_configuration() const
{
	// Check for reasonable values
	if (_config.boom_length <= 0.0f ||
	    _config.actuator_joint_to_pivot_length <= 0.0f ||
	    _config.actuator_base_to_pivot_length <= 0.0f ||
	    _config.actuator_joint_to_boom_end_length <= 0.0f ||
	    _config.actuator_length_at_zero <= 0.0f ||
	    _config.pivot_height_from_ground <= 0.0f) {
		PX4_ERR("Invalid kinematic configuration - check parameters");
		return false;
	}

	// Check triangle inequality for boom structure (pivot, actuator joint, bucket)
	float boom_length = _config.boom_length;
	float actuator_to_pivot = _config.actuator_joint_to_pivot_length;
	float actuator_to_bucket = _config.actuator_joint_to_boom_end_length;

	// Triangle inequality: sum of any two sides must be greater than third side
	if ((actuator_to_pivot + boom_length <= actuator_to_bucket) ||
	    (actuator_to_pivot + actuator_to_bucket <= boom_length) ||
	    (boom_length + actuator_to_bucket <= actuator_to_pivot)) {
		PX4_ERR("Invalid boom geometry - triangle inequality violated");
		return false;
	}

	// Check triangle inequality for extreme positions
	float pivot_to_mount_dist = _config.actuator_base_to_pivot_length;  // Use parameter value
	float actuator_to_pivot_dist = _config.actuator_joint_to_pivot_length;

	// Check encoder calibration
	if (_config.encoder_angle_at_max <= _config.encoder_angle_at_min) {
		PX4_WARN("Invalid encoder calibration - max angle must be greater than min angle");
		return false;
	}

	return true;
}

float BoomKinematics::calculate_pivot_angle_in_actuator_triangle(float actuator_length) const
{
	// Calculate angle at boom pivot in actuator triangle
	float pivot_to_mount_dist = _config.actuator_base_to_pivot_length;  // Use parameter value
	float pivot_to_actuator_joint_dist = _config.actuator_joint_to_pivot_length;
	float mount_to_joint_dist = actuator_length;

	return law_of_cosines_angle(pivot_to_mount_dist, pivot_to_actuator_joint_dist, mount_to_joint_dist);
}

float BoomKinematics::calculate_base_mount_angle_in_actuator_triangle(float actuator_length) const
{
	// Calculate angle at actuator base mount in actuator triangle
	float pivot_to_mount_dist = _config.actuator_base_to_pivot_length;  // Use parameter value
	float pivot_to_actuator_joint_dist = _config.actuator_joint_to_pivot_length;
	float mount_to_joint_dist = actuator_length;

	return law_of_cosines_angle(pivot_to_mount_dist, mount_to_joint_dist, pivot_to_actuator_joint_dist);
}

float BoomKinematics::calculate_actuator_joint_angle_in_actuator_triangle(float actuator_length) const
{
	// Calculate angle at actuator joint in actuator triangle
	float pivot_to_mount_dist = _config.actuator_base_to_pivot_length;  // Use parameter value
	float pivot_to_actuator_joint_dist = _config.actuator_joint_to_pivot_length;
	float mount_to_joint_dist = actuator_length;

	return law_of_cosines_angle(pivot_to_actuator_joint_dist, mount_to_joint_dist, pivot_to_mount_dist);
}

float BoomKinematics::calculate_actuator_joint_angle(float actuator_length) const
{
	// Calculate angle of actuator joint line from horizontal
	float angle_at_pivot = calculate_pivot_angle_in_actuator_triangle(actuator_length);
	float mount_angle = _config.actuator_base_to_pivot_angle;  // Use parameter value

	return mount_angle + angle_at_pivot;
}

float BoomKinematics::calculate_actuator_joint_to_boom_diff_angle() const
{
	// Calculate angle between actuator joint and boom centerline
	// This is a fixed geometric relationship based on boom design

	// Using the triangle formed by boom_pivot, actuator_joint, and bucket_joint
	float boom_to_joint = _config.actuator_joint_to_pivot_length;
	float boom_to_bucket = _config.boom_length;
	float joint_to_bucket = _config.actuator_joint_to_boom_end_length;

	// Use law of cosines to find angle at boom pivot between boom centerline and actuator joint
	return law_of_cosines_angle(boom_to_bucket, boom_to_joint, joint_to_bucket);
}

float BoomKinematics::law_of_cosines_side(float side_a, float side_b, float angle_c) const
{
	// Calculate the third side of a triangle given two sides and included angle
	// c² = a² + b² - 2ab*cos(C)
	float side_c_sq = side_a * side_a + side_b * side_b - 2.0f * side_a * side_b * cosf(angle_c);
	return sqrtf(side_c_sq);
}

float BoomKinematics::law_of_cosines_angle(float side_a, float side_b, float side_c) const
{
	// Calculate angle C opposite to side c
	// cos(C) = (a² + b² - c²) / (2ab)
	float cos_c = (side_a * side_a + side_b * side_b - side_c * side_c) / (2.0f * side_a * side_b);
	return acosf(math::constrain(cos_c, -1.0f, 1.0f));
}

float BoomKinematics::get_actuator_base_to_pivot_length() const
{
	return _config.actuator_base_to_pivot_length;
}
