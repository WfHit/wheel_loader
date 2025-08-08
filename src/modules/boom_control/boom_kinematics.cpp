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

BoomKinematics::BoomKinematics(ModuleParams* parent) :
	ModuleParams(parent)
{
	update_configuration();
}

float BoomKinematics::actuator_to_boom_angle(float actuator_length) const
{
	// Forward kinematics: Given actuator length, find boom angle
	// Using triangle formed by boom_pivot, actuator_mount, and actuator_boom_joint

	float pivot_to_mount_dist = _config.pivot_to_mount_distance;  // Use cached value
	float pivot_to_actuator_joint_dist = _config.pivot_to_actuator_joint;      // Distance from pivot to actuator joint
	float mount_to_joint_dist = actuator_length;                      // Actuator length

	// Use law of cosines to find angle at boom pivot
	float angle_at_pivot = law_of_cosines_angle(pivot_to_mount_dist, pivot_to_actuator_joint_dist, mount_to_joint_dist);

	// Use cached angle of pivot-to-mount line from horizontal
	float mount_angle = _config.mount_angle_from_horizontal;

	// Calculate angle of pivot-to-actuator-joint line from horizontal
	float actuator_joint_angle = mount_angle + angle_at_pivot;

	// Calculate boom angle (pivot-to-bucket line from horizontal)
	// Add the fixed angle between actuator joint and bucket joint
	float boom_angle = actuator_joint_angle + _config.actuator_joint_to_bucket_angle;

	return boom_angle;
}

float BoomKinematics::boom_angle_to_actuator(float boom_angle) const
{
	// Inverse kinematics: Given boom angle, find required actuator length

	// Calculate angle of actuator joint line from horizontal
	float actuator_joint_angle = boom_angle - _config.actuator_joint_to_bucket_angle;

	// Use cached angle of actuator mount line from horizontal
	float mount_angle = _config.mount_angle_from_horizontal;

	// Calculate angle at boom pivot
	float angle_at_pivot = actuator_joint_angle - mount_angle;

	float pivot_to_mount_dist = _config.pivot_to_mount_distance;  // Use cached value
	float pivot_to_actuator_joint_dist = _config.pivot_to_actuator_joint;

	// Use law of cosines to find actuator length
	float actuator_length = law_of_cosines_side(pivot_to_mount_dist, pivot_to_actuator_joint_dist, angle_at_pivot);

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
	float pivot_to_mount_dist = _config.pivot_to_mount_distance;  // OA
	float pivot_to_actuator_joint_dist = _config.pivot_to_actuator_joint;  // OB

	// Use law of cosines: AB = sqrt(OA² + OB² - 2*OA*OB*cos(angle_OAB))
	float actuator_length = law_of_cosines_side(pivot_to_mount_dist, pivot_to_actuator_joint_dist, angle_oab);

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

void BoomKinematics::calculate_bucket_position(float boom_angle, float& x_pos, float& y_pos) const
{
	// Calculate bucket position from boom angle
	float pivot_to_bucket_dist = _config.pivot_to_bucket;

	x_pos = pivot_to_bucket_dist * cosf(boom_angle);
	y_pos = pivot_to_bucket_dist * sinf(boom_angle);
}

float BoomKinematics::calculate_bucket_height(float boom_angle) const
{
	// Calculate bucket height from ground
	float bucket_y_from_pivot = _config.pivot_to_bucket * sinf(boom_angle);
	float bucket_height = _config.pivot_height_from_ground + bucket_y_from_pivot;

	return bucket_height;
}

float BoomKinematics::calculate_mechanical_advantage(float boom_angle) const
{
	// Calculate mechanical advantage at current boom position
	float actuator_length = boom_angle_to_actuator(boom_angle);

	// Calculate moment arms
	float pivot_to_mount_dist = _config.pivot_to_mount_distance;  // Use cached value
	float actuator_joint_angle = boom_angle - _config.actuator_joint_to_bucket_angle;
	float mount_angle = _config.mount_angle_from_horizontal;  // Use cached value
	float angle_at_pivot = actuator_joint_angle - mount_angle;

	// Actuator moment arm
	float actuator_moment_arm = pivot_to_mount_dist * sinf(angle_at_pivot);

	// Boom moment arm (distance to load application point)
	float boom_moment_arm = _config.pivot_to_bucket * sinf(boom_angle);

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
	float x_pos, y_pos;
	calculate_bucket_position(state.boom_angle, x_pos, y_pos);
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
	// Check angle limits
	if (boom_angle < _config.boom_angle_min || boom_angle > _config.boom_angle_max) {
		return false;
	}

	// Check actuator length limits
	float required_actuator_length = boom_angle_to_actuator(boom_angle);
	if (required_actuator_length < _config.actuator_min_length ||
	    required_actuator_length > _config.actuator_max_length) {
		return false;
	}

	// Check for physical constraints (triangle inequality)
	float pivot_to_mount_dist = _config.pivot_to_mount_distance;  // Use cached value
	float pivot_to_actuator_joint_dist = _config.pivot_to_actuator_joint;
	float mount_to_joint_dist = required_actuator_length;

	if (mount_to_joint_dist > (pivot_to_mount_dist + pivot_to_actuator_joint_dist) ||
	    mount_to_joint_dist < fabsf(pivot_to_mount_dist - pivot_to_actuator_joint_dist)) {
		return false;
	}

	return true;
}

void BoomKinematics::update_configuration()
{
	updateParams();

	_config.actuator_mount_x = _param_actuator_mount_x.get();
	_config.actuator_mount_y = _param_actuator_mount_y.get();
	_config.pivot_to_actuator_joint = _param_pivot_to_actuator.get();
	_config.pivot_to_bucket = _param_pivot_to_bucket.get();
	_config.actuator_joint_to_bucket_angle = math::radians(_param_actuator_bucket_angle.get());
	_config.pivot_height_from_ground = _param_pivot_height.get();
	_config.actuator_min_length = _param_actuator_min.get();
	_config.actuator_max_length = _param_actuator_max.get();
	_config.boom_angle_min = math::radians(_param_angle_min.get());
	_config.boom_angle_max = math::radians(_param_angle_max.get());
	_config.encoder_angle_at_min = _param_encoder_min_angle.get();
	_config.encoder_angle_at_max = _param_encoder_max_angle.get();

	// Compute and cache derived values that don't change unless parameters change
	_config.pivot_to_mount_distance = sqrtf(_config.actuator_mount_x * _config.actuator_mount_x +
	                                       _config.actuator_mount_y * _config.actuator_mount_y);
	_config.mount_angle_from_horizontal = atan2f(_config.actuator_mount_y, _config.actuator_mount_x);
}

bool BoomKinematics::validate_configuration() const
{
	// Check for reasonable values
	if (_config.pivot_to_actuator_joint <= 0.0f ||
	    _config.pivot_to_bucket <= 0.0f ||
	    _config.actuator_min_length <= 0.0f ||
	    _config.actuator_max_length <= _config.actuator_min_length ||
	    _config.pivot_height_from_ground <= 0.0f) {
		PX4_ERR("Invalid kinematic configuration - check parameters");
		return false;
	}

	// Check triangle inequality for extreme positions
	float pivot_to_mount_dist = _config.pivot_to_mount_distance;  // Use cached value
	float pivot_to_actuator_joint_dist = _config.pivot_to_actuator_joint;

	if (_config.actuator_max_length > (pivot_to_mount_dist + pivot_to_actuator_joint_dist)) {
		PX4_WARN("Maximum actuator length exceeds geometric limits");
		return false;
	}

	if (_config.actuator_min_length < fabsf(pivot_to_mount_dist - pivot_to_actuator_joint_dist)) {
		PX4_WARN("Minimum actuator length below geometric limits");
		return false;
	}

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
	float pivot_to_mount_dist = _config.pivot_to_mount_distance;  // Use cached value
	float pivot_to_actuator_joint_dist = _config.pivot_to_actuator_joint;
	float mount_to_joint_dist = actuator_length;

	return law_of_cosines_angle(pivot_to_mount_dist, pivot_to_actuator_joint_dist, mount_to_joint_dist);
}

float BoomKinematics::calculate_mount_angle_in_actuator_triangle(float actuator_length) const
{
	// Calculate angle at actuator mount in actuator triangle
	float pivot_to_mount_dist = _config.pivot_to_mount_distance;  // Use cached value
	float pivot_to_actuator_joint_dist = _config.pivot_to_actuator_joint;
	float mount_to_joint_dist = actuator_length;

	return law_of_cosines_angle(pivot_to_mount_dist, mount_to_joint_dist, pivot_to_actuator_joint_dist);
}

float BoomKinematics::calculate_boom_joint_angle_in_actuator_triangle(float actuator_length) const
{
	// Calculate angle at boom joint in actuator triangle
	float pivot_to_mount_dist = _config.pivot_to_mount_distance;  // Use cached value
	float pivot_to_actuator_joint_dist = _config.pivot_to_actuator_joint;
	float mount_to_joint_dist = actuator_length;

	return law_of_cosines_angle(pivot_to_actuator_joint_dist, mount_to_joint_dist, pivot_to_mount_dist);
}

float BoomKinematics::calculate_actuator_joint_angle(float actuator_length) const
{
	// Calculate angle of actuator joint line from horizontal
	float angle_at_pivot = calculate_pivot_angle_in_actuator_triangle(actuator_length);
	float mount_angle = _config.mount_angle_from_horizontal;  // Use cached value

	return mount_angle + angle_at_pivot;
}

float BoomKinematics::law_of_cosines_angle(float side1, float side2, float opposite_side) const
{
	// Calculate angle opposite to the given side using law of cosines
	float cos_angle = (side1 * side1 + side2 * side2 - opposite_side * opposite_side) / (2.0f * side1 * side2);
	cos_angle = math::constrain(cos_angle, -1.0f, 1.0f);
	return acosf(cos_angle);
}

float BoomKinematics::law_of_cosines_side(float side1, float side2, float included_angle) const
{
	// Calculate side opposite to the given angle using law of cosines
	return sqrtf(side1 * side1 + side2 * side2 - 2.0f * side1 * side2 * cosf(included_angle));
}

float BoomKinematics::calculate_pivot_to_mount_distance() const
{
	// Return cached value - this function kept for backward compatibility
	return _config.pivot_to_mount_distance;
}

float BoomKinematics::calculate_mount_angle_from_horizontal() const
{
	// Return cached value - this function kept for backward compatibility
	return _config.mount_angle_from_horizontal;
}
