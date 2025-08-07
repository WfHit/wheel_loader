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
	// Using triangle formed by pivot, actuator base, and actuator attachment

	float a = _config.pivot_to_actuator_base;
	float b = _config.pivot_to_actuator_attach;
	float c = actuator_length;

	// Law of cosines to find angle at pivot
	float cos_gamma = (a * a + b * b - c * c) / (2.0f * a * b);
	cos_gamma = math::constrain(cos_gamma, -1.0f, 1.0f); // Ensure valid range

	float gamma = acosf(cos_gamma);
	float boom_angle = _config.actuator_base_angle - gamma;

	return boom_angle;
}

float BoomKinematics::boom_angle_to_actuator(float boom_angle) const
{
	// Inverse kinematics: Given boom angle, find required actuator length

	float a = _config.pivot_to_actuator_base;
	float b = _config.pivot_to_actuator_attach;
	float gamma = _config.actuator_base_angle - boom_angle;

	// Law of cosines to find actuator length (side c)
	float actuator_length = sqrtf(a * a + b * b - 2.0f * a * b * cosf(gamma));

	return actuator_length;
}

float BoomKinematics::encoder_to_actuator_length(float encoder_angle) const
{
	// Convert encoder angle to actuator length using calibration parameters
	float calibrated_angle = encoder_angle * _param_encoder_scale.get() + _param_encoder_offset.get();

	// Convert angle to actuator length using kinematic model
	// This assumes encoder is mounted to measure boom angle directly
	return boom_angle_to_actuator(math::radians(calibrated_angle));
}

float BoomKinematics::calculate_mechanical_advantage(float actuator_length) const
{
	// Calculate mechanical advantage at current position
	// This is the ratio of actuator force to boom torque

	float boom_angle = actuator_to_boom_angle(actuator_length);
	float a = _config.pivot_to_actuator_base;
	float b = _config.pivot_to_actuator_attach;
	float gamma = _config.actuator_base_angle - boom_angle;

	// Moment arm of actuator force about boom pivot
	float actuator_moment_arm = a * sinf(gamma);

	// Mechanical advantage is ratio of moment arms
	// (assuming boom load applied at end, distance = pivot_to_actuator_attach)
	float mechanical_advantage = actuator_moment_arm / b;

	return fabsf(mechanical_advantage);
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
	float a = _config.pivot_to_actuator_base;
	float b = _config.pivot_to_actuator_attach;
	float c = required_actuator_length;

	if (c > (a + b) || c < fabsf(a - b)) {
		return false;
	}

	return true;
}

void BoomKinematics::update_configuration()
{
	updateParams();

	_config.pivot_to_actuator_base = _param_pivot_to_base.get();
	_config.pivot_to_actuator_attach = _param_pivot_to_attach.get();
	_config.actuator_base_angle = math::radians(_param_base_angle.get());
	_config.actuator_min_length = _param_actuator_min.get();
	_config.actuator_max_length = _param_actuator_max.get();
	_config.boom_angle_min = math::radians(_param_angle_min.get());
	_config.boom_angle_max = math::radians(_param_angle_max.get());
}

bool BoomKinematics::validate_configuration() const
{
	// Check for reasonable values
	if (_config.pivot_to_actuator_base <= 0.0f ||
	    _config.pivot_to_actuator_attach <= 0.0f ||
	    _config.actuator_min_length <= 0.0f ||
	    _config.actuator_max_length <= _config.actuator_min_length) {
		PX4_ERR("Invalid kinematic configuration - check parameters");
		return false;
	}

	// Check triangle inequality for extreme positions
	float a = _config.pivot_to_actuator_base;
	float b = _config.pivot_to_actuator_attach;

	if (_config.actuator_max_length > (a + b)) {
		PX4_WARN("Maximum actuator length exceeds geometric limits");
		return false;
	}

	if (_config.actuator_min_length < fabsf(a - b)) {
		PX4_WARN("Minimum actuator length below geometric limits");
		return false;
	}

	return true;
}

float BoomKinematics::law_of_cosines_angle(float a, float b, float c) const
{
	// Calculate angle opposite to side c
	float cos_angle = (a * a + b * b - c * c) / (2.0f * a * b);
	cos_angle = math::constrain(cos_angle, -1.0f, 1.0f);
	return acosf(cos_angle);
}

float BoomKinematics::law_of_cosines_side(float a, float b, float gamma_angle) const
{
	// Calculate side c opposite to angle gamma
	return sqrtf(a * a + b * b - 2.0f * a * b * cosf(gamma_angle));
}
