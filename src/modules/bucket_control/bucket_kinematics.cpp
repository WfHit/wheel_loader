/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#include "bucket_kinematics.hpp"
#include <px4_platform_common/log.h>
#include <mathlib/mathlib.h>

BucketKinematics::BucketKinematics(ModuleParams *parent) :
	ModuleParams(parent)
{
	update_configuration();
}

void BucketKinematics::update_configuration()
{
	// Load attachment points
	_config.actuator_base = matrix::Vector2f(_param_actuator_base_x.get(), _param_actuator_base_y.get());
	_config.bellcrank_boom = matrix::Vector2f(_param_bellcrank_boom_x.get(), _param_bellcrank_boom_y.get());
	_config.bucket_pivot = matrix::Vector2f(_param_bucket_pivot_x.get(), _param_bucket_pivot_y.get());

	// Load linkage dimensions
	_config.bellcrank_length = _param_bellcrank_length.get();
	_config.coupler_length = _param_coupler_length.get();
	_config.actuator_offset = _param_actuator_offset.get();
	_config.bucket_arm_length = _param_bucket_arm_length.get();

	// Load angular constraints
	_config.bellcrank_internal_angle = _param_bellcrank_internal_angle.get();
	_config.bucket_offset = _param_bucket_offset.get();

	// Load physical limits
	_config.actuator_min_length = _param_actuator_min.get();
	_config.actuator_max_length = _param_actuator_max.get();
	_config.bucket_angle_min = _param_bucket_angle_min.get();
	_config.bucket_angle_max = _param_bucket_angle_max.get();
}

BucketKinematics::LinkageState BucketKinematics::compute_forward_kinematics(
	float actuator_length, float boom_angle) const
{
	LinkageState state{};
	state.actuator_length = actuator_length;
	state.is_valid = false;

	// Solve linkage geometry
	if (!solve_linkage(actuator_length, boom_angle, state)) {
		// Fallback to linear approximation
		float normalized = (actuator_length - _config.actuator_min_length) /
				   (_config.actuator_max_length - _config.actuator_min_length);
		state.bucket_angle = _config.bucket_angle_min +
				     normalized * (_config.bucket_angle_max - _config.bucket_angle_min);
		state.is_valid = true;
		state.condition_number = SINGULARITY_THRESHOLD; // Mark as approximate
		PX4_DEBUG("Forward kinematics using linear approximation");
	}

	return state;
}

BucketKinematics::LinkageState BucketKinematics::compute_inverse_kinematics(
	float bucket_angle, float boom_angle) const
{
	LinkageState state{};
	state.bucket_angle = bucket_angle;
	state.is_valid = false;

	// Use iterative solver (binary search) to find actuator length
	float length_min = _config.actuator_min_length;
	float length_max = _config.actuator_max_length;

	for (int i = 0; i < MAX_ITERATIONS; i++) {
		float length_mid = (length_min + length_max) * 0.5f;
		LinkageState test_state = compute_forward_kinematics(length_mid, boom_angle);

		if (!test_state.is_valid) {
			break;
		}

		float error = bucket_angle - test_state.bucket_angle;

		if (fabsf(error) < CONVERGENCE_TOL) {
			state = test_state;
			state.actuator_length = length_mid;
			state.is_valid = true;
			break;
		}

		if (error > 0) {
			// Need longer actuator
			length_min = length_mid;
		} else {
			// Need shorter actuator
			length_max = length_mid;
		}
	}

	if (!state.is_valid) {
		PX4_DEBUG("Inverse kinematics failed to converge");
	}

	return state;
}

bool BucketKinematics::solve_linkage(float actuator_length, float boom_angle, LinkageState &state) const
{
	// Transform coordinates to account for boom rotation
	float cos_boom = cosf(boom_angle);
	float sin_boom = sinf(boom_angle);

	// Actuator base is fixed to chassis, rotates relative to boom
	float act_base_x_boom =
		_config.actuator_base(0) * cos_boom + _config.actuator_base(1) * sin_boom;
	float act_base_y_boom =
		-_config.actuator_base(0) * sin_boom + _config.actuator_base(1) * cos_boom;

	// Bellcrank pivot is fixed to boom (no transformation needed)
	float bellcrank_pivot_x = _config.bellcrank_boom(0);
	float bellcrank_pivot_y = _config.bellcrank_boom(1);

	// Step 1: Solve for bellcrank angle using actuator triangle
	float dx = bellcrank_pivot_x - act_base_x_boom;
	float dy = bellcrank_pivot_y - act_base_y_boom;
	float base_to_bellcrank_dist = sqrtf(dx * dx + dy * dy);

	// Law of cosines to find angle at bellcrank pivot
	float cos_bellcrank_angle = (base_to_bellcrank_dist * base_to_bellcrank_dist +
				     _config.actuator_offset * _config.actuator_offset -
				     actuator_length * actuator_length) /
				    (2.0f * base_to_bellcrank_dist * _config.actuator_offset);

	if (fabsf(cos_bellcrank_angle) > 1.0f) {
		// No valid solution - actuator cannot reach
		return false;
	}

	float base_angle = atan2f(dy, dx);
	state.bellcrank_angle = base_angle - acosf(cos_bellcrank_angle); // Subtract because actuator pulls

	// Step 2: Find bellcrank coupler attachment point using internal angle
	float coupler_arm_angle = state.bellcrank_angle + _config.bellcrank_internal_angle;
	float bellcrank_end_x = bellcrank_pivot_x + _config.bellcrank_length * cosf(coupler_arm_angle);
	float bellcrank_end_y = bellcrank_pivot_y + _config.bellcrank_length * sinf(coupler_arm_angle);

	// Step 3: Calculate bucket coupler attachment point
	float bucket_pivot_x = _config.bucket_pivot(0);
	float bucket_pivot_y = _config.bucket_pivot(1);

	dx = bucket_pivot_x - bellcrank_end_x;
	dy = bucket_pivot_y - bellcrank_end_y;
	float coupler_required_length = sqrtf(dx * dx + dy * dy);

	// Check if coupler can span the distance
	if (fabsf(coupler_required_length - _config.coupler_length) > 1.0f) { // 1mm tolerance
		return false;
	}

	state.coupler_angle = atan2f(dy, dx);

	// Step 4: Calculate bucket angle using bucket arm geometry
	float bucket_arm_angle = state.coupler_angle + M_PI_F + _config.bucket_offset; // +PI because coupler pulls bucket
	state.bucket_angle = bucket_arm_angle; // This is the bucket's orientation

	// Step 5: Validate mechanical limits and compute condition number
	if (!check_mechanical_limits(state)) {
		return false;
	}

	state.condition_number = compute_condition_number(state);
	state.is_valid = true;

	return true;
}

bool BucketKinematics::check_mechanical_limits(const LinkageState &state) const
{
	// Check actuator limits
	if (state.actuator_length < _config.actuator_min_length ||
	    state.actuator_length > _config.actuator_max_length) {
		return false;
	}

	// Check bucket angle limits
	if (state.bucket_angle < _config.bucket_angle_min ||
	    state.bucket_angle > _config.bucket_angle_max) {
		return false;
	}

	// Check for mechanical interference (simplified)
	// Additional geometric constraints could be added here

	return true;
}

float BucketKinematics::compute_condition_number(const LinkageState &state) const
{
	// Compute Jacobian and its condition number
	matrix::Matrix<float, 2, 2> jacobian = compute_jacobian(state);

	// Calculate 2x2 determinant manually: det = ad - bc
	float det = jacobian(0, 0) * jacobian(1, 1) - jacobian(0, 1) * jacobian(1, 0);

	if (fabsf(det) < 1e-8f) {
		return SINGULARITY_THRESHOLD;
	}

	// Manual Frobenius norm calculation for condition number
	float norm = sqrtf(jacobian(0, 0) * jacobian(0, 0) + jacobian(0, 1) * jacobian(0, 1) +
	                   jacobian(1, 0) * jacobian(1, 0) + jacobian(1, 1) * jacobian(1, 1));
	return norm / fabsf(det);
}

matrix::Matrix<float, 2, 2> BucketKinematics::compute_jacobian(const LinkageState &state) const
{
	matrix::Matrix<float, 2, 2> jacobian;

	// Numerical differentiation for Jacobian computation
	const float h = 1e-4f; // Small perturbation

	// Partial derivative with respect to actuator length
	LinkageState state_plus, state_minus;
	state_plus = compute_forward_kinematics(state.actuator_length + h);
	state_minus = compute_forward_kinematics(state.actuator_length - h);

	if (state_plus.is_valid && state_minus.is_valid) {
		jacobian(0, 0) = (state_plus.bucket_angle - state_minus.bucket_angle) / (2.0f * h);
	} else {
		jacobian(0, 0) = 0.0f;
	}

	// Partial derivative with respect to boom angle (for boom compensation)
	// This represents how bucket angle changes with boom angle
	jacobian(0, 1) = 1.0f; // Simplified: bucket rotates with boom

	// Second row for bellcrank angle derivatives (if needed)
	jacobian(1, 0) = 0.0f; // Placeholder
	jacobian(1, 1) = 0.0f; // Placeholder

	return jacobian;
}

bool BucketKinematics::validate_configuration() const
{
	// Check for positive lengths
	if (_config.bellcrank_length <= 0.0f ||
	    _config.coupler_length <= 0.0f ||
	    _config.actuator_offset <= 0.0f ||
	    _config.bucket_arm_length <= 0.0f) {
		PX4_ERR("Invalid linkage lengths");
		return false;
	}

	// Check actuator range
	if (_config.actuator_min_length >= _config.actuator_max_length) {
		PX4_ERR("Invalid actuator length range");
		return false;
	}

	// Check bucket angle range
	if (_config.bucket_angle_min >= _config.bucket_angle_max) {
		PX4_ERR("Invalid bucket angle range");
		return false;
	}

	// Test forward and inverse kinematics at several points
	const float test_lengths[] = {
		_config.actuator_min_length,
		(_config.actuator_min_length + _config.actuator_max_length) * 0.5f,
		_config.actuator_max_length
	};

	for (float test_length : test_lengths) {
		LinkageState state = compute_forward_kinematics(test_length);
		if (!state.is_valid) {
			PX4_ERR("Forward kinematics failed at test length");
			return false;
		}

		LinkageState inverse_state = compute_inverse_kinematics(state.bucket_angle);
		if (!inverse_state.is_valid) {
			PX4_ERR("Inverse kinematics failed at test angle");
			return false;
		}

		// Check round-trip accuracy
		float length_error = fabsf(inverse_state.actuator_length - test_length);
		if (length_error > 1.0f) { // 1mm tolerance
			PX4_ERR("Round-trip error too large");
			return false;
		}
	}

	PX4_INFO("Bucket kinematics configuration validated");
	return true;
}

float BucketKinematics::get_boom_compensation_factor(float boom_angle) const
{
	// Compensation factor calculation based on linkage geometry
	// This converts boom angle change to required actuator length change

	// Base factor at nominal position (can be calibrated)
	const float BASE_FACTOR = 250.0f;  // mm/rad
	const float ANGLE_SENSITIVITY = 0.2f;  // Factor change per radian

	// Adjust factor based on current boom angle
	float factor = BASE_FACTOR * (1.0f + ANGLE_SENSITIVITY * boom_angle);

	// Limit factor to reasonable range
	return math::constrain(factor, 100.0f, 400.0f);
}
