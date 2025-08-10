/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the distribution in the documentation
 *    and/or other materials provided with the distribution.
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

#include "bucket_kinematics_tilt.hpp"
#include <px4_platform_common/log.h>
#include <mathlib/mathlib.h>

BucketKinematicsTilt::BucketKinematicsTilt(ModuleParams *parent) :
	ModuleParams(parent)
{
	update_configuration();
}

void BucketKinematicsTilt::update_configuration()
{
	// Stage 2 - Bucket Tilt Linkage (Boom-End Frame)
	_config.bucket_pivot = matrix::Vector2f(_param_bucket_pivot_x.get(), _param_bucket_pivot_y.get());

	// Link dimensions
	_config.bellcrank_length = _param_bellcrank_length.get();
	_config.coupler_length = _param_coupler_length.get();
	_config.bucket_arm_length = _param_bucket_arm_length.get();

	// Mechanical coupling and offsets
	_config.bellcrank_internal_angle = _param_bellcrank_internal_angle.get();
	_config.bucket_offset = _param_bucket_offset.get();

	// Physical and safety limits
	_config.bucket_angle_min = _param_bucket_angle_min.get();
	_config.bucket_angle_max = _param_bucket_angle_max.get();
}

// =========================
// MAIN KINEMATIC FUNCTIONS
// =========================

BucketKinematicsTilt::TiltState BucketKinematicsTilt::compute_forward_kinematics(
	float bellcrank_angle_drive, float boom_angle) const
{
	TiltState state{};
	state.is_valid = false;

	// Apply mechanical coupling: ∠OCB₂ = ∠OCB₁ + α
	state.bellcrank_angle = apply_stage_coupling(bellcrank_angle_drive);

	// Solve Stage 2: Bucket Tilt Linkage (Boom-End Frame)
	if (!solve_trigonometric(state.bellcrank_angle, boom_angle, state)) {
		PX4_WARN("Tilt Stage 2 solution failed for bellcrank angle: %.3f rad", (double)state.bellcrank_angle);
		return state;
	}

	// Final validation
	state.is_valid = check_mechanical_limits(state);

	return state;
}

float BucketKinematicsTilt::compute_inverse_kinematics(
	float bucket_angle, float boom_angle) const
{
	// Stage 2 Inverse: Find required bellcrank angle for desired bucket angle
	float required_bellcrank_tilt = solve_inverse_trigonometric(bucket_angle, boom_angle);
	if (!isfinite(required_bellcrank_tilt)) {
		PX4_WARN("Tilt Stage 2 inverse failed for bucket angle: %.3f rad", (double)bucket_angle);
		return NAN;
	}

	return required_bellcrank_tilt;
}

float BucketKinematicsTilt::apply_stage_coupling(float bellcrank_angle_drive) const
{
	// Coupling relationship: ∠OCB₂ = ∠OCB₁ + α
	// Where α is the internal angle between bellcrank planes
	return bellcrank_angle_drive + _config.bellcrank_internal_angle;
}

float BucketKinematicsTilt::remove_stage_coupling(float bellcrank_angle_tilt) const
{
	// Remove mechanical coupling to get Stage 1 angle
	// ∠OCB₁ = ∠OCB₂ - α
	return bellcrank_angle_tilt - _config.bellcrank_internal_angle;
}

// =========================
// TRIGONOMETRIC SOLUTION METHODS
// =========================

bool BucketKinematicsTilt::solve_trigonometric(float bellcrank_angle_tilt, float boom_angle, TiltState& state) const
{
	// Stage 2: Bucket Tilt Linkage (OABC) - Boom-End Frame
	// O: Boom end reference point (origin)
	// A: Bucket attachment (fixed to bucket)
	// B: Coupler joint (moving)
	// C: Bucket point (bellcrank pivot)
	//
	// Your calculation approach:
	// 1. Calculate OB length using angle OCB and known OC length and crank length (BC)
	// 2. Calculate angle AOB and angle BOC using known lengths OA, AB, OB
	// 3. Calculate angle AOC = AOB - BOC (bucket angle relative to boom)
	// 4. Add boom angle compensation to get bucket angle relative to chassis


	//TODO: update with law_of_cosines_angle


	// Known parameters
	float L_OA = _config.bucket_arm_length;      // OA length (parameter)
	float L_AB = _config.coupler_length;         // AB length (parameter)
	float L_BC = _config.bellcrank_length;       // Crank length (parameter)
	float L_OC = _config.bucket_pivot.norm();    // OC length (parameter)
	float angle_OCB = bellcrank_angle_tilt;      // angle OCB from stage 1

	// Step 1: Calculate OB length using law of cosines in triangle OBC
	// OB² = OC² + BC² - 2·OC·BC·cos(OCB)
	float L_OB_squared = L_OC * L_OC + L_BC * L_BC - 2.0f * L_OC * L_BC * cosf(angle_OCB);

	if (L_OB_squared <= 0.0f) {
		return false;  // Invalid geometry
	}

	float L_OB = sqrtf(L_OB_squared);

	// Step 2a: Calculate angle AOB using law of cosines in triangle AOB
	// cos(AOB) = (OA² + OB² - AB²) / (2·OA·OB)
	float cos_AOB = (L_OA * L_OA + L_OB * L_OB - L_AB * L_AB) / (2.0f * L_OA * L_OB);

	// Check if triangle AOB is valid
	if (cos_AOB < -1.0f || cos_AOB > 1.0f) {
		return false;  // Invalid triangle
	}

	float angle_AOB = acosf(cos_AOB);

	// Step 2b: Calculate angle BOC using law of cosines in triangle BOC
	// cos(BOC) = (OB² + OC² - BC²) / (2·OB·OC)
	float cos_BOC = (L_OB * L_OB + L_OC * L_OC - L_BC * L_BC) / (2.0f * L_OB * L_OC);

	// Check if calculation is valid
	if (cos_BOC < -1.0f || cos_BOC > 1.0f) {
		return false;  // Invalid triangle
	}

	float angle_BOC = acosf(cos_BOC);

	// Step 3: Calculate bucket angle relative to boom
	// angle AOC = AOB - BOC (assuming B is between A and C in angular sense)
	float angle_AOC_relative_to_boom = angle_AOB - angle_BOC;

	// Step 4: Apply boom angle compensation to get bucket angle relative to chassis
	// Final bucket angle = bucket angle relative to boom + boom angle + bucket offset
	state.bucket_angle = angle_AOC_relative_to_boom + boom_angle + _config.bucket_offset;

	// Store intermediate results
	state.bellcrank_angle = bellcrank_angle_tilt;
	state.coupler_angle = atan2f(sinf(angle_AOB), cosf(angle_AOB));  // Simplified coupler angle

	// Calculate joint B position for visualization/debugging
	// B is at distance L_OB from O at angle (angle_BOC from OC direction)
	float angle_OB = angle_BOC;  // Assuming OC is along x-axis reference
	state.joint_B = matrix::Vector2f{L_OB * cosf(angle_OB), L_OB * sinf(angle_OB)};

	return true;
}

float BucketKinematicsTilt::solve_inverse_trigonometric(float bucket_angle, float boom_angle) const
{
	// Inverse Stage 2: Given bucket angle (chassis-relative), find required bellcrank angle

	// Step 1: Remove boom angle and bucket offset to get bucket angle relative to boom
	float angle_AOC_relative_to_boom = bucket_angle - boom_angle - _config.bucket_offset;

	// Known parameters
	float L_OA = _config.bucket_arm_length;      // OA length
	float L_AB = _config.coupler_length;         // AB length
	float L_BC = _config.bellcrank_length;       // Crank length (BC)
	float L_OC = _config.bucket_pivot.norm();    // OC length

	// We need to solve for angle OCB that produces the desired angle AOC
	// Using the inverse of the forward kinematics calculation:
	//
	// Forward: angle_AOC = angle_AOB - angle_BOC
	// Inverse: We know angle_AOC, need to find angle_OCB (bellcrank angle)

	// This is a complex inverse problem. We can use an iterative approach
	// or geometric relationships. Let's use geometric approach:

	// From the desired bucket angle, we can calculate point A position

	//TODO:  do not need to calculate point A

	matrix::Vector2f point_A = {
		L_OA * cosf(angle_AOC_relative_to_boom),
		L_OA * sinf(angle_AOC_relative_to_boom)
	};

	// Point C is fixed at the bucket pivot location
	matrix::Vector2f point_C = {L_OC, 0.0f};  // Assuming C is along x-axis

	// Point B must satisfy two constraints:
	// 1. Distance AB = L_AB (coupler length)
	// 2. Distance BC = L_BC (bellcrank length)
	//
	// This means B is at the intersection of two circles:
	// Circle centered at A with radius L_AB
	// Circle centered at C with radius L_BC

	// Calculate distance AC
	matrix::Vector2f AC_vector = point_C - point_A;
	float L_AC = AC_vector.norm();

	// Check if solution is geometrically possible using triangle inequality
	if (!is_triangle_valid(L_AB, L_BC, L_AC)) {
		return NAN;  // No valid solution
	}

	// Use law of cosines to find angle ACB in triangle ABC
	// cos(ACB) = (AC² + BC² - AB²) / (2·AC·BC)
	float cos_ACB = (L_AC * L_AC + L_BC * L_BC - L_AB * L_AB) / (2.0f * L_AC * L_BC);

	// Check if calculation is valid
	if (cos_ACB < -1.0f || cos_ACB > 1.0f) {
		return NAN;  // Invalid triangle
	}

	float angle_ACB = acosf(cos_ACB);

	// Calculate angle from C to A in global coordinate system
	float angle_CA = atan2f(AC_vector(1), AC_vector(0));

	// The bellcrank angle (angle OCB) can be calculated as:
	// angle_OCB = angle_CA ± angle_ACB
	// We need to choose the correct sign based on the geometry

	// Try both solutions and pick the one that makes geometric sense
	float angle_OCB_1 = angle_CA + angle_ACB;
	float angle_OCB_2 = angle_CA - angle_ACB;

	// Validate both solutions by checking if they produce the correct bucket angle
	TiltState test_state_1{}, test_state_2{};
	bool valid_1 = solve_trigonometric(angle_OCB_1, boom_angle, test_state_1);
	bool valid_2 = solve_trigonometric(angle_OCB_2, boom_angle, test_state_2);

	// Choose the solution that produces the closest bucket angle
	if (valid_1 && valid_2) {
		float error_1 = fabsf(test_state_1.bucket_angle - bucket_angle);
		float error_2 = fabsf(test_state_2.bucket_angle - bucket_angle);
		return (error_1 < error_2) ? angle_OCB_1 : angle_OCB_2;
	} else if (valid_1) {
		return angle_OCB_1;
	} else if (valid_2) {
		return angle_OCB_2;
	} else {
		return NAN;  // No valid solution
	}
}

// =========================
// GEOMETRIC HELPER FUNCTIONS
// =========================

bool BucketKinematicsTilt::is_triangle_valid(float a, float b, float c) const
{
	// Check triangle inequality: sum of any two sides > third side
	return (a + b > c) && (a + c > b) && (b + c > a) &&
	       (a > GEOMETRIC_TOLERANCE) && (b > GEOMETRIC_TOLERANCE) && (c > GEOMETRIC_TOLERANCE);
}

// =========================
// VALIDATION AND ANALYSIS
// =========================

bool BucketKinematicsTilt::check_mechanical_limits(const TiltState &state) const
{
	// Check bucket angle limits
	if (state.bucket_angle < _config.bucket_angle_min ||
	    state.bucket_angle > _config.bucket_angle_max) {
		return false;
	}

	return true;
}

bool BucketKinematicsTilt::validate_configuration() const
{
	// Check for positive link lengths
	if (_config.bellcrank_length <= 0.0f ||
	    _config.coupler_length <= 0.0f ||
	    _config.bucket_arm_length <= 0.0f) {
		PX4_ERR("Invalid tilt linkage dimensions: all lengths must be positive");
		return false;
	}

	// Check bucket angle range
	if (_config.bucket_angle_min >= _config.bucket_angle_max) {
		PX4_ERR("Invalid bucket angle range: min (%.3f) >= max (%.3f)",
		        (double)_config.bucket_angle_min, (double)_config.bucket_angle_max);
		return false;
	}

	// Test trigonometric solutions at key points
	const float test_angles_drive[] = {
		0.0f,
		PI * 0.25f,
		PI * 0.5f,
		PI * 0.75f
	};

	for (float test_angle_drive : test_angles_drive) {
		TiltState state = compute_forward_kinematics(test_angle_drive, 0.0f);
		if (!state.is_valid) {
			continue;  // Some angles may be out of range, that's OK
		}

		// Test inverse kinematics round-trip
		float inverse_angle_tilt = compute_inverse_kinematics(state.bucket_angle, 0.0f);
		if (!isfinite(inverse_angle_tilt)) {
			PX4_ERR("Tilt inverse kinematics failed at test bucket angle: %.3f rad", (double)state.bucket_angle);
			return false;
		}

		// Check round-trip accuracy
		float angle_error = fabsf(inverse_angle_tilt - state.bellcrank_angle);
		if (angle_error > 0.01f) {  // 0.01 rad tolerance
			PX4_ERR("Tilt round-trip error too large: %.3f rad (tolerance: 0.01 rad)", (double)angle_error);
			return false;
		}
	}

	PX4_INFO("Bucket tilt kinematics configuration validated successfully");
	PX4_INFO("Bucket angle range: %.3f - %.3f rad",
	         (double)_config.bucket_angle_min, (double)_config.bucket_angle_max);

	return true;
}
