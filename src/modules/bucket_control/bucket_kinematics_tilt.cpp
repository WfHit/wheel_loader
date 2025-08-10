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
	float bellcrank_angle_s1, float boom_angle) const
{
	TiltState state{};
	state.is_valid = false;

	// Apply mechanical coupling: ∠OCB₂ = ∠OCB₁ + α
	state.bellcrank_angle = apply_stage_coupling(bellcrank_angle_s1);

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
	float required_bellcrank_s2 = solve_inverse_trigonometric(bucket_angle, boom_angle);
	if (!isfinite(required_bellcrank_s2)) {
		PX4_WARN("Tilt Stage 2 inverse failed for bucket angle: %.3f rad", (double)bucket_angle);
		return NAN;
	}

	return required_bellcrank_s2;
}

float BucketKinematicsTilt::apply_stage_coupling(float bellcrank_angle_s1) const
{
	// Apply mechanical coupling between Stage 1 and Stage 2
	// ∠OCB₂ = ∠OCB₁ + α (where α is the internal bellcrank angle)
	return bellcrank_angle_s1 + _config.bellcrank_internal_angle;
}

float BucketKinematicsTilt::remove_stage_coupling(float bellcrank_angle_s2) const
{
	// Remove mechanical coupling to get Stage 1 angle
	// ∠OCB₁ = ∠OCB₂ - α
	return bellcrank_angle_s2 - _config.bellcrank_internal_angle;
}

// =========================
// TRIGONOMETRIC SOLUTION METHODS
// =========================

bool BucketKinematicsTilt::solve_trigonometric(float bellcrank_angle_s2, float boom_angle, TiltState& state) const
{
	// Stage 2: Bucket Tilt Linkage (OABC) - Boom-End Frame
	// O: Boom end reference point
	// A: Bucket attachment (fixed to bucket)
	// B: Coupler joint (moving)
	// C: Bucket point (final output)

	// Stage 2 link lengths
	float L_OA = _config.bucket_arm_length;   // Bucket arm
	float L_AB = _config.coupler_length;      // Coupler link
	float L_BC = _config.bellcrank_length;    // Bellcrank (other half)
	float L_OC = _config.bucket_pivot.norm();  // Boom structure

	// Method 2: Coordinate geometry approach
	// Known angle OCB₂ from Stage 1 coupling
	matrix::Vector2f point_C = {L_OC, 0.0f};  // Along boom direction
	matrix::Vector2f point_B_direction = {cosf(bellcrank_angle_s2), sinf(bellcrank_angle_s2)};
	matrix::Vector2f point_B = point_C + point_B_direction * L_BC;

	// Find point A using circle intersections
	matrix::Vector2f origin = {0.0f, 0.0f};
	state.joint_B = point_B;

	matrix::Vector2f point_A = circle_intersection(origin, L_OA, point_B, L_AB, 0);

	if (!isfinite(point_A(0)) || !isfinite(point_A(1))) {
		return false;  // No valid solution
	}

	// Calculate final bucket angle ∠AOC
	state.bucket_angle = atan2f(point_A(1), point_A(0)) + _config.bucket_offset;
	state.coupler_angle = atan2f(point_B(1) - point_A(1), point_B(0) - point_A(0));

	return true;
}

float BucketKinematicsTilt::solve_inverse_trigonometric(float bucket_angle, float boom_angle) const
{
	// Inverse Stage 2: Given bucket angle, find required bellcrank angle

	// Remove bucket offset
	float target_angle = bucket_angle - _config.bucket_offset;

	// Calculate point A position from target angle
	matrix::Vector2f point_A = {
		_config.bucket_arm_length * cosf(target_angle),
		_config.bucket_arm_length * sinf(target_angle)
	};

	// Known point C position (along boom)
	matrix::Vector2f point_C = {_config.bucket_pivot.norm(), 0.0f};

	// Find point B using circle intersections
	matrix::Vector2f origin = {0.0f, 0.0f};
	matrix::Vector2f point_B = circle_intersection(point_A, _config.coupler_length,
	                                             point_C, _config.bellcrank_length, 0);

	if (!isfinite(point_B(0)) || !isfinite(point_B(1))) {
		return NAN;  // No valid solution
	}

	// Calculate bellcrank angle ∠OCB₂
	matrix::Vector2f CB_vector = point_B - point_C;
	return atan2f(CB_vector(1), CB_vector(0));
}

// =========================
// GEOMETRIC HELPER FUNCTIONS
// =========================

matrix::Vector2f BucketKinematicsTilt::circle_intersection(const matrix::Vector2f& c1, float r1,
                                                          const matrix::Vector2f& c2, float r2,
                                                          int solution_select) const
{
	// Find intersection points of two circles using analytical geometry
	matrix::Vector2f d = c2 - c1;
	float dist = d.norm();

	// Check if circles can intersect
	if (dist > r1 + r2 || dist < fabsf(r1 - r2) || dist < GEOMETRIC_TOLERANCE) {
		return matrix::Vector2f{NAN, NAN};  // No intersection or coincident circles
	}

	// Distance from c1 to radical axis
	float a = (r1 * r1 - r2 * r2 + dist * dist) / (2.0f * dist);

	// Distance from radical axis to intersection points
	float h_sq = r1 * r1 - a * a;
	if (h_sq < 0.0f) {
		return matrix::Vector2f{NAN, NAN};  // No real intersection
	}
	float h = sqrtf(h_sq);

	// Midpoint on radical axis
	matrix::Vector2f mid = c1 + d * (a / dist);

	// Perpendicular vector
	matrix::Vector2f perp = {-d(1), d(0)};  // Rotate d by 90°
	perp = perp * (h / dist);

	// Two intersection points
	if (solution_select == 0) {
		return mid + perp;
	} else {
		return mid - perp;
	}
}

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
	const float test_angles_s1[] = {
		0.0f,
		PI * 0.25f,
		PI * 0.5f,
		PI * 0.75f
	};

	for (float test_angle_s1 : test_angles_s1) {
		TiltState state = compute_forward_kinematics(test_angle_s1, 0.0f);
		if (!state.is_valid) {
			continue;  // Some angles may be out of range, that's OK
		}

		// Test inverse kinematics round-trip
		float inverse_angle_s2 = compute_inverse_kinematics(state.bucket_angle, 0.0f);
		if (!isfinite(inverse_angle_s2)) {
			PX4_ERR("Tilt inverse kinematics failed at test bucket angle: %.3f rad", (double)state.bucket_angle);
			return false;
		}

		// Check round-trip accuracy
		float angle_error = fabsf(inverse_angle_s2 - state.bellcrank_angle);
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
