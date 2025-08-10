/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the distribution in the
 *    documentation and/or other materials provided with the
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

#include "bucket_kinematics_drive.hpp"
#include <px4_platform_common/log.h>
#include <mathlib/mathlib.h>

BucketKinematicsDrive::BucketKinematicsDrive(ModuleParams *parent) :
	ModuleParams(parent)
{
	update_configuration();
}

void BucketKinematicsDrive::update_configuration()
{
	// Stage 1 - Bucket Actuation Linkage (Machine Body Frame)
	_config.actuator_base = matrix::Vector2f(_param_actuator_base_x.get(), _param_actuator_base_y.get());
	_config.bellcrank_boom = matrix::Vector2f(_param_bellcrank_boom_x.get(), _param_bellcrank_boom_y.get());

	// Link dimensions
	_config.bellcrank_length = _param_bellcrank_length.get();
	_config.actuator_offset = _param_actuator_offset.get();

	// Physical and safety limits
	_config.actuator_min_length = _param_actuator_min.get();
	_config.actuator_max_length = _param_actuator_max.get();
}

// =========================
// MAIN KINEMATIC FUNCTIONS
// =========================

BucketKinematicsDrive::DriveState BucketKinematicsDrive::compute_forward_kinematics(
	float actuator_length, float boom_angle) const
{
	DriveState state{};
	state.actuator_length = actuator_length;
	state.is_valid = false;

	// Solve Stage 1: Bucket Actuation Linkage (Machine Body Frame)
	if (!solve_trigonometric(actuator_length, boom_angle, state)) {
		PX4_WARN("Drive Stage 1 solution failed for actuator length: %.2f mm", (double)actuator_length);
		return state;
	}

	// Compute transmission angle and condition number
	state.transmission_angle = compute_transmission_angle(state);
	state.condition_number = compute_condition_number(state);

	// Final validation
	state.is_valid = check_mechanical_limits(state);

	return state;
}

float BucketKinematicsDrive::compute_inverse_kinematics(
	float bellcrank_angle, float boom_angle) const
{
	// Inverse Stage 1: Find required actuator length for bellcrank angle
	float required_actuator_length = solve_inverse_trigonometric(bellcrank_angle, boom_angle);
	if (required_actuator_length < 0.0f) {
		PX4_DEBUG("Drive Stage 1 inverse failed for bellcrank angle: %.3f rad", (double)bellcrank_angle);
		return -1.0f;
	}

	// Validate against actuator limits
	if (required_actuator_length < _config.actuator_min_length ||
	    required_actuator_length > _config.actuator_max_length) {
		PX4_DEBUG("Drive required actuator length %.2f mm outside limits [%.2f, %.2f]",
		          (double)required_actuator_length,
		          (double)_config.actuator_min_length,
		          (double)_config.actuator_max_length);
		return -1.0f;
	}

	return required_actuator_length;
}

// =========================
// TRIGONOMETRIC SOLUTION METHODS
// =========================

bool BucketKinematicsDrive::solve_trigonometric(float actuator_length, float boom_angle, DriveState& state) const
{
	// Stage 1: Bucket Actuation Linkage (OABC) - Machine Body Frame
	// O: Boom pivot (origin)
	// A: Actuator base
	// B: Actuator-bellcrank joint (moving)
	// C: Bellcrank-boom attachment

	// Transform actuator base to boom-relative coordinates
	float cos_boom = cosf(boom_angle);
	float sin_boom = sinf(boom_angle);

	matrix::Vector2f actuator_base_boom = {
		_config.actuator_base(0) * cos_boom + _config.actuator_base(1) * sin_boom,
		-_config.actuator_base(0) * sin_boom + _config.actuator_base(1) * cos_boom
	};

	// Link lengths for Stage 1
	float L_OA = actuator_base_boom.norm();  // Distance O to A
	float L_AB = actuator_length;            // Variable actuator length
	float L_BC = _config.bellcrank_length;   // Fixed bellcrank length
	float L_OC = calculate_boom_span(boom_angle);  // Boom span distance

	// Validate triangle inequality for triangle OAB
	if (!is_triangle_valid(L_OA, L_AB, L_OC - L_BC)) {
		return false;
	}

	// Method 1: Law of cosines approach
	// Step 1: Find diagonal OB using triangle OAB
	float angle_OAB = atan2f(actuator_base_boom(1), actuator_base_boom(0));
	float L_OB_sq = L_OA * L_OA + L_AB * L_AB - 2.0f * L_OA * L_AB * cosf(angle_OAB);
	float L_OB = sqrtf(L_OB_sq);

	// Step 2: Apply law of cosines in triangle OBC to find angle OCB
	float cos_OCB = (L_OC * L_OC + L_BC * L_BC - L_OB_sq) / (2.0f * L_OC * L_BC);

	if (fabsf(cos_OCB) > 1.0f) {
		return false;  // No valid solution
	}

	state.bellcrank_angle = acosf(cos_OCB);

	// Step 3: Find joint B position using circle intersection
	matrix::Vector2f center_A = actuator_base_boom;
	matrix::Vector2f center_C = _config.bellcrank_boom;

	state.joint_B = circle_intersection(center_A, L_AB, center_C, L_BC, 0);

	if (!isfinite(state.joint_B(0)) || !isfinite(state.joint_B(1))) {
		return false;  // No intersection found
	}

	return true;
}

float BucketKinematicsDrive::solve_inverse_trigonometric(float bellcrank_angle, float boom_angle) const
{
	// Inverse Stage 1: Given bellcrank angle, find required actuator length

	// Transform coordinates for boom angle
	float cos_boom = cosf(boom_angle);
	float sin_boom = sinf(boom_angle);

	matrix::Vector2f actuator_base_boom = {
		_config.actuator_base(0) * cos_boom + _config.actuator_base(1) * sin_boom,
		-_config.actuator_base(0) * sin_boom + _config.actuator_base(1) * cos_boom
	};

	// Find bellcrank joint B position from known bellcrank angle
	matrix::Vector2f bellcrank_pivot = _config.bellcrank_boom;
	matrix::Vector2f joint_B = bellcrank_pivot +
		matrix::Vector2f{cosf(bellcrank_angle), sinf(bellcrank_angle)} * _config.bellcrank_length;

	// Calculate required actuator length as distance from A to B
	matrix::Vector2f AB_vector = joint_B - actuator_base_boom;
	float required_length = AB_vector.norm();

	// Validate against actuator limits
	if (required_length < _config.actuator_min_length ||
	    required_length > _config.actuator_max_length) {
		return -1.0f;  // Invalid solution
	}

	return required_length;
}

// =========================
// GEOMETRIC HELPER FUNCTIONS
// =========================

matrix::Vector2f BucketKinematicsDrive::circle_intersection(const matrix::Vector2f& c1, float r1,
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

float BucketKinematicsDrive::law_of_cosines_angle(float side_a, float side_b, float side_c) const
{
	// Calculate angle C using law of cosines: cos(C) = (a²+b²-c²)/(2ab)
	if (side_a <= 0.0f || side_b <= 0.0f || side_c <= 0.0f) {
		return NAN;
	}

	float cos_C = (side_a * side_a + side_b * side_b - side_c * side_c) / (2.0f * side_a * side_b);

	if (fabsf(cos_C) > 1.0f) {
		return NAN;  // Invalid triangle
	}

	return acosf(cos_C);
}

float BucketKinematicsDrive::calculate_boom_span(float boom_angle) const
{
	// Calculate effective boom span distance for Stage 1 linkage
	// This accounts for boom geometry and rotation

	// For simplified model, use direct distance to bellcrank pivot
	// In practice, this would account for boom length and angle
	return _config.bellcrank_boom.norm();
}

bool BucketKinematicsDrive::is_triangle_valid(float a, float b, float c) const
{
	// Check triangle inequality: sum of any two sides > third side
	return (a + b > c) && (a + c > b) && (b + c > a) &&
	       (a > GEOMETRIC_TOLERANCE) && (b > GEOMETRIC_TOLERANCE) && (c > GEOMETRIC_TOLERANCE);
}

// =========================
// VALIDATION AND ANALYSIS
// =========================

bool BucketKinematicsDrive::check_mechanical_limits(const DriveState &state) const
{
	// Check actuator length limits
	if (state.actuator_length < _config.actuator_min_length ||
	    state.actuator_length > _config.actuator_max_length) {
		return false;
	}

	// Check transmission angle (avoid poor mechanical advantage)
	if (state.transmission_angle < MIN_TRANSMISSION_ANGLE ||
	    state.transmission_angle > (PI - MIN_TRANSMISSION_ANGLE)) {
		return false;
	}

	// Check condition number (avoid singularities)
	if (state.condition_number > MAX_CONDITION_NUMBER) {
		return false;
	}

	return true;
}

float BucketKinematicsDrive::compute_transmission_angle(const DriveState& state) const
{
	// Compute transmission angle for Stage 1 linkage
	// This is the angle between actuator and bellcrank at joint B

	if (!isfinite(state.joint_B(0)) || !isfinite(state.joint_B(1))) {
		return 0.0f;  // Invalid joint position
	}

	// Vectors from joint B to adjacent joints
	matrix::Vector2f BA = _config.actuator_base - state.joint_B;
	matrix::Vector2f BC = _config.bellcrank_boom - state.joint_B;

	// Normalize vectors
	BA = BA.normalized();
	BC = BC.normalized();

	// Compute angle between vectors
	float dot_product = BA.dot(BC);
	dot_product = math::constrain(dot_product, -1.0f, 1.0f);

	return acosf(fabsf(dot_product));  // Return acute angle
}

float BucketKinematicsDrive::compute_condition_number(const DriveState& state) const
{
	// Compute linkage condition number based on Jacobian
	matrix::Matrix<float, 2, 2> jacobian = compute_jacobian(state);

	// Calculate determinant for 2x2 matrix: det = ad - bc
	float det = jacobian(0, 0) * jacobian(1, 1) - jacobian(0, 1) * jacobian(1, 0);

	if (fabsf(det) < GEOMETRIC_TOLERANCE) {
		return MAX_CONDITION_NUMBER;  // Singular
	}

	// Calculate Frobenius norm
	float norm = 0.0f;
	for (int i = 0; i < 2; i++) {
		for (int j = 0; j < 2; j++) {
			norm += jacobian(i, j) * jacobian(i, j);
		}
	}
	norm = sqrtf(norm);

	return norm / fabsf(det);
}

matrix::Matrix<float, 2, 2> BucketKinematicsDrive::compute_jacobian(const DriveState &state) const
{
	matrix::Matrix<float, 2, 2> jacobian;

	// Analytical Jacobian computation for trigonometric solution
	// J = [∂bellcrank_angle/∂actuator_length,  ∂bellcrank_angle/∂boom_angle]
	//     [∂actuator_length/∂bellcrank_angle,  ∂actuator_length/∂boom_angle]

	// For analytical differentiation, we need partial derivatives of the trigonometric solution
	// This is complex, so we'll use numerical differentiation with small perturbation

	const float h = 1e-4f;  // Small perturbation for numerical differentiation

	// Partial derivatives with respect to actuator length
	DriveState state_plus = compute_forward_kinematics(state.actuator_length + h, 0.0f);
	DriveState state_minus = compute_forward_kinematics(state.actuator_length - h, 0.0f);

	if (state_plus.is_valid && state_minus.is_valid) {
		jacobian(0, 0) = (state_plus.bellcrank_angle - state_minus.bellcrank_angle) / (2.0f * h);
		jacobian(1, 0) = (state_plus.actuator_length - state_minus.actuator_length) / (2.0f * h);
	} else {
		jacobian(0, 0) = 0.0f;
		jacobian(1, 0) = 0.0f;
	}

	// Partial derivatives with respect to boom angle
	state_plus = compute_forward_kinematics(state.actuator_length, h);
	state_minus = compute_forward_kinematics(state.actuator_length, -h);

	if (state_plus.is_valid && state_minus.is_valid) {
		jacobian(0, 1) = (state_plus.bellcrank_angle - state_minus.bellcrank_angle) / (2.0f * h);
		jacobian(1, 1) = (state_plus.actuator_length - state_minus.actuator_length) / (2.0f * h);
	} else {
		jacobian(0, 1) = 0.0f;
		jacobian(1, 1) = 0.0f;
	}

	return jacobian;
}

bool BucketKinematicsDrive::validate_configuration() const
{
	// Check for positive link lengths
	if (_config.bellcrank_length <= 0.0f ||
	    _config.actuator_offset <= 0.0f) {
		PX4_ERR("Invalid drive linkage dimensions: all lengths must be positive");
		return false;
	}

	// Check actuator range
	if (_config.actuator_min_length >= _config.actuator_max_length) {
		PX4_ERR("Invalid actuator range: min (%.1f) >= max (%.1f)",
		        (double)_config.actuator_min_length, (double)_config.actuator_max_length);
		return false;
	}

	// Test trigonometric solutions at key points
	const float test_lengths[] = {
		_config.actuator_min_length,
		(_config.actuator_min_length + _config.actuator_max_length) * 0.5f,
		_config.actuator_max_length
	};

	for (float test_length : test_lengths) {
		DriveState state = compute_forward_kinematics(test_length, 0.0f);
		if (!state.is_valid) {
			PX4_ERR("Drive forward kinematics failed at test length: %.1f mm", (double)test_length);
			return false;
		}

		// Test inverse kinematics round-trip
		float inverse_length = compute_inverse_kinematics(state.bellcrank_angle, 0.0f);
		if (inverse_length < 0.0f) {
			PX4_ERR("Drive inverse kinematics failed at test angle: %.3f rad", (double)state.bellcrank_angle);
			return false;
		}

		// Check round-trip accuracy
		float length_error = fabsf(inverse_length - test_length);
		if (length_error > 1.0f) {  // 1mm tolerance
			PX4_ERR("Drive round-trip error too large: %.2f mm (tolerance: 1.0 mm)", (double)length_error);
			return false;
		}
	}

	PX4_INFO("Bucket drive kinematics configuration validated successfully");
	PX4_INFO("Actuator range: %.1f - %.1f mm",
	         (double)_config.actuator_min_length, (double)_config.actuator_max_length);

	return true;
}

float BucketKinematicsDrive::get_boom_compensation_factor(float boom_angle) const
{
	// Analytical boom compensation based on trigonometric linkage geometry
	// This factor converts boom angle changes to required actuator length changes

	// Compute current linkage state at mid-range actuator position
	float mid_actuator = (_config.actuator_min_length + _config.actuator_max_length) * 0.5f;
	DriveState current_state = compute_forward_kinematics(mid_actuator, boom_angle);

	if (!current_state.is_valid) {
		// Fallback to empirical formula
		const float BASE_FACTOR = 200.0f;  // mm/rad
		const float ANGLE_SENSITIVITY = 0.15f;
		return BASE_FACTOR * (1.0f + ANGLE_SENSITIVITY * fabsf(boom_angle));
	}

	// Use Jacobian to compute exact compensation factor
	matrix::Matrix<float, 2, 2> jacobian = compute_jacobian(current_state);

	// Compensation factor is ∂actuator_length/∂boom_angle for constant bellcrank angle
	// This requires inverting the relationship: if J[0,1] = ∂bellcrank/∂boom, then
	// compensation = -J[0,1] / J[0,0] (negative because we want to counteract boom effect)

	if (fabsf(jacobian(0, 0)) > GEOMETRIC_TOLERANCE) {
		float factor = -jacobian(0, 1) / jacobian(0, 0);
		return math::constrain(factor, 50.0f, 500.0f);  // Reasonable range
	}

	// Fallback value
	return 200.0f;  // mm/rad
}
