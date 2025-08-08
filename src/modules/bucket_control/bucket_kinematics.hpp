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

#pragma once

// System includes
#include <px4_platform_common/module_params.h>
#include <matrix/matrix/math.hpp>

// Standard library
#include <cmath>

/**
 * @brief Handles all bucket linkage kinematic calculations
 *
 * This class encapsulates the geometric calculations for the bucket
 * four-bar linkage mechanism, providing forward and inverse kinematics
 * along with mechanical validation and sensitivity analysis.
 */
/**
 * @brief Bucket kinematics solver for wheel loader dual-linkage mechanism
 *
 * This class handles the kinematic calculations for the wheel loader bucket system,
 * which consists of two mechanically coupled four-bar linkages:
 *
 * STAGE 1: BUCKET ACTUATION LINKAGE (Primary - OABC mechanism):
 *    - Coordinate System: Machine body frame (X=Forward, Y=Left, Z=Up)
 *    - O: Boom pivot point (origin, fixed to machine body)
 *    - A: Linear actuator base attachment
 *    - B: Actuator-bellcrank joint (moving)
 *    - C: Bellcrank-boom attachment (moves with boom)
 *    - Function: Converts actuator length + boom angle → bellcrank angle ∠OCB₁
 *
 * STAGE 2: BUCKET TILT LINKAGE (Secondary - OABC mechanism):
 *    - Coordinate System: Boom-end frame (X=Boom-forward, Y=Left, Z=Up)
 *    - O: Reference point at boom end
 *    - A: Bucket attachment point (fixed to bucket)
 *    - B: Coupler joint (moving)
 *    - C: Bucket point (final output)
 *    - Function: Converts bellcrank angle ∠OCB₂ → final bucket angle ∠AOC
 *
 * MECHANICAL COUPLING: ∠OCB₁ = ∠OCB₂ + α (fixed angular offset)
 *
 * SOLUTION METHOD: Analytical trigonometric approach using:
 *    - Law of cosines for link constraints
 *    - Circle intersections for joint positions
 *    - Direct atan2() calculations for angles
 *    - O(1) constant time complexity
 */
class BucketKinematics : public ModuleParams
{
public:
	/**
	 * @brief Kinematic configuration structure
	 */
	struct Configuration {
		// Attachment points (mm, relative to boom pivot)
		matrix::Vector2f actuator_base;
		matrix::Vector2f bellcrank_boom;
		matrix::Vector2f bucket_pivot;

		// Linkage dimensions (mm)
		float bellcrank_length;
		float coupler_length;
		float actuator_offset;
		float bucket_arm_length;

		// Angular constraints (rad)
		float bellcrank_internal_angle;
		float bucket_offset;

		// Physical limits
		float actuator_min_length;
		float actuator_max_length;
		float bucket_angle_min;
		float bucket_angle_max;
	};

	/**
	 * @brief Complete linkage state description
	 */
	struct LinkageState {
		// Primary outputs
		float actuator_length;     // mm - linear actuator extension
		float bucket_angle;        // rad - final bucket angle (ground-relative)
		float bellcrank_angle_s1;  // rad - Stage 1 bellcrank angle ∠OCB₁
		float bellcrank_angle_s2;  // rad - Stage 2 bellcrank angle ∠OCB₂

		// Secondary outputs
		float coupler_angle;       // rad - coupler link angle
		float transmission_angle;  // rad - linkage transmission angle

		// Validation
		bool is_valid;            // True if within mechanical limits
		float condition_number;   // Linkage condition (singularity detection)

		// Joint positions for visualization/debugging
		matrix::Vector2f joint_B_s1;  // Stage 1 joint B position
		matrix::Vector2f joint_B_s2;  // Stage 2 joint B position
	};

	explicit BucketKinematics(ModuleParams *parent);

	/**
	 * @brief Forward kinematics: actuator length -> bucket angle
	 * Uses analytical trigonometric solution (O(1) complexity)
	 * @param actuator_length Actuator extension length (mm)
	 * @param boom_angle Current boom angle (rad)
	 * @return Complete linkage state
	 */
	LinkageState compute_forward_kinematics(float actuator_length, float boom_angle = 0.0f) const;

	/**
	 * @brief Inverse kinematics: bucket angle -> actuator length
	 * Uses analytical trigonometric solution (O(1) complexity)
	 * @param bucket_angle Desired bucket angle (rad, ground-relative)
	 * @param boom_angle Current boom angle (rad)
	 * @return Required linkage state to achieve bucket angle
	 */
	LinkageState compute_inverse_kinematics(float bucket_angle, float boom_angle = 0.0f) const;

	/**
	 * @brief Validate kinematic configuration
	 * @return True if configuration is mechanically sound
	 */
	bool validate_configuration() const;

	/**
	 * @brief Get Jacobian matrix for sensitivity analysis
	 * @param state Current linkage state
	 * @return 2x2 Jacobian [dangle/dlength, dangle/dboom]
	 */
	matrix::Matrix<float, 2, 2> compute_jacobian(const LinkageState& state) const;

	/**
	 * @brief Update configuration from parameters
	 */
	void update_configuration();

	/**
	 * @brief Get current configuration
	 */
	const Configuration& get_configuration() const { return _config; }

	/**
	 * @brief Get boom compensation factor
	 * @param boom_angle Current boom angle (rad)
	 * @return Compensation factor (mm/rad)
	 */
	float get_boom_compensation_factor(float boom_angle) const;

private:
	Configuration _config;

	// =========================
	// TRIGONOMETRIC SOLUTION METHODS
	// =========================

	/**
	 * @brief Stage 1: Solve Bucket Actuation Linkage using law of cosines
	 * Machine body frame: Actuator + boom angle → bellcrank angle
	 * @param actuator_length Current actuator length (mm)
	 * @param boom_angle Current boom angle (rad)
	 * @param state Output linkage state
	 * @return True if valid solution found
	 */
	bool solve_stage1_trigonometric(float actuator_length, float boom_angle, LinkageState& state) const;

	/**
	 * @brief Stage 2: Solve Bucket Tilt Linkage using coordinate geometry
	 * Boom-end frame: Bellcrank angle → final bucket angle
	 * @param state Linkage state with bellcrank angle from Stage 1
	 * @param boom_angle Current boom angle (rad)
	 * @return True if valid solution found
	 */
	bool solve_stage2_trigonometric(LinkageState& state, float boom_angle) const;

	/**
	 * @brief Inverse Stage 1: Required actuator length for bellcrank angle
	 * @param bellcrank_angle Required bellcrank angle (rad)
	 * @param boom_angle Current boom angle (rad)
	 * @return Required actuator length, or -1 if no valid solution
	 */
	float solve_stage1_inverse_trigonometric(float bellcrank_angle, float boom_angle) const;

	/**
	 * @brief Inverse Stage 2: Required bellcrank angle for bucket angle
	 * @param bucket_angle Desired bucket angle (rad)
	 * @param boom_angle Current boom angle (rad)
	 * @return Required bellcrank angle, or NaN if no valid solution
	 */
	float solve_stage2_inverse_trigonometric(float bucket_angle, float boom_angle) const;

	// =========================
	// GEOMETRIC HELPER FUNCTIONS
	// =========================

	/**
	 * @brief Find intersection points of two circles
	 * @param c1 Center of circle 1
	 * @param r1 Radius of circle 1
	 * @param c2 Center of circle 2
	 * @param r2 Radius of circle 2
	 * @param solution_select 0=first solution, 1=second solution
	 * @return Intersection point, or {NaN, NaN} if no intersection
	 */
	matrix::Vector2f circle_intersection(const matrix::Vector2f& c1, float r1,
	                                   const matrix::Vector2f& c2, float r2,
	                                   int solution_select = 0) const;

	/**
	 * @brief Calculate angle using law of cosines: cos(C) = (a²+b²-c²)/(2ab)
	 * @param side_a Length of side a
	 * @param side_b Length of side b
	 * @param side_c Length of side c (opposite to angle C)
	 * @return Angle C in radians, or NaN if triangle impossible
	 */
	float law_of_cosines_angle(float side_a, float side_b, float side_c) const;

	/**
	 * @brief Calculate boom geometry parameters based on boom angle
	 * @param boom_angle Current boom angle (rad)
	 * @return Effective boom span distance for Stage 1
	 */
	float calculate_boom_span(float boom_angle) const;

	/**
	 * @brief Apply mechanical coupling between Stage 1 and Stage 2
	 * @param bellcrank_angle_s1 Stage 1 bellcrank angle
	 * @return Stage 2 bellcrank angle with coupling offset
	 */
	float apply_stage_coupling(float bellcrank_angle_s1) const;

	// =========================
	// VALIDATION AND ANALYSIS
	// =========================

	// =========================
	// VALIDATION AND ANALYSIS
	// =========================

	/**
	 * @brief Check mechanical limits and validate linkage state
	 * @param state Linkage state to validate
	 * @return True if state is mechanically valid and within limits
	 */
	bool check_mechanical_limits(const LinkageState& state) const;

	/**
	 * @brief Compute transmission angle for singularity detection
	 * @param state Current linkage state
	 * @return Transmission angle in radians (closer to 0° or 180° = worse)
	 */
	float compute_transmission_angle(const LinkageState& state) const;

	/**
	 * @brief Compute linkage condition number for sensitivity analysis
	 * @param state Current linkage state
	 * @return Condition number (higher = closer to singularity)
	 */
	float compute_condition_number(const LinkageState& state) const;

	/**
	 * @brief Validate that triangle with given side lengths is possible
	 * @param a Side length a
	 * @param b Side length b
	 * @param c Side length c
	 * @return True if triangle inequality satisfied
	 */
	bool is_triangle_valid(float a, float b, float c) const;

	// Kinematic parameters (following 16-char limit)
	DEFINE_PARAMETERS(
		// Stage 1 - Bucket Actuation Linkage (Machine Body Frame)
		(ParamFloat<px4::params::BCT_ACT_BASE_X>) _param_actuator_base_x,
		(ParamFloat<px4::params::BCT_ACT_BASE_Y>) _param_actuator_base_y,
		(ParamFloat<px4::params::BCT_BCK_BOOM_X>) _param_bellcrank_boom_x,
		(ParamFloat<px4::params::BCT_BCK_BOOM_Y>) _param_bellcrank_boom_y,

		// Stage 2 - Bucket Tilt Linkage (Boom-End Frame)
		(ParamFloat<px4::params::BCT_BKT_PIV_X>) _param_bucket_pivot_x,
		(ParamFloat<px4::params::BCT_BKT_PIV_Y>) _param_bucket_pivot_y,

		// Link dimensions (both stages)
		(ParamFloat<px4::params::BCT_BCK_LEN>) _param_bellcrank_length,
		(ParamFloat<px4::params::BCT_COUP_LEN>) _param_coupler_length,
		(ParamFloat<px4::params::BCT_ACT_OFF>) _param_actuator_offset,
		(ParamFloat<px4::params::BCT_BKT_ARM_LEN>) _param_bucket_arm_length,

		// Mechanical coupling and offsets
		(ParamFloat<px4::params::BCT_BCK_INT_ANG>) _param_bellcrank_internal_angle,
		(ParamFloat<px4::params::BCT_BKT_OFF>) _param_bucket_offset,

		// Physical and safety limits
		(ParamFloat<px4::params::BCT_ACT_MIN>) _param_actuator_min,
		(ParamFloat<px4::params::BCT_ACT_MAX>) _param_actuator_max,
		(ParamFloat<px4::params::BCT_ANG_MIN>) _param_bucket_angle_min,
		(ParamFloat<px4::params::BCT_ANG_MAX>) _param_bucket_angle_max
	)

	// Computation constants for trigonometric solutions
	static constexpr float GEOMETRIC_TOLERANCE = 1e-6f;      // Geometric precision
	static constexpr float MIN_TRANSMISSION_ANGLE = 0.174f;  // 10° in radians
	static constexpr float MAX_CONDITION_NUMBER = 100.0f;     // Singularity threshold
	static constexpr float PI = 3.14159265359f;              // Mathematical constant
};
