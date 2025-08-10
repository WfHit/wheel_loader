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

#pragma once

// System includes
#include <px4_platform_common/module_params.h>
#include <matrix/matrix/math.hpp>

// Standard library
#include <cmath>

/**
 * @brief Bucket drive kinematics solver for wheel loader Stage 1 actuation linkage
 *
 * This class handles the kinematic calculations for Stage 1 of the wheel loader bucket system:
 *
 * STAGE 1: BUCKET ACTUATION LINKAGE (Primary - OABC mechanism):
 *    - Coordinate System: Machine body frame (X=Forward, Y=Left, Z=Up)
 *    - O: Boom pivot point (origin, fixed to machine body)
 *    - A: Linear actuator base attachment
 *    - B: Actuator-bellcrank joint (moving)
 *    - C: Bellcrank-boom attachment (moves with boom)
 *    - Function: Converts actuator length + boom angle → bellcrank angle ∠OCB₁
 *
 * SOLUTION METHOD: Analytical trigonometric approach using:
 *    - Law of cosines for link constraints
 *    - Circle intersections for joint positions
 *    - Direct atan2() calculations for angles
 *    - O(1) constant time complexity
 */
class BucketKinematicsDrive : public ModuleParams
{
public:
	/**
	 * @brief Drive linkage configuration structure
	 */
	struct DriveConfiguration {
		// Attachment points (mm, relative to boom pivot)
		matrix::Vector2f actuator_base;
		matrix::Vector2f bellcrank_boom;

		// Linkage dimensions (mm)
		float bellcrank_length;
		float actuator_offset;

		// Physical limits
		float actuator_min_length;
		float actuator_max_length;
	};

	/**
	 * @brief Drive linkage state description
	 */
	struct DriveState {
		// Primary outputs
		float actuator_length;     // mm - linear actuator extension
		float bellcrank_angle;     // rad - Stage 1 bellcrank angle ∠OCB₁

		// Secondary outputs
		float transmission_angle;  // rad - linkage transmission angle

		// Validation
		bool is_valid;            // True if within mechanical limits
		float condition_number;   // Linkage condition (singularity detection)

		// Joint positions for visualization/debugging
		matrix::Vector2f joint_B;  // Stage 1 joint B position
	};

	explicit BucketKinematicsDrive(ModuleParams *parent);

	/**
	 * @brief Forward kinematics: actuator length -> bellcrank angle
	 * Uses analytical trigonometric solution (O(1) complexity)
	 * @param actuator_length Actuator extension length (mm)
	 * @param boom_angle Current boom angle (rad)
	 * @return Drive linkage state
	 */
	DriveState compute_forward_kinematics(float actuator_length, float boom_angle = 0.0f) const;

	/**
	 * @brief Inverse kinematics: bellcrank angle -> actuator length
	 * Uses analytical trigonometric solution (O(1) complexity)
	 * @param bellcrank_angle Desired bellcrank angle (rad)
	 * @param boom_angle Current boom angle (rad)
	 * @return Required actuator length, or -1 if no valid solution
	 */
	float compute_inverse_kinematics(float bellcrank_angle, float boom_angle = 0.0f) const;

	/**
	 * @brief Validate drive configuration
	 * @return True if configuration is mechanically sound
	 */
	bool validate_configuration() const;

	/**
	 * @brief Get Jacobian matrix for sensitivity analysis
	 * @param state Current drive state
	 * @return 2x2 Jacobian [dbellcrank/dlength, dbellcrank/dboom]
	 */
	matrix::Matrix<float, 2, 2> compute_jacobian(const DriveState& state) const;

	/**
	 * @brief Update configuration from parameters
	 */
	void update_configuration();

	/**
	 * @brief Get current configuration
	 */
	const DriveConfiguration& get_configuration() const { return _config; }

	/**
	 * @brief Get boom compensation factor
	 * @param boom_angle Current boom angle (rad)
	 * @return Compensation factor (mm/rad)
	 */
	float get_boom_compensation_factor(float boom_angle) const;

private:
	DriveConfiguration _config;

	// =========================
	// TRIGONOMETRIC SOLUTION METHODS
	// =========================

	/**
	 * @brief Solve Stage 1 linkage using law of cosines
	 * Machine body frame: Actuator + boom angle → bellcrank angle
	 * @param actuator_length Current actuator length (mm)
	 * @param boom_angle Current boom angle (rad)
	 * @param state Output drive state
	 * @return True if valid solution found
	 */
	bool solve_trigonometric(float actuator_length, float boom_angle, DriveState& state) const;

	/**
	 * @brief Inverse solve: Required actuator length for bellcrank angle
	 * @param bellcrank_angle Required bellcrank angle (rad)
	 * @param boom_angle Current boom angle (rad)
	 * @return Required actuator length, or -1 if no valid solution
	 */
	float solve_inverse_trigonometric(float bellcrank_angle, float boom_angle) const;

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

	// =========================
	// VALIDATION AND ANALYSIS
	// =========================

	/**
	 * @brief Check mechanical limits and validate drive state
	 * @param state Drive state to validate
	 * @return True if state is mechanically valid and within limits
	 */
	bool check_mechanical_limits(const DriveState& state) const;

	/**
	 * @brief Compute transmission angle for singularity detection
	 * @param state Current drive state
	 * @return Transmission angle in radians (closer to 0° or 180° = worse)
	 */
	float compute_transmission_angle(const DriveState& state) const;

	/**
	 * @brief Compute drive condition number for sensitivity analysis
	 * @param state Current drive state
	 * @return Condition number (higher = closer to singularity)
	 */
	float compute_condition_number(const DriveState& state) const;

	/**
	 * @brief Validate that triangle with given side lengths is possible
	 * @param a Side length a
	 * @param b Side length b
	 * @param c Side length c
	 * @return True if triangle inequality satisfied
	 */
	bool is_triangle_valid(float a, float b, float c) const;

	// Drive kinematic parameters (following 16-char limit)
	DEFINE_PARAMETERS(
		// Stage 1 - Bucket Actuation Linkage (Machine Body Frame)
		(ParamFloat<px4::params::BCT_ACT_BASE_X>) _param_actuator_base_x,
		(ParamFloat<px4::params::BCT_ACT_BASE_Y>) _param_actuator_base_y,
		(ParamFloat<px4::params::BCT_BCK_BOOM_X>) _param_bellcrank_boom_x,
		(ParamFloat<px4::params::BCT_BCK_BOOM_Y>) _param_bellcrank_boom_y,

		// Link dimensions
		(ParamFloat<px4::params::BCT_BCK_LEN>) _param_bellcrank_length,
		(ParamFloat<px4::params::BCT_ACT_OFF>) _param_actuator_offset,

		// Physical and safety limits
		(ParamFloat<px4::params::BCT_ACT_MIN>) _param_actuator_min,
		(ParamFloat<px4::params::BCT_ACT_MAX>) _param_actuator_max
	)

	// Computation constants for trigonometric solutions
	static constexpr float GEOMETRIC_TOLERANCE = 1e-6f;      // Geometric precision
	static constexpr float MIN_TRANSMISSION_ANGLE = 0.174f;  // 10° in radians
	static constexpr float MAX_CONDITION_NUMBER = 100.0f;     // Singularity threshold
	static constexpr float PI = 3.14159265359f;              // Mathematical constant
};
