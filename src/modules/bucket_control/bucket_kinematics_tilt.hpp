/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in so	// =========================
	// VALIDATION AND ANALYSIS
	// =========================forms, with or without
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
 * @brief Bucket tilt kinematics solver for wheel loader Stage 2 tilt linkage
 *
 * This class handles the kinematic calculations for Stage 2 of the wheel loader bucket system:
 *
 * STAGE 2: BUCKET TILT LINKAGE (Secondary - OABC mechanism):
 *    - Coordinate System: Boom-end frame (X=Boom-forward, Y=Left, Z=Up)
 *    - O: Reference point at boom end
 *    - A: Bucket attachment point (fixed to bucket)
 *    - B: Coupler joint (moving)
 *    - C: Bucket point (final output)
 *    - Function: Converts bellcrank angle ∠OCB₂ → final bucket angle ∠AOC
 *
 * MECHANICAL COUPLING: ∠OCB₁ = ∠OCB₂ + α (fixed angular offset from Stage 1)
 *
 * SOLUTION METHOD: Analytical trigonometric approach using:
 *    - Coordinate geometry for link constraints
 *    - Circle intersections for joint positions
 *    - Direct atan2() calculations for angles
 *    - O(1) constant time complexity
 */
class BucketKinematicsTilt : public ModuleParams
{
public:
	/**
	 * @brief Tilt linkage configuration structure
	 */
	struct TiltConfiguration {
		// Attachment points (mm, relative to boom pivot)
		matrix::Vector2f bucket_pivot;

		// Linkage dimensions (mm)
		float bellcrank_length;
		float coupler_length;
		float bucket_arm_length;

		// Angular constraints (rad)
		float bellcrank_internal_angle;
		float bucket_offset;

		// Physical limits
		float bucket_angle_min;
		float bucket_angle_max;
	};

	/**
	 * @brief Tilt linkage state description
	 */
	struct TiltState {
		// Primary outputs
		float bucket_angle;        // rad - final bucket angle (ground-relative)
		float bellcrank_angle;     // rad - Stage 2 bellcrank angle ∠OCB₂

		// Secondary outputs
		float coupler_angle;       // rad - coupler link angle

		// Validation
		bool is_valid;            // True if within mechanical limits

		// Joint positions for visualization/debugging
		matrix::Vector2f joint_B;  // Stage 2 joint B position
	};

	explicit BucketKinematicsTilt(ModuleParams *parent);

	/**
	 * @brief Forward kinematics: bellcrank angle -> bucket angle
	 * Uses analytical trigonometric solution (O(1) complexity)
	 * @param bellcrank_angle_drive Stage 1 bellcrank angle (from drive stage)
	 * @param boom_angle Current boom angle (rad)
	 * @return Tilt linkage state
	 */
	TiltState compute_forward_kinematics(float bellcrank_angle_drive, float boom_angle = 0.0f) const;

	/**
	 * @brief Inverse kinematics: bucket angle -> bellcrank angle
	 * Uses analytical trigonometric solution (O(1) complexity)
	 * @param bucket_angle Desired bucket angle (rad, ground-relative)
	 * @param boom_angle Current boom angle (rad)
	 * @return Required Stage 2 bellcrank angle, or NaN if no valid solution
	 */
	float compute_inverse_kinematics(float bucket_angle, float boom_angle = 0.0f) const;

	/**
	 * @brief Validate tilt configuration
	 * @return True if configuration is mechanically sound
	 */
	bool validate_configuration() const;

	/**
	 * @brief Update configuration from parameters
	 */
	void update_configuration();

	/**
	 * @brief Get current configuration
	 */
	const TiltConfiguration& get_configuration() const { return _config; }

	/**
	 * @brief Apply mechanical coupling from Stage 1 to Stage 2
	 * @param bellcrank_angle_drive Stage 1 bellcrank angle
	 * @return Stage 2 bellcrank angle with coupling offset
	 */
	float apply_stage_coupling(float bellcrank_angle_drive) const;

	/**
	 * @brief Get Stage 1 angle from Stage 2 angle (inverse coupling)
	 * @param bellcrank_angle_tilt Stage 2 bellcrank angle
	 * @return Stage 1 bellcrank angle
	 */
	float remove_stage_coupling(float bellcrank_angle_tilt) const;

private:
	TiltConfiguration _config;

	// =========================
	// TRIGONOMETRIC SOLUTION METHODS
	// =========================

	/**
	 * @brief Stage 2 forward trigonometric solution
	 * @param bellcrank_angle_tilt Stage 2 bellcrank angle
	 * @param boom_angle Current boom angle (rad)
	 * @param state Output tilt state
	 * @return True if valid solution found
	 */
	bool solve_trigonometric(float bellcrank_angle_tilt, float boom_angle, TiltState& state) const;

	/**
	 * @brief Inverse Stage 2: Required bellcrank angle for bucket angle
	 * @param bucket_angle Desired bucket angle (rad)
	 * @param boom_angle Current boom angle (rad)
	 * @return Required bellcrank angle, or NaN if no valid solution
	 */
	float solve_inverse_trigonometric(float bucket_angle, float boom_angle) const;

	// =========================
	// VALIDATION AND ANALYSIS
	// =========================

	/**
	 * @brief Check mechanical limits and validate tilt state
	 * @param state Tilt state to validate
	 * @return True if state is mechanically valid and within limits
	 */
	bool check_mechanical_limits(const TiltState& state) const;

	/**
	 * @brief Validate that triangle with given side lengths is possible
	 * @param a Side length a
	 * @param b Side length b
	 * @param c Side length c
	 * @return True if triangle inequality satisfied
	 */
	bool is_triangle_valid(float a, float b, float c) const;

	// Tilt kinematic parameters (following 16-char limit)
	DEFINE_PARAMETERS(
		// Stage 2 - Bucket Tilt Linkage (Boom-End Frame)
		(ParamFloat<px4::params::BCT_BKT_PIV_X>) _param_bucket_pivot_x,
		(ParamFloat<px4::params::BCT_BKT_PIV_Y>) _param_bucket_pivot_y,

		// Link dimensions
		(ParamFloat<px4::params::BCT_BCK_LEN>) _param_bellcrank_length,
		(ParamFloat<px4::params::BCT_COUP_LEN>) _param_coupler_length,
		(ParamFloat<px4::params::BCT_BKT_ARM_LEN>) _param_bucket_arm_length,

		// Mechanical coupling and offsets
		(ParamFloat<px4::params::BCT_BCK_INT_ANG>) _param_bellcrank_internal_angle,
		(ParamFloat<px4::params::BCT_BKT_OFF>) _param_bucket_offset,

		// Physical and safety limits
		(ParamFloat<px4::params::BCT_ANG_MIN>) _param_bucket_angle_min,
		(ParamFloat<px4::params::BCT_ANG_MAX>) _param_bucket_angle_max
	)

	// Computation constants for trigonometric solutions
	static constexpr float GEOMETRIC_TOLERANCE = 1e-6f;      // Geometric precision
	static constexpr float PI = 3.14159265359f;              // Mathematical constant
};
