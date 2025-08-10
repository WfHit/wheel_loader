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

#pragma once

// Local includes
#include "bucket_kinematics_drive.hpp"
#include "bucket_kinematics_tilt.hpp"

// System includes
#include <px4_platform_common/module_params.h>
#include <matrix/matrix/math.hpp>

// Standard library
#include <cmath>

/**
 * @brief Bucket kinematics coordinator for wheel loader dual-linkage mechanism
 *
 * This class coordinates the kinematic calculations for the wheel loader bucket system,
 * which consists of two mechanically coupled four-bar linkages managed by separate classes:
 *
 * STAGE 1: BUCKET ACTUATION LINKAGE (Primary - handled by BucketKinematicsDrive):
 *    - Coordinate System: Machine body frame (X=Forward, Y=Left, Z=Up)
 *    - Function: Converts actuator length + boom angle → bellcrank angle ∠OCB₁
 *
 * STAGE 2: BUCKET TILT LINKAGE (Secondary - handled by BucketKinematicsTilt):
 *    - Coordinate System: Boom-end frame (X=Boom-forward, Y=Left, Z=Up)
 *    - Function: Converts bellcrank angle ∠OCB₂ → final bucket angle ∠AOC
 *
 * MECHANICAL COUPLING: ∠OCB₁ = ∠OCB₂ + α (handled by tilt class)
 *
 * This coordinator provides the same interface as the original BucketKinematics class
 * but delegates the actual calculations to the specialized drive and tilt classes.
 */
class BucketKinematics : public ModuleParams
{
public:
	/**
	 * @brief Complete linkage state description (maintains backward compatibility)
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
	 * @brief Get boom compensation factor
	 * @param boom_angle Current boom angle (rad)
	 * @return Compensation factor (mm/rad)
	 */
	float get_boom_compensation_factor(float boom_angle) const;

	/**
	 * @brief Get access to drive kinematics component
	 */
	const BucketKinematicsDrive& get_drive_kinematics() const { return _drive_kinematics; }

	/**
	 * @brief Get access to tilt kinematics component
	 */
	const BucketKinematicsTilt& get_tilt_kinematics() const { return _tilt_kinematics; }

private:
	BucketKinematicsDrive _drive_kinematics;
	BucketKinematicsTilt _tilt_kinematics;

	/**
	 * @brief Convert drive state and tilt state to combined linkage state
	 * @param drive_state Drive linkage state
	 * @param tilt_state Tilt linkage state
	 * @return Combined linkage state
	 */
	LinkageState combine_states(const BucketKinematicsDrive::DriveState& drive_state,
	                           const BucketKinematicsTilt::TiltState& tilt_state) const;
};
