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
		float actuator_length;    // mm
		float bucket_angle;       // rad (ground-relative)
		float bellcrank_angle;    // rad
		float coupler_angle;      // rad
		bool is_valid;           // True if within mechanical limits
		float condition_number;  // Linkage condition for singularity detection
	};

	explicit BucketKinematics(ModuleParams *parent);

	/**
	 * @brief Forward kinematics: actuator length -> bucket angle
	 * @param actuator_length Actuator extension length (mm)
	 * @param boom_angle Current boom angle (rad)
	 * @return Complete linkage state
	 */
	LinkageState compute_forward_kinematics(float actuator_length, float boom_angle = 0.0f) const;

	/**
	 * @brief Inverse kinematics: bucket angle -> actuator length
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

	/**
	 * @brief Solve four-bar linkage using iterative method
	 * @param actuator_length Current actuator length (mm)
	 * @param boom_angle Current boom angle (rad)
	 * @param state Output linkage state
	 * @return True if solution found
	 */
	bool solve_linkage(float actuator_length, float boom_angle, LinkageState& state) const;

	/**
	 * @brief Check for singularities and mechanical limits
	 * @param state Linkage state to check
	 * @return True if state is mechanically valid
	 */
	bool check_mechanical_limits(const LinkageState& state) const;

	/**
	 * @brief Compute linkage condition number
	 * @param state Current linkage state
	 * @return Condition number (higher = closer to singularity)
	 */
	float compute_condition_number(const LinkageState& state) const;

	// Kinematic parameters (following 16-char limit)
	DEFINE_PARAMETERS(
		// Attachment points
		(ParamFloat<px4::params::BCT_ACT_BASE_X>) _param_actuator_base_x,
		(ParamFloat<px4::params::BCT_ACT_BASE_Y>) _param_actuator_base_y,
		(ParamFloat<px4::params::BCT_BCK_BOOM_X>) _param_bellcrank_boom_x,
		(ParamFloat<px4::params::BCT_BCK_BOOM_Y>) _param_bellcrank_boom_y,
		(ParamFloat<px4::params::BCT_BKT_PIV_X>) _param_bucket_pivot_x,
		(ParamFloat<px4::params::BCT_BKT_PIV_Y>) _param_bucket_pivot_y,

		// Linkage dimensions
		(ParamFloat<px4::params::BCT_BCK_LEN>) _param_bellcrank_length,
		(ParamFloat<px4::params::BCT_COUP_LEN>) _param_coupler_length,
		(ParamFloat<px4::params::BCT_ACT_OFF>) _param_actuator_offset,
		(ParamFloat<px4::params::BCT_BKT_ARM_LEN>) _param_bucket_arm_length,

		// Angular constraints
		(ParamFloat<px4::params::BCT_BCK_INT_ANG>) _param_bellcrank_internal_angle,
		(ParamFloat<px4::params::BCT_BKT_OFF>) _param_bucket_offset,

		// Physical limits
		(ParamFloat<px4::params::BCT_ACT_MIN>) _param_actuator_min,
		(ParamFloat<px4::params::BCT_ACT_MAX>) _param_actuator_max,
		(ParamFloat<px4::params::BCT_ANG_MIN>) _param_bucket_angle_min,
		(ParamFloat<px4::params::BCT_ANG_MAX>) _param_bucket_angle_max
	)

	// Computation constants
	static constexpr float CONVERGENCE_TOL = 1e-6f;
	static constexpr int MAX_ITERATIONS = 50;
	static constexpr float SINGULARITY_THRESHOLD = 100.0f;
};
