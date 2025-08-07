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

#pragma once

#include <px4_platform_common/module_params.h>
#include <mathlib/mathlib.h>

/**
 * @brief Boom kinematics calculator
 *
 * Handles geometric calculations for boom mechanism:
 * - Forward/inverse kinematics
 * - Actuator length to boom angle conversions
 * - Mechanical constraint checking
 */
class BoomKinematics : public ModuleParams
{
public:
	struct Configuration {
		float pivot_to_actuator_base;     // mm - Distance from pivot to actuator base
		float pivot_to_actuator_attach;   // mm - Distance from pivot to actuator attachment
		float actuator_base_angle;        // rad - Angle of actuator base from horizontal
		float actuator_min_length;        // mm - Minimum actuator extension
		float actuator_max_length;        // mm - Maximum actuator extension
		float boom_angle_min;             // rad - Minimum boom angle
		float boom_angle_max;             // rad - Maximum boom angle
	};

	struct KinematicState {
		float boom_angle;           // rad - Current boom angle
		float actuator_length;      // mm - Current actuator length
		float mechanical_advantage; // Ratio of actuator force to boom torque
		bool is_valid;             // True if within mechanical limits
	};

	explicit BoomKinematics(ModuleParams* parent);

	/**
	 * @brief Convert actuator length to boom angle
	 * @param actuator_length Actuator extension in mm
	 * @return Boom angle in radians
	 */
	float actuator_to_boom_angle(float actuator_length) const;

	/**
	 * @brief Convert boom angle to required actuator length
	 * @param boom_angle Desired boom angle in radians
	 * @return Required actuator length in mm
	 */
	float boom_angle_to_actuator(float boom_angle) const;

	/**
	 * @brief Convert AS5600 encoder angle to actuator length
	 * @param encoder_angle Raw encoder angle in degrees
	 * @return Actuator length in mm
	 */
	float encoder_to_actuator_length(float encoder_angle) const;

	/**
	 * @brief Calculate mechanical advantage at current position
	 * @param actuator_length Current actuator length in mm
	 * @return Mechanical advantage ratio
	 */
	float calculate_mechanical_advantage(float actuator_length) const;

	/**
	 * @brief Check if position is within mechanical limits
	 * @param boom_angle Boom angle to check in radians
	 * @return True if position is valid
	 */
	bool is_position_valid(float boom_angle) const;

	/**
	 * @brief Update configuration from parameters
	 */
	void update_configuration();

	/**
	 * @brief Validate kinematic configuration
	 * @return True if configuration is valid
	 */
	bool validate_configuration() const;

	const Configuration& get_configuration() const { return _config; }

private:
	Configuration _config;

	// Kinematic parameters
	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::BOOM_KIN_P_BASE>) _param_pivot_to_base,
		(ParamFloat<px4::params::BOOM_KIN_P_ATT>) _param_pivot_to_attach,
		(ParamFloat<px4::params::BOOM_KIN_B_ANG>) _param_base_angle,
		(ParamFloat<px4::params::BOOM_ACT_MIN>) _param_actuator_min,
		(ParamFloat<px4::params::BOOM_ACT_MAX>) _param_actuator_max,
		(ParamFloat<px4::params::BOOM_ANG_MIN>) _param_angle_min,
		(ParamFloat<px4::params::BOOM_ANG_MAX>) _param_angle_max,
		(ParamFloat<px4::params::BOOM_ENC_SCALE>) _param_encoder_scale,
		(ParamFloat<px4::params::BOOM_ENC_OFF>) _param_encoder_offset
	)

	/**
	 * @brief Calculate angle using law of cosines
	 * @param a First side length
	 * @param b Second side length
	 * @param c Third side length (opposite to angle)
	 * @return Angle in radians
	 */
	float law_of_cosines_angle(float a, float b, float c) const;

	/**
	 * @brief Calculate side length using law of cosines
	 * @param a First side length
	 * @param b Second side length
	 * @param gamma_angle Angle between sides a and b in radians
	 * @return Third side length
	 */
	float law_of_cosines_side(float a, float b, float gamma_angle) const;
};
