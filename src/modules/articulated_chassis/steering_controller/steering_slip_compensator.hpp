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

#include <px4_platform_common/module_params.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/predictive_traction.h>
#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>

/**
 * @brief Slip compensation for steering control
 *
 * Uses predictive traction data to compensate for vehicle slip
 * by adjusting steering commands to maintain desired trajectory.
 */
class SteeringSlipCompensator : public ModuleParams
{
public:
	struct SlipState {
		float slip_front{0.0f};
		float slip_rear{0.0f};
		float slip_error{0.0f};
		float compensation_rad{0.0f};
		bool traction_active{false};
		uint64_t last_update{0};
	};

	SteeringSlipCompensator(ModuleParams *parent);

	/**
	 * Apply slip compensation to target angle
	 * @param target_angle_rad Original target steering angle
	 * @return Compensated steering angle
	 */
	float apply_compensation(float target_angle_rad);

	/**
	 * Update slip compensation state
	 */
	void update();

	/**
	 * Get current slip state
	 */
	const SlipState &get_state() const { return _slip_state; }

	/**
	 * Check if slip compensation is active
	 */
	bool is_active() const { return _slip_compensation_enabled.get() && _slip_state.traction_active; }

private:
	// uORB subscription
	uORB::Subscription _predictive_traction_sub{ORB_ID(predictive_traction)};

	// Slip state
	SlipState _slip_state;

	// Slip compensation parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::STEER_SLP_CP_EN>) _slip_compensation_enabled,
		(ParamFloat<px4::params::STEER_SLP_CP_GN>) _slip_compensation_gain,
		(ParamFloat<px4::params::STEER_SLP_CP_MA>) _slip_compensation_max
	)
};