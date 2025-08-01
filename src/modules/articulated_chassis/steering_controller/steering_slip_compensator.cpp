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

#include "steering_slip_compensator.hpp"

SteeringSlipCompensator::SteeringSlipCompensator(ModuleParams *parent)
	: ModuleParams(parent)
{
}

float SteeringSlipCompensator::apply_compensation(float target_angle_rad)
{
	if (!is_active()) {
		_slip_state.compensation_rad = 0.0f;
		return target_angle_rad;
	}

	// Calculate slip error (positive = understeer, negative = oversteer)
	_slip_state.slip_error = _slip_state.slip_front - _slip_state.slip_rear;

	// Apply compensation
	float compensation = _slip_state.slip_error * _slip_compensation_gain.get();
	compensation = math::constrain(compensation,
				       -_slip_compensation_max.get(),
				       _slip_compensation_max.get());

	_slip_state.compensation_rad = compensation;
	return target_angle_rad + compensation;
}

void SteeringSlipCompensator::update()
{
	predictive_traction_s traction{};

	if (_predictive_traction_sub.update(&traction)) {
		_slip_state.last_update = hrt_absolute_time();
		_slip_state.slip_front = traction.predicted_slip_front;
		_slip_state.slip_rear = traction.predicted_slip_rear;
		_slip_state.traction_active = traction.predictive_active;
	}
}