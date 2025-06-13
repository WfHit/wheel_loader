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

/**
 * @file uwb_control.cpp
 * Control functions for UWB range fusion
 *
 */

#include "ekf.h"
#include <mathlib/mathlib.h>

using namespace estimator;

void Ekf::controlUwbFusion(const imuSample &imu_delayed)
{
#if defined(CONFIG_EKF2_UWB)
	// Check if we have new UWB data that has fallen behind the fusion time horizon
	if (_uwb_data_ready) {
		// Run UWB checks
		if (_uwb_checks.run(_uwb_sample_delayed, _time_delayed_us)) {

			// Update the measurement
			updateUwbRange(_uwb_sample_delayed, _aid_src_uwb);

			// Determine if we should start UWB fusion
			const bool continuing_conditions_passing = _aid_src_uwb.timestamp_sample != 0;
			const bool starting_conditions_passing = continuing_conditions_passing;

			if (_control_status.flags.uwb) {
				// Currently fusing UWB
				if (continuing_conditions_passing) {
					fuseUwbRange(_uwb_sample_delayed);

					const bool is_fusion_failing = isTimedOut(_aid_src_uwb.time_last_fuse, (uint64_t)10e6);

					if (is_fusion_failing) {
						stopUwbFusion();
					}

				} else {
					stopUwbFusion();
				}

			} else {
				// Not currently fusing UWB
				if (starting_conditions_passing) {
					ECL_INFO("Starting UWB fusion");
					_control_status.flags.uwb = true;
					fuseUwbRange(_uwb_sample_delayed);
				}
			}

		} else {
			// UWB checks failed
			if (_control_status.flags.uwb) {
				ECL_WARN("UWB quality checks failed, stopping fusion");
				stopUwbFusion();
			}
		}

		_uwb_data_ready = false;
	}

	// Check for UWB sensor timeout
	if (_control_status.flags.uwb) {
		if (isTimedOut(_time_last_uwb_buffer_push, 2 * GNSS_MAX_INTERVAL)) {
			ECL_WARN("UWB data timeout");
			stopUwbFusion();
		}
	}
#endif // CONFIG_EKF2_UWB
}

void Ekf::updateUwbRange(const uwbSample &uwb_sample, estimator_aid_source1d_s &aid_src)
{
	// Calculate the range to the anchor from current position estimate
	const Vector3f anchor_pos_ned = uwb_sample.anchor_pos;
	const Vector3f pos_offset_earth = _R_to_earth * _params.uwb_pos_body;
	const Vector3f pred_pos = _state.pos + pos_offset_earth;

	const Vector3f range_vector = pred_pos - anchor_pos_ned;
	const float predicted_range = range_vector.norm();

	// Calculate innovation
	const float innovation = uwb_sample.range_m - predicted_range;

	// Calculate innovation variance
	// H = [dx/r, dy/r, dz/r, 0, 0, 0, ...] where r = sqrt(dx^2 + dy^2 + dz^2)
	VectorState H;
	H.setZero();

	if (predicted_range > 0.1f) {
		// Position states (0, 1, 2 correspond to North, East, Down)
		H(0) = range_vector(0) / predicted_range; // North
		H(1) = range_vector(1) / predicted_range; // East
		H(2) = range_vector(2) / predicted_range; // Down
	}

	// Calculate innovation variance: H*P*H^T + R
	const float innovation_variance = (H.T() * P * H)(0, 0) + sq(uwb_sample.range_accuracy);

	// Apply adaptive measurement noise based on signal quality
	float measurement_noise = uwb_sample.range_accuracy;

	// Scale noise based on RSSI quality
	if (uwb_sample.rssi < _params.req_rssi_threshold) {
		const float rssi_factor = math::constrain(1.0f + ((_params.req_rssi_threshold - uwb_sample.rssi) * 0.1f), 1.0f, 5.0f);
		measurement_noise *= rssi_factor;
	}

	// Scale noise based on LOS confidence
	if (uwb_sample.los_confidence < _params.req_los_confidence) {
		const float los_factor = math::constrain(1.0f / math::max(uwb_sample.los_confidence, 0.1f), 1.0f, 10.0f);
		measurement_noise *= los_factor;
	}

	// Update aid source status
	updateAidSourceStatus(aid_src,
			      uwb_sample.time_us,
			      uwb_sample.range_m,
			      sq(measurement_noise),
			      innovation,
			      innovation_variance,
			      _params.uwb_innov_gate);
}

void Ekf::fuseUwbRange(const uwbSample &uwb_sample)
{
	// Check innovation gate
	if (_aid_src_uwb.innovation_rejected) {
		return;
	}

	// Calculate observation matrix H
	const Vector3f anchor_pos_ned = uwb_sample.anchor_pos;
	const Vector3f pos_offset_earth = _R_to_earth * _params.uwb_pos_body;
	const Vector3f pred_pos = _state.pos + pos_offset_earth;

	const Vector3f range_vector = pred_pos - anchor_pos_ned;
	const float predicted_range = range_vector.norm();

	VectorState H;
	H.setZero();

	if (predicted_range > 0.1f) {
		H(0) = range_vector(0) / predicted_range; // North
		H(1) = range_vector(1) / predicted_range; // East
		H(2) = range_vector(2) / predicted_range; // Down
	}

	// Calculate Kalman gain
	const float innovation_variance = _aid_src_uwb.innovation_variance;
	VectorState K = P * H / innovation_variance;

	// Apply innovation consistency check
	if (innovation_variance > 0.0f) {
		// Apply measurement update
		if (measurementUpdate(K, H, _aid_src_uwb.observation_variance, _aid_src_uwb.innovation)) {
			_aid_src_uwb.fused = true;
			_aid_src_uwb.time_last_fuse = _time_delayed_us;
		}
	}
}

void Ekf::resetUwbFusion()
{
	ECL_INFO("Resetting UWB fusion");
	_aid_src_uwb.reset();
	_uwb_checks.resetHard();
	_control_status.flags.uwb = false;
}

void Ekf::stopUwbFusion()
{
	ECL_INFO("Stopping UWB fusion");
	_control_status.flags.uwb = false;
	_aid_src_uwb.reset();
}

bool Ekf::isUwbDataReady()
{
	// Implementation will depend on how UWB data is buffered
	// For now, return the basic check
	return _uwb_data_ready;
}
