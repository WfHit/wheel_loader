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
 * INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "uwb_checks.hpp"
#include <math.h>

using namespace estimator;

bool UwbChecks::run(const uwbSample &uwb, uint64_t time_us)
{
	_check_fail_status.value = 0;

	// Update anchor statistics
	updateAnchorStats(uwb, time_us);

	bool check_pass = true;

	// Range accuracy check
	if (isCheckEnabled(UwbChecksMask::kRangeAccuracy)) {
		if (!runRangeAccuracyCheck(uwb)) {
			_check_fail_status.flags.range_accuracy = true;
			check_pass = false;
		}
	}

	// Signal quality checks (RSSI and LOS confidence)
	if (isCheckEnabled(UwbChecksMask::kRssi) || isCheckEnabled(UwbChecksMask::kLosConfidence)) {
		if (!runSignalQualityChecks(uwb)) {
			check_pass = false;
		}
	}

	// Range drift check
	if (isCheckEnabled(UwbChecksMask::kRangeDrift)) {
		if (!runDriftCheck(uwb)) {
			_check_fail_status.flags.range_drift = true;
			check_pass = false;
		}
	}

	// Timeout check
	if (isCheckEnabled(UwbChecksMask::kTimeout)) {
		if (!runTimeoutCheck(uwb, time_us)) {
			_check_fail_status.flags.timeout = true;
			check_pass = false;
		}
	}

	// Anchor count check
	if (isCheckEnabled(UwbChecksMask::kAnchorCount)) {
		if (!runAnchorCountCheck()) {
			_check_fail_status.flags.anchor_count = true;
			check_pass = false;
		}
	}

	// Multipath detection
	if (isCheckEnabled(UwbChecksMask::kMultipath)) {
		if (!runMultipathCheck(uwb)) {
			_check_fail_status.flags.multipath = true;
			check_pass = false;
		}
	}

	// Update pass/fail state
	if (check_pass) {
		_time_last_pass_us = time_us;

		if (!_initial_checks_passed) {
			_initial_checks_passed = true;
		}

		_passed = true;

	} else {
		_time_last_fail_us = time_us;
		_passed = false;
	}

	return _passed;
}

bool UwbChecks::runRangeAccuracyCheck(const uwbSample &uwb)
{
	return (uwb.range_accuracy >= 0.0f) && (uwb.range_accuracy <= _params.req_range_accuracy);
}

bool UwbChecks::runSignalQualityChecks(const uwbSample &uwb)
{
	bool rssi_check_pass = true;
	bool los_check_pass = true;

	if (isCheckEnabled(UwbChecksMask::kRssi)) {
		rssi_check_pass = (uwb.rssi >= _params.req_rssi_threshold);
		if (!rssi_check_pass) {
			_check_fail_status.flags.rssi = true;
		}
	}

	if (isCheckEnabled(UwbChecksMask::kLosConfidence)) {
		los_check_pass = (uwb.los_confidence >= _params.req_los_confidence);
		if (!los_check_pass) {
			_check_fail_status.flags.los_confidence = true;
		}
	}

	return rssi_check_pass && los_check_pass;
}

bool UwbChecks::runDriftCheck(const uwbSample &uwb)
{
	// Find the anchor stats for this measurement
	AnchorStats *anchor = nullptr;
	for (uint8_t i = 0; i < MAX_ANCHORS; i++) {
		if (_anchor_stats[i].anchor_id == uwb.anchor_id) {
			anchor = &_anchor_stats[i];
			break;
		}
	}

	if (anchor == nullptr) {
		// No previous data for this anchor - pass the check
		return true;
	}

	// Check if drift rate exceeds threshold
	return fabsf(anchor->range_drift_rate_m_s) <= _params.max_range_drift;
}

bool UwbChecks::runTimeoutCheck(const uwbSample &uwb, uint64_t time_us)
{
	// Check if UWB data is fresh enough
	static constexpr uint64_t UWB_TIMEOUT_US = 500000; // 500ms timeout
	return !isTimedOut(uwb.time_us, time_us, UWB_TIMEOUT_US);
}

bool UwbChecks::runAnchorCountCheck()
{
	// Require at least 3 active anchors for position estimation
	return _active_anchor_count >= 3;
}

bool UwbChecks::runMultipathCheck(const uwbSample &uwb)
{
	// Basic multipath detection based on signal characteristics
	// This is a simplified implementation - real multipath detection would be more complex

	// Check for abnormally high path loss for the given range
	if (uwb.range_m > 0.0f && uwb.rssi > -100.0f) {
		// Calculate expected signal strength based on free-space path loss
		const float frequency_hz = 6.5e9f; // Center frequency for UWB
		const float c = 299792458.0f; // Speed of light
		const float wavelength = c / frequency_hz;
		const float expected_path_loss_db = 20.0f * log10f((4.0f * M_PI * uwb.range_m) / wavelength);

		// Assuming 0 dBm transmit power and some margin for multipath
		const float expected_rssi = -expected_path_loss_db - 10.0f; // 10dB margin

		// If measured RSSI is significantly lower than expected, suspect multipath
		if (uwb.rssi < (expected_rssi - 15.0f)) {
			return false;
		}
	}

	return true;
}

void UwbChecks::updateAnchorStats(const uwbSample &uwb, uint64_t time_us)
{
	// Find existing anchor or create new entry
	AnchorStats *anchor = nullptr;
	uint8_t free_slot = MAX_ANCHORS;

	for (uint8_t i = 0; i < MAX_ANCHORS; i++) {
		if (_anchor_stats[i].anchor_id == uwb.anchor_id) {
			anchor = &_anchor_stats[i];
			break;
		} else if (_anchor_stats[i].anchor_id == 0 && free_slot == MAX_ANCHORS) {
			free_slot = i;
		}
	}

	// If anchor not found, use free slot
	if (anchor == nullptr && free_slot < MAX_ANCHORS) {
		anchor = &_anchor_stats[free_slot];
		anchor->anchor_id = uwb.anchor_id;
		anchor->is_healthy = false;
	}

	if (anchor == nullptr) {
		// No space for new anchor
		return;
	}

	// Update drift calculation
	if (anchor->last_update_us > 0) {
		const float dt_s = (time_us - anchor->last_update_us) * 1e-6f;
		if (dt_s > 0.001f && dt_s < 10.0f) { // Reasonable time interval
			const float range_change = uwb.range_m - anchor->last_range_m;
			const float new_drift_rate = range_change / dt_s;

			// Apply low-pass filter to drift rate
			const float alpha = 0.1f;
			anchor->range_drift_rate_m_s = alpha * new_drift_rate + (1.0f - alpha) * anchor->range_drift_rate_m_s;
		}
	}

	// Update filtered signal quality metrics
	const float signal_alpha = 0.2f;
	if (anchor->last_update_us > 0) {
		anchor->filtered_rssi = signal_alpha * uwb.rssi + (1.0f - signal_alpha) * anchor->filtered_rssi;
		anchor->filtered_los_confidence = signal_alpha * uwb.los_confidence + (1.0f - signal_alpha) * anchor->filtered_los_confidence;
	} else {
		anchor->filtered_rssi = uwb.rssi;
		anchor->filtered_los_confidence = uwb.los_confidence;
	}

	// Update anchor state
	anchor->last_range_m = uwb.range_m;
	anchor->last_update_us = time_us;

	// Update health status
	bool anchor_healthy = true;
	if (isCheckEnabled(UwbChecksMask::kRssi) && anchor->filtered_rssi < _params.req_rssi_threshold) {
		anchor_healthy = false;
	}
	if (isCheckEnabled(UwbChecksMask::kLosConfidence) && anchor->filtered_los_confidence < _params.req_los_confidence) {
		anchor_healthy = false;
	}
	if (isCheckEnabled(UwbChecksMask::kRangeDrift) && fabsf(anchor->range_drift_rate_m_s) > _params.max_range_drift) {
		anchor_healthy = false;
	}

	if (anchor_healthy) {
		anchor->consecutive_failures = 0;
		anchor->is_healthy = true;
	} else {
		anchor->consecutive_failures++;
		if (anchor->consecutive_failures > 5) {
			anchor->is_healthy = false;
		}
	}

	// Count active anchors
	_active_anchor_count = 0;
	for (uint8_t i = 0; i < MAX_ANCHORS; i++) {
		if (_anchor_stats[i].anchor_id != 0 &&
		    !isTimedOut(_anchor_stats[i].last_update_us, time_us, 1000000)) { // 1 second timeout
			_active_anchor_count++;
		}
	}
}

void UwbChecks::resetDriftFilters()
{
	for (auto &anchor : _anchor_stats) {
		anchor.range_drift_rate_m_s = 0.0f;
		anchor.consecutive_failures = 0;
	}
}
