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

#ifndef EKF_UWB_CHECKS_H
#define EKF_UWB_CHECKS_H

#include "../../common.h"

namespace estimator
{
class UwbChecks final
{
public:
	UwbChecks(int32_t &check_mask, float &req_range_accuracy, float &req_rssi_threshold, float &req_los_confidence,
		  float &max_range_drift, float &innovation_gate, uint32_t &min_health_time_us,
		  filter_control_status_u &control_status):
		_params{check_mask, req_range_accuracy, req_rssi_threshold, req_los_confidence, max_range_drift, innovation_gate, min_health_time_us},
		_control_status(control_status)
	{};

	union uwb_check_fail_status_u {
		struct {
			uint16_t range_accuracy : 1;  ///< 0 - true if reported range accuracy is insufficient
			uint16_t rssi          : 1;   ///< 1 - true if RSSI is below threshold
			uint16_t los_confidence : 1;  ///< 2 - true if Line-of-Sight confidence is low
			uint16_t range_drift   : 1;   ///< 3 - true if range drift is excessive
			uint16_t innovation    : 1;   ///< 4 - true if innovation gate test fails
			uint16_t timeout       : 1;   ///< 5 - true if anchor data is stale
			uint16_t anchor_count  : 1;   ///< 6 - true if insufficient active anchors
			uint16_t multipath     : 1;   ///< 7 - true if multipath detected
		} flags;
		uint16_t value;
	};

	struct AnchorStats {
		uint8_t anchor_id;
		float last_range_m;
		float range_drift_rate_m_s;
		float filtered_rssi;
		float filtered_los_confidence;
		uint64_t last_update_us;
		uint32_t consecutive_failures;
		bool is_healthy;
	};

	void resetHard()
	{
		_initial_checks_passed = false;
		reset();
	}

	void reset()
	{
		_passed = false;
		_time_last_pass_us = 0;
		_time_last_fail_us = 0;
		resetDriftFilters();
		_active_anchor_count = 0;
		for (auto &anchor : _anchor_stats) {
			anchor = {};
		}
	}

	/*
	 * Return true if the UWB solution quality is adequate.
	*/
	bool run(const uwbSample &uwb, uint64_t time_us);
	bool passed() const { return _passed; }
	bool initialChecksPassed() const { return _initial_checks_passed; }
	uint64_t getLastPassUs() const { return _time_last_pass_us; }
	uint64_t getLastFailUs() const { return _time_last_fail_us; }

	const uwb_check_fail_status_u &getFailStatus() const { return _check_fail_status; }

	uint8_t getActiveAnchorCount() const { return _active_anchor_count; }
	const AnchorStats *getAnchorStats() const { return _anchor_stats; }

private:
	enum class UwbChecksMask : int32_t {
		kRangeAccuracy  = (1 << 0),
		kRssi          = (1 << 1),
		kLosConfidence = (1 << 2),
		kRangeDrift    = (1 << 3),
		kInnovation    = (1 << 4),
		kTimeout       = (1 << 5),
		kAnchorCount   = (1 << 6),
		kMultipath     = (1 << 7)
	};

	bool isCheckEnabled(UwbChecksMask check) { return (_params.check_mask & static_cast<int32_t>(check)); }

	bool runRangeAccuracyCheck(const uwbSample &uwb);
	bool runSignalQualityChecks(const uwbSample &uwb);
	bool runDriftCheck(const uwbSample &uwb);
	bool runTimeoutCheck(const uwbSample &uwb, uint64_t time_us);
	bool runAnchorCountCheck();
	bool runMultipathCheck(const uwbSample &uwb);

	void updateAnchorStats(const uwbSample &uwb, uint64_t time_us);
	void resetDriftFilters();

	bool isTimedOut(uint64_t timestamp_to_check_us, uint64_t now_us, uint64_t timeout_period) const
	{
		return (timestamp_to_check_us == 0) || (timestamp_to_check_us + timeout_period < now_us);
	}

	uwb_check_fail_status_u _check_fail_status{};

	static constexpr uint8_t MAX_ANCHORS = 8;
	AnchorStats _anchor_stats[MAX_ANCHORS]{};
	uint8_t _active_anchor_count{0};

	uint64_t _time_last_fail_us{0};
	uint64_t _time_last_pass_us{0};
	bool _initial_checks_passed{false};
	bool _passed{false};

	struct Params {
		const int32_t &check_mask;
		const float &req_range_accuracy;
		const float &req_rssi_threshold;
		const float &req_los_confidence;
		const float &max_range_drift;
		const float &innovation_gate;
		const uint32_t &min_health_time_us;
	};

	const Params _params;
	const filter_control_status_u &_control_status;
};
}; // namespace estimator

#endif // !EKF_UWB_CHECKS_H
