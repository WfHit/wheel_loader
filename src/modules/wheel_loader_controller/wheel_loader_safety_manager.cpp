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

#include "wheel_loader_safety_manager.hpp"

#include <px4_platform_common/log.h>
#include <lib/mathlib/mathlib.h>

WheelLoaderSafetyManager::WheelLoaderSafetyManager()
{
}

bool WheelLoaderSafetyManager::init()
{
	// Initialize performance counters
	_emergency_stop_perf = perf_alloc(PC_COUNT, "wl_safety: emergency_stops");

	// Initialize health states
	_boom_health = HealthState::UNKNOWN;
	_bucket_health = HealthState::UNKNOWN;
	_steering_health = HealthState::UNKNOWN;
	_front_wheel_health = HealthState::UNKNOWN;
	_rear_wheel_health = HealthState::UNKNOWN;

	return true;
}

void WheelLoaderSafetyManager::update()
{
	updateSubsystemHealth();
	updateSlipEstimation();
	performSafetyChecks();
}

void WheelLoaderSafetyManager::performSafetyChecks()
{
	// Check for emergency stop conditions
	if (_emergency_stop_active) {
		hrt_abstime now = hrt_absolute_time();

		if ((now - _emergency_stop_time) > (MAX_EMERGENCY_STOP_TIME_S * 1_s)) {
			// Emergency stop has been active long enough, allow normal operation
			_emergency_stop_active = false;
		}
	}

	// Check for critical health conditions
	if (_boom_health == HealthState::CRITICAL ||
	    _bucket_health == HealthState::CRITICAL ||
	    _steering_health == HealthState::CRITICAL ||
	    _front_wheel_health == HealthState::CRITICAL ||
	    _rear_wheel_health == HealthState::CRITICAL) {
		handleEmergencyStop();
	}

	// Check for critical slip conditions
	if (_critical_slip) {
		PX4_WARN("Critical slip detected - reducing traction commands");
		_traction_reduction_factor = math::constrain(_traction_reduction_factor - 0.1f, 0.2f, 1.0f);
	} else if (_slip_detected) {
		PX4_DEBUG("Slip detected - applying moderate traction reduction");
		_traction_reduction_factor = math::constrain(_traction_reduction_factor - 0.05f, 0.5f, 1.0f);
	} else {
		// No slip detected, gradually restore full traction
		_traction_reduction_factor = math::constrain(_traction_reduction_factor + 0.02f, 0.0f, 1.0f);
	}
}

void WheelLoaderSafetyManager::updateSubsystemHealth()
{
	hrt_abstime now = hrt_absolute_time();

	// Update boom health
	if (_boom_status_sub.updated()) {
		boom_status_s boom_status;
		_boom_status_sub.copy(&boom_status);
		_last_boom_status_time = boom_status.timestamp;

		if (boom_status.enabled && boom_status.healthy) {
			_boom_health = HealthState::HEALTHY;
		} else if (boom_status.enabled && !boom_status.healthy) {
			_boom_health = HealthState::ERROR;
		} else {
			_boom_health = HealthState::WARNING;
		}
	}

	// Update bucket health
	if (_bucket_status_sub.updated()) {
		bucket_status_s bucket_status;
		_bucket_status_sub.copy(&bucket_status);
		_last_bucket_status_time = bucket_status.timestamp;

		if (bucket_status.enabled && bucket_status.healthy) {
			_bucket_health = HealthState::HEALTHY;
		} else if (bucket_status.enabled && !bucket_status.healthy) {
			_bucket_health = HealthState::ERROR;
		} else {
			_bucket_health = HealthState::WARNING;
		}
	}

	// Update steering health
	if (_steering_status_sub.updated()) {
		steering_status_s steering_status;
		_steering_status_sub.copy(&steering_status);
		_last_steering_status_time = steering_status.timestamp;

		if (steering_status.enabled && steering_status.healthy) {
			_steering_health = HealthState::HEALTHY;
		} else if (steering_status.enabled && !steering_status.healthy) {
			_steering_health = HealthState::ERROR;
		} else {
			_steering_health = HealthState::WARNING;
		}
	}

	// Update wheel health
	for (int i = 0; i < 2; i++) {
		if (_wheel_status_subs[i].updated()) {
			wheel_status_s wheel_status;
			_wheel_status_subs[i].copy(&wheel_status);
			_last_wheel_status_time[i] = wheel_status.timestamp;

			HealthState wheel_health;
			if (wheel_status.controller_healthy && wheel_status.motor_healthy) {
				wheel_health = HealthState::HEALTHY;
			} else {
				wheel_health = HealthState::ERROR;
			}

			if (i == 0) {
				_front_wheel_health = wheel_health;
			} else {
				_rear_wheel_health = wheel_health;
			}
		}
	}

	// Check for timeouts
	checkSubsystemTimeouts();
}

void WheelLoaderSafetyManager::checkSubsystemTimeouts()
{
	hrt_abstime now = hrt_absolute_time();
	hrt_abstime timeout_threshold = HEALTH_TIMEOUT_S * 1_s;

	// Check boom timeout
	if (_last_boom_status_time > 0 && (now - _last_boom_status_time) > timeout_threshold) {
		_boom_health = HealthState::ERROR;
	}

	// Check bucket timeout
	if (_last_bucket_status_time > 0 && (now - _last_bucket_status_time) > timeout_threshold) {
		_bucket_health = HealthState::ERROR;
	}

	// Check steering timeout
	if (_last_steering_status_time > 0 && (now - _last_steering_status_time) > timeout_threshold) {
		_steering_health = HealthState::ERROR;
	}

	// Check wheel timeouts
	for (int i = 0; i < 2; i++) {
		if (_last_wheel_status_time[i] > 0 && (now - _last_wheel_status_time[i]) > timeout_threshold) {
			if (i == 0) {
				_front_wheel_health = HealthState::ERROR;
			} else {
				_rear_wheel_health = HealthState::ERROR;
			}
		}
	}
}

void WheelLoaderSafetyManager::updateSlipEstimation()
{
	if (_slip_estimation_sub.updated()) {
		_slip_estimation_sub.copy(&_current_slip_data);
		_last_slip_estimation_time = _current_slip_data.timestamp;

		// Update slip detection flags based on slip estimation data
		_slip_detected = _current_slip_data.front_slip > 0.1f || _current_slip_data.rear_slip > 0.1f;
		_critical_slip = _current_slip_data.front_slip > 0.3f || _current_slip_data.rear_slip > 0.3f;

		if (_slip_detected) {
			PX4_DEBUG("Slip detected: front=%.2f, rear=%.2f",
				(double)_current_slip_data.front_slip,
				(double)_current_slip_data.rear_slip);
		}
	} else {
		// Check for slip estimation timeout
		hrt_abstime now = hrt_absolute_time();
		if (_last_slip_estimation_time > 0 && (now - _last_slip_estimation_time) > (2.0f * 1_s)) {
			// No slip data for 2 seconds, assume no slip
			_slip_detected = false;
			_critical_slip = false;
		}
	}
}

void WheelLoaderSafetyManager::handleEmergencyStop()
{
	if (!_emergency_stop_active) {
		_emergency_stop_active = true;
		_emergency_stop_time = hrt_absolute_time();
		perf_count(_emergency_stop_perf);

		PX4_WARN("Emergency stop activated");
	}
}

bool WheelLoaderSafetyManager::isSystemHealthy() const
{
	HealthState overall_health = evaluateOverallHealth();
	return overall_health == HealthState::HEALTHY || overall_health == HealthState::WARNING;
}

WheelLoaderSafetyManager::HealthState WheelLoaderSafetyManager::evaluateOverallHealth() const
{
	HealthState worst_health = HealthState::HEALTHY;

	// Check all subsystem health states and find the worst one
	if (_boom_health > worst_health) {
		worst_health = _boom_health;
	}
	if (_bucket_health > worst_health) {
		worst_health = _bucket_health;
	}
	if (_steering_health > worst_health) {
		worst_health = _steering_health;
	}
	if (_front_wheel_health > worst_health) {
		worst_health = _front_wheel_health;
	}
	if (_rear_wheel_health > worst_health) {
		worst_health = _rear_wheel_health;
	}

	return worst_health;
}