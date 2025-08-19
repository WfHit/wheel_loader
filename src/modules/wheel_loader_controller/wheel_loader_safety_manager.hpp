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

// System includes first
#include <drivers/drv_hrt.h>

// Library includes
#include <lib/perf/perf_counter.h>

// uORB includes
#include <uORB/Subscription.hpp>
#include <uORB/topics/boom_status.h>
#include <uORB/topics/bucket_status.h>
#include <uORB/topics/steering_status.h>
#include <uORB/topics/wheel_status.h>
#include <uORB/topics/slip_estimation.h>

/**
 * @brief Safety and health monitoring component for wheel loader controller
 *
 * Manages safety checks, emergency stop functionality, and subsystem health monitoring.
 * This component is responsible for:
 * - Monitoring subsystem health status
 * - Evaluating overall system health
 * - Handling emergency stop conditions
 * - Slip detection and traction control
 *
 * @note This component follows the composition pattern used by other PX4 modules
 * and provides a clear separation of safety-related functionality.
 */
class WheelLoaderSafetyManager
{
public:
	// System health states
	enum class HealthState : uint8_t {
		UNKNOWN = 0,
		HEALTHY = 1,
		WARNING = 2,
		ERROR = 3,
		CRITICAL = 4
	};

	static constexpr float HEALTH_TIMEOUT_S = 1.0f;
	static constexpr float MAX_EMERGENCY_STOP_TIME_S = 0.1f;

	WheelLoaderSafetyManager();
	~WheelLoaderSafetyManager() = default;

	/**
	 * @brief Initialize the safety manager
	 */
	bool init();

	/**
	 * @brief Update safety checks and health monitoring
	 *
	 * Should be called at regular intervals from the main control loop.
	 */
	void update();

	/**
	 * @brief Perform safety checks
	 *
	 * Checks for emergency stop conditions and other safety violations.
	 */
	void performSafetyChecks();

	/**
	 * @brief Update subsystem health monitoring
	 *
	 * Monitors health status of all subsystems and updates health states.
	 */
	void updateSubsystemHealth();

	/**
	 * @brief Handle emergency stop
	 *
	 * Activates emergency stop procedures and notifies all subsystems.
	 */
	void handleEmergencyStop();

	/**
	 * @brief Check if system is healthy
	 *
	 * @return true if system is operating normally, false if there are health issues
	 */
	bool isSystemHealthy() const;

	/**
	 * @brief Evaluate overall health state
	 *
	 * @return worst health state among all subsystems
	 */
	HealthState evaluateOverallHealth() const;

	/**
	 * @brief Check if emergency stop is active
	 *
	 * @return true if emergency stop is currently active
	 */
	bool isEmergencyStopActive() const { return _emergency_stop_active; }

	/**
	 * @brief Check if slip is detected
	 *
	 * @return true if wheel slip is currently detected
	 */
	bool isSlipDetected() const { return _slip_detected; }

	/**
	 * @brief Get traction reduction factor
	 *
	 * @return factor to reduce traction commands (0.0 to 1.0)
	 */
	float getTractionReductionFactor() const { return _traction_reduction_factor; }

	/**
	 * @brief Get subsystem health states
	 */
	HealthState getBoomHealth() const { return _boom_health; }
	HealthState getBucketHealth() const { return _bucket_health; }
	HealthState getSteeringHealth() const { return _steering_health; }
	HealthState getFrontWheelHealth() const { return _front_wheel_health; }
	HealthState getRearWheelHealth() const { return _rear_wheel_health; }

private:
	// Health monitoring
	void checkSubsystemTimeouts();
	void updateSlipEstimation();

	// uORB subscriptions for health monitoring
	uORB::Subscription _boom_status_sub{ORB_ID(boom_status)};
	uORB::Subscription _bucket_status_sub{ORB_ID(bucket_status)};
	uORB::Subscription _steering_status_sub{ORB_ID(steering_status)};
	uORB::Subscription _slip_estimation_sub{ORB_ID(slip_estimation)};
	uORB::SubscriptionMultiArray<wheel_status_s, 2> _wheel_status_subs{ORB_ID::wheel_status};

	// Subsystem health tracking
	HealthState _boom_health{HealthState::UNKNOWN};
	HealthState _bucket_health{HealthState::UNKNOWN};
	HealthState _steering_health{HealthState::UNKNOWN};
	HealthState _front_wheel_health{HealthState::UNKNOWN};
	HealthState _rear_wheel_health{HealthState::UNKNOWN};

	// Health timestamp tracking
	hrt_abstime _last_boom_status_time{0};
	hrt_abstime _last_bucket_status_time{0};
	hrt_abstime _last_steering_status_time{0};
	hrt_abstime _last_wheel_status_time[2]{0, 0};

	// Safety state
	bool _emergency_stop_active{false};
	bool _safety_override_active{false};
	hrt_abstime _emergency_stop_time{0};

	// Slip detection and traction control
	slip_estimation_s _current_slip_data{};
	bool _slip_detected{false};
	bool _critical_slip{false};
	hrt_abstime _last_slip_estimation_time{0};
	float _traction_reduction_factor{1.0f};

	// Performance counters
	perf_counter_t _emergency_stop_perf;
};