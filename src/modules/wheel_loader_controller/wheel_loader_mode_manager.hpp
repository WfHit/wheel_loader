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
#include <px4_platform_common/module_params.h>

/**
 * @brief Mode management component for wheel loader controller
 *
 * Manages operation mode switching between manual RC control and VLA autonomous operation.
 * This component is responsible for:
 * - Safe mode transitions between manual and autonomous control
 * - Mode validation and timeout handling
 * - Transition state management
 *
 * @note This component follows the composition pattern used by other PX4 modules
 * and provides a clear separation of mode management functionality.
 */
class WheelLoaderModeManager
{
public:
	// Operation modes for dual control system
	enum class OperationMode : uint8_t {
		MANUAL_MODE = 0,        // RC control mode
		AUTO_MODE = 1,          // VLA autonomous mode
		TRANSITION_MODE = 2     // Transitioning between modes
	};

	static constexpr float MODE_TRANSITION_TIME_S = 1.0f;

	WheelLoaderModeManager();
	~WheelLoaderModeManager() = default;

	/**
	 * @brief Initialize the mode manager
	 */
	bool init();

	/**
	 * @brief Update mode management
	 *
	 * Should be called at regular intervals from the main control loop.
	 * Checks for mode change requests and handles transitions.
	 *
	 * @param requested_mode Mode requested via parameter
	 */
	void update(int requested_mode);

	/**
	 * @brief Request mode transition
	 *
	 * @param new_mode Target operation mode
	 * @return true if transition is valid and started
	 */
	bool requestModeTransition(OperationMode new_mode);

	/**
	 * @brief Handle ongoing mode transition
	 *
	 * Manages the transition process and switches to target mode when complete.
	 */
	void handleModeTransition();

	/**
	 * @brief Check if a mode transition is valid
	 *
	 * @param from Source operation mode
	 * @param to Target operation mode
	 * @return true if transition is allowed
	 */
	bool isValidModeTransition(OperationMode from, OperationMode to) const;

	/**
	 * @brief Get current operation mode
	 *
	 * @return Current operation mode
	 */
	OperationMode getCurrentMode() const { return _operation_mode; }

	/**
	 * @brief Get previous operation mode
	 *
	 * @return Previous operation mode (useful during transitions)
	 */
	OperationMode getPreviousMode() const { return _previous_operation_mode; }

	/**
	 * @brief Check if currently in transition
	 *
	 * @return true if mode transition is in progress
	 */
	bool isInTransition() const { return _operation_mode == OperationMode::TRANSITION_MODE; }

	/**
	 * @brief Get target mode during transition
	 *
	 * @return Target mode for current transition
	 */
	OperationMode getTargetMode() const { return _target_mode; }

private:
	// Mode state
	OperationMode _operation_mode{OperationMode::MANUAL_MODE};
	OperationMode _previous_operation_mode{OperationMode::MANUAL_MODE};
	OperationMode _target_mode{OperationMode::MANUAL_MODE};
	int _last_requested_mode{0};

	// Transition management
	hrt_abstime _mode_transition_start_time{0};
	bool _transition_in_progress{false};
};