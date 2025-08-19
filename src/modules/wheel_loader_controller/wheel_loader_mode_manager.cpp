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

#include "wheel_loader_mode_manager.hpp"

#include <px4_platform_common/log.h>

using namespace time_literals;

WheelLoaderModeManager::WheelLoaderModeManager()
{
}

bool WheelLoaderModeManager::init()
{
	// Initialize to manual mode for safety
	_operation_mode = OperationMode::MANUAL_MODE;
	_previous_operation_mode = OperationMode::MANUAL_MODE;
	_target_mode = OperationMode::MANUAL_MODE;
	_last_requested_mode = 0;

	return true;
}

void WheelLoaderModeManager::update(int requested_mode)
{
	// Check for mode change request
	if (requested_mode != _last_requested_mode) {
		OperationMode new_mode = static_cast<OperationMode>(requested_mode);

		if (new_mode != _operation_mode && new_mode != OperationMode::TRANSITION_MODE) {
			if (requestModeTransition(new_mode)) {
				_last_requested_mode = requested_mode;
			}
		}
	}

	// Handle ongoing mode transition
	if (_transition_in_progress) {
		handleModeTransition();
	}
}

bool WheelLoaderModeManager::requestModeTransition(OperationMode new_mode)
{
	if (!isValidModeTransition(_operation_mode, new_mode)) {
		PX4_WARN("Invalid mode transition from %d to %d", (int)_operation_mode, (int)new_mode);
		return false;
	}

	if (_transition_in_progress) {
		PX4_WARN("Mode transition already in progress, ignoring new request");
		return false;
	}

	PX4_INFO("Transitioning from mode %d to mode %d", (int)_operation_mode, (int)new_mode);

	_previous_operation_mode = _operation_mode;
	_target_mode = new_mode;
	_operation_mode = OperationMode::TRANSITION_MODE;
	_mode_transition_start_time = hrt_absolute_time();
	_transition_in_progress = true;

	return true;
}

void WheelLoaderModeManager::handleModeTransition()
{
	hrt_abstime now = hrt_absolute_time();
	hrt_abstime transition_duration = now - _mode_transition_start_time;

	// Check if transition timeout has elapsed
	if (transition_duration >= (MODE_TRANSITION_TIME_S * 1_s)) {
		// Complete the transition
		_operation_mode = _target_mode;
		_transition_in_progress = false;

		PX4_INFO("Mode transition complete: now in mode %d", (int)_operation_mode);
	}
}

bool WheelLoaderModeManager::isValidModeTransition(OperationMode from, OperationMode to) const
{
	// Manual mode can always override auto mode (safety requirement)
	if (to == OperationMode::MANUAL_MODE) {
		return true;
	}

	// Auto mode can be entered from manual mode or idle
	if (to == OperationMode::AUTO_MODE) {
		return from == OperationMode::MANUAL_MODE;
	}

	// Transition mode is internal only
	if (to == OperationMode::TRANSITION_MODE) {
		return false;
	}

	return false;
}