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

#include "boom_actuator_interface.hpp"
#include <px4_platform_common/log.h>
#include <mathlib/mathlib.h>

BoomActuatorInterface::BoomActuatorInterface(ModuleParams *parent) :
	ModuleParams(parent)
{
}

bool BoomActuatorInterface::initialize(int motor_instance)
{
	_motor_instance = motor_instance;
	updateParams();

	// Select H-bridge instance
	_hbridge_selected = select_hbridge_instance();

	if (_hbridge_selected < 0) {
		PX4_ERR("No suitable H-bridge found for motor instance %d", motor_instance);
		return false;
	}

	_initialized = true;
	_is_healthy = true;

	PX4_INFO("Boom H-bridge interface initialized (H-bridge: %d)", _hbridge_selected);
	return true;
}

bool BoomActuatorInterface::send_command(const HbridgeCommand &command)
{
	if (!_initialized) {
		return false;
	}

	// Create H-bridge command
	hbridge_command_s hbridge_cmd{};
	hbridge_cmd.timestamp = hrt_absolute_time();
	hbridge_cmd.instance = _hbridge_selected; // Use configured H-bridge instance
	hbridge_cmd.duty_cycle = math::constrain(command.duty_cycle, -1.0f, 1.0f);
	hbridge_cmd.enable = command.enable;
	// Note: hbridge_command doesn't have a mode field, removed command.mode assignment

	// Apply duty cycle limit
	float max_duty = _param_duty_max.get();

	if (fabsf(hbridge_cmd.duty_cycle) > max_duty) {
		hbridge_cmd.duty_cycle = (hbridge_cmd.duty_cycle > 0.0f) ? max_duty : -max_duty;
	}

	// Publish command
	if (_hbridge_command_pub == nullptr) {
		_hbridge_command_pub = orb_advertise(ORB_ID(hbridge_command), &hbridge_cmd);
	} else {
		orb_publish(ORB_ID(hbridge_command), _hbridge_command_pub, &hbridge_cmd);
	}

	_last_command = command;
	_last_command_time = hrt_absolute_time();

	return true;
}

bool BoomActuatorInterface::update_status(HbridgeStatus &status)
{
	if (!_initialized) {
		return false;
	}

	// Get H-bridge status - use index for the configured H-bridge instance
	hbridge_status_s hbridge_status;

	if (_hbridge_status_sub[_hbridge_selected].updated() &&
		_hbridge_status_sub[_hbridge_selected].copy(&hbridge_status)) {
		// Update status from H-bridge
		status.enabled = hbridge_status.enabled;
		status.fault = hbridge_status.forward_limit || hbridge_status.reverse_limit; // Use limits as fault indicators
		status.current = 0.0f;  // Not available in hbridge_status
		status.voltage = 0.0f;  // Not available in hbridge_status
		status.temperature = 0.0f;  // Not available in hbridge_status
		status.timestamp = hbridge_status.timestamp;

		_last_status = status;
		_last_status_time = hrt_absolute_time();

		// Update health status
		if (status.fault) {
			_fault_count++;

			if (_fault_count >= MAX_FAULT_COUNT) {
				_is_healthy = false;
			}

		} else {
			_fault_count = 0;
			_is_healthy = true;
		}

		return true;
	}

	return false;
}

void BoomActuatorInterface::emergency_stop()
{
	HbridgeCommand stop_cmd{};
	stop_cmd.duty_cycle = 0.0f;
	stop_cmd.enable = false;
	stop_cmd.mode = 0;

	send_command(stop_cmd);
	_is_healthy = false;

	PX4_WARN("H-bridge emergency stop activated");
}

bool BoomActuatorInterface::is_healthy() const
{
	if (!_initialized) {
		return false;
	}

	// Check for status timeout
	if ((hrt_absolute_time() - _last_status_time) > STATUS_TIMEOUT_US) {
		return false;
	}

	return _is_healthy && !_last_status.fault;
}

int BoomActuatorInterface::select_hbridge_instance()
{
	// For now, just use the configured motor index
	// In a real implementation, this would scan available H-bridges
	// and select the best one based on health, availability, etc.
	return _param_motor_index.get();
}

BoomActuatorInterface::HbridgeCommand
BoomActuatorInterface::apply_current_limit(const HbridgeCommand &command, float current) const
{
	HbridgeCommand limited_cmd = command;

	float current_limit = _param_current_limit.get();

	if (fabsf(current) > current_limit) {
		// Reduce duty cycle proportionally
		float reduction_factor = current_limit / fabsf(current);
		limited_cmd.duty_cycle *= reduction_factor;

		PX4_WARN("Current limit exceeded: %.1fA > %.1fA, reducing duty cycle",
			 (double)current, (double)current_limit);
	}

	return limited_cmd;
}
