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
 * @file boom_calibration.cpp
 * @brief AS5600 magnetic encoder calibration routines for boom control
 *
 * Contains calibration state machine and translation functions for the boom control module.
 */

#include "boom_control.hpp"

#include <px4_platform_common/log.h>
#include <lib/mathlib/mathlib.h>

void BoomControl::start_auto_calibration()
{
	if (_state != BoomState::IDLE) {
		PX4_WARN("Cannot start calibration - boom not in IDLE state");
		return;
	}

	_calibration_mode = true;
	_calib_state = CalibrationState::MOVING_TO_MIN;
	_calib_start_time = hrt_absolute_time();
	_calib_limit_detected = false;

	PX4_INFO("Starting boom calibration - moving to down limit");
}

void BoomControl::update_calibration()
{
	switch (_calib_state) {
		case CalibrationState::MOVING_TO_MIN:
		{
			// Move down slowly until limit sensor is triggered
			_motor_output = -0.3f; // 30% power downward

			if (check_limit_sensors()) {
				if (!_calib_limit_detected) {
					_calib_limit_detected = true;
					_calib_settle_time = hrt_absolute_time();
				} else if (hrt_elapsed_time(&_calib_settle_time) > 500_ms) {
					// Settled for 500ms, record minimum angle
					_calib_state = CalibrationState::RECORDING_MIN;
				}
			} else {
				_calib_limit_detected = false;
			}

			// Timeout protection
			if (hrt_elapsed_time(&_calib_start_time) > 30_s) {
				PX4_ERR("Calibration timeout moving to minimum");
				abort_calibration();
				return;
			}
			break;
		}

		case CalibrationState::RECORDING_MIN:
		{
			_motor_output = 0.0f; // Stop motor

			// Read current AS5600 angle as minimum
			sensor_mag_encoder_s mag_encoder_data;
			if (_mag_encoder_sub.copy(&mag_encoder_data) &&
			    mag_encoder_data.device_id == static_cast<uint32_t>(_param_mag_encoder_instance_id.get())) {

				float raw_angle_deg = math::degrees(mag_encoder_data.angle);
				_calib_min_angle = raw_angle_deg * _param_mag_encoder_scale.get() + _param_mag_encoder_offset.get();

				PX4_INFO("Minimum angle recorded: %.1f degrees", static_cast<double>(_calib_min_angle));

				_calib_state = CalibrationState::MOVING_TO_MAX;
				_calib_limit_detected = false;
				PX4_INFO("Moving to up limit");
			}
			break;
		}

		case CalibrationState::MOVING_TO_MAX:
		{
			// Move up slowly until limit sensor is triggered
			_motor_output = 0.3f; // 30% power upward

			if (check_limit_sensors()) {
				if (!_calib_limit_detected) {
					_calib_limit_detected = true;
					_calib_settle_time = hrt_absolute_time();
				} else if (hrt_elapsed_time(&_calib_settle_time) > 500_ms) {
					// Settled for 500ms, record maximum angle
					_calib_state = CalibrationState::RECORDING_MAX;
				}
			} else {
				_calib_limit_detected = false;
			}

			// Timeout protection
			if (hrt_elapsed_time(&_calib_start_time) > 60_s) {
				PX4_ERR("Calibration timeout moving to maximum");
				abort_calibration();
				return;
			}
			break;
		}

		case CalibrationState::RECORDING_MAX:
		{
			_motor_output = 0.0f; // Stop motor

			// Read current AS5600 angle as maximum
			sensor_mag_encoder_s mag_encoder_data;
			if (_mag_encoder_sub.copy(&mag_encoder_data) &&
			    mag_encoder_data.device_id == static_cast<uint32_t>(_param_mag_encoder_instance_id.get())) {

				float raw_angle_deg = math::degrees(mag_encoder_data.angle);
				_calib_max_angle = raw_angle_deg * _param_mag_encoder_scale.get() + _param_mag_encoder_offset.get();

				PX4_INFO("Maximum angle recorded: %.1f degrees", static_cast<double>(_calib_max_angle));

				complete_calibration();
			}
			break;
		}

		case CalibrationState::COMPLETED:
		case CalibrationState::FAILED:
		default:
			// Return to normal operation
			_calibration_mode = false;
			_motor_output = 0.0f;
			break;
	}
}

void BoomControl::complete_calibration()
{
	_calib_state = CalibrationState::COMPLETED;
	_motor_output = 0.0f;

	// Determine direction based on angle difference
	float angle_diff = _calib_max_angle - _calib_min_angle;
	_calib_direction = (angle_diff > 0) ? 1.0f : -1.0f;

	// Display results
	PX4_INFO("=== Boom Calibration Results ===");
	PX4_INFO("Minimum AS5600 angle: %.1f degrees", static_cast<double>(_calib_min_angle));
	PX4_INFO("Maximum AS5600 angle: %.1f degrees", static_cast<double>(_calib_max_angle));
	PX4_INFO("Angle difference: %.1f degrees", static_cast<double>(fabsf(angle_diff)));
	PX4_INFO("Direction: %s", (_calib_direction > 0) ? "Normal" : "Reversed");

	// Suggest parameter values
	PX4_INFO("=== Suggested Parameters ===");
	PX4_INFO("BOOM_MAG_SCALE: %.3f", static_cast<double>(_calib_direction));
	PX4_INFO("BOOM_MAG_OFFSET: %.1f", static_cast<double>(-_calib_min_angle * _calib_direction));
	PX4_INFO("BOOM_ANGLE_REV: %d", (_calib_direction < 0) ? 1 : 0);

	_calibration_mode = false;
}

void BoomControl::abort_calibration()
{
	_calib_state = CalibrationState::FAILED;
	_motor_output = 0.0f;
	_calibration_mode = false;
	_state = BoomState::ERROR;

	PX4_ERR("Boom calibration failed");
}

bool BoomControl::check_limit_sensors()
{
	hbridge_status_s hbridge_status;
	bool limit_triggered = false;
	uint8_t hbridge_channel = static_cast<uint8_t>(_param_hbridge_channel.get());

	// Check for hbridge status updates
	if (_hbridge_status_sub[hbridge_channel].updated() && _hbridge_status_sub[hbridge_channel].copy(&hbridge_status)) {
		// Check if forward or reverse limits are active
		if (hbridge_status.forward_limit || hbridge_status.reverse_limit) {
			limit_triggered = true;
		}
	}

	return limit_triggered;
}

float BoomControl::translate_as5600_to_boom_angle(float as5600_angle)
{
	if (_calib_state != CalibrationState::COMPLETED) {
		PX4_WARN("Calibration not completed - using uncalibrated angle");
		return as5600_angle;
	}

	// Normalize AS5600 angle to 0-1 range between calibrated limits
	float normalized = (as5600_angle - _calib_min_angle) / (_calib_max_angle - _calib_min_angle);

	// Map to boom angle range (assuming -10° to +75° boom range)
	float boom_min = _param_boom_angle_min.get();
	float boom_max = _param_boom_angle_max.get();
	float boom_angle = boom_min + normalized * (boom_max - boom_min);

	return boom_angle;
}

float BoomControl::translate_boom_to_as5600_angle(float boom_angle)
{
	if (_calib_state != CalibrationState::COMPLETED) {
		PX4_WARN("Calibration not completed - using uncalibrated angle");
		return boom_angle;
	}

	// Normalize boom angle to 0-1 range
	float boom_min = _param_boom_angle_min.get();
	float boom_max = _param_boom_angle_max.get();
	float normalized = (boom_angle - boom_min) / (boom_max - boom_min);

	// Map to AS5600 angle range
	float as5600_angle = _calib_min_angle + normalized * (_calib_max_angle - _calib_min_angle);

	return as5600_angle;
}
