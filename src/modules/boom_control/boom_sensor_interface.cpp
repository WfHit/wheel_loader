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

#include "boom_sensor_interface.hpp"
#include <px4_platform_common/log.h>

BoomSensorInterface::BoomSensorInterface(ModuleParams* parent) :
	ModuleParams(parent)
{
}

bool BoomSensorInterface::initialize(int encoder_instance)
{
	_encoder_instance = encoder_instance;
	_initialized = true;
	updateParams();

	// Configure parameters
	_calibration_scale = _param_encoder_scale.get();
	_calibration_offset = _param_encoder_offset.get();
	_angle_reversed = (_param_angle_reverse.get() != 0);

	PX4_INFO("Boom sensor interface initialized (encoder instance: %d)", _encoder_instance);
	return true;
}

bool BoomSensorInterface::update(SensorData& data)
{
	if (!_initialized) {
		return false;
	}

	sensor_mag_encoder_s encoder_msg;
	if (_mag_encoder_sub.update(&encoder_msg)) {
		// Process new encoder data
		data.raw_angle = encoder_msg.angle;
		data.calibrated_angle = apply_calibration(data.raw_angle);
		data.magnet_detected = encoder_msg.magnet_detected;
		data.status_flags = 0; // encoder_msg doesn't have status field, use 0
		data.timestamp = encoder_msg.timestamp;
		data.is_valid = validate_data(data);

		// Simple conversion - this would be done through kinematics in real system
		data.actuator_length = data.calibrated_angle * 5.0f; // Placeholder conversion

		_last_data = data;
		_last_update_time = hrt_absolute_time();
		return true;
	}

	return false;
}

bool BoomSensorInterface::calibrate(float reference_angle)
{
	// Simple calibration - set offset to match reference
	_calibration_offset = reference_angle - _last_data.raw_angle;
	PX4_INFO("Sensor calibrated: offset = %.2f deg", (double)_calibration_offset);
	return true;
}

void BoomSensorInterface::reset_calibration()
{
	_calibration_scale = 1.0f;
	_calibration_offset = 0.0f;
	PX4_INFO("Sensor calibration reset");
}

bool BoomSensorInterface::is_healthy() const
{
	if (!_initialized) {
		return false;
	}

	// Check for recent valid data
	if (time_since_last_update() > SENSOR_TIMEOUT_US) {
		return false;
	}

	return _last_data.is_valid && _last_data.magnet_detected;
}

hrt_abstime BoomSensorInterface::time_since_last_update() const
{
	return hrt_elapsed_time(&_last_update_time);
}

float BoomSensorInterface::apply_calibration(float raw_angle) const
{
	float calibrated = raw_angle * _calibration_scale + _calibration_offset;
	if (_angle_reversed) {
		calibrated = -calibrated;
	}
	return calibrated;
}

bool BoomSensorInterface::validate_data(const SensorData& data) const
{
	// Basic validation
	if (!data.magnet_detected) {
		return false;
	}

	// Check for reasonable angle range
	if (fabsf(data.calibrated_angle) > 360.0f) {
		return false;
	}

	return true;
}
