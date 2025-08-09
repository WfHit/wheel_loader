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

#pragma once

#include <px4_platform_common/module_params.h>
#include <drivers/drv_hrt.h>
#include <uORB/Subscription.hpp>
#include <uORB/topics/sensor_mag_encoder.h>

/**
 * @brief Sensor interface for boom position feedback
 *
 * Abstracts sensor reading and processing:
 * - AS5600 magnetic encoder interface
 * - Calibration and scaling
 * - Fault detection and validation
 */
class BoomSensorInterface : public ModuleParams
{
public:
	static constexpr hrt_abstime SENSOR_TIMEOUT_US = 500000; // 500ms

	struct SensorData {
		float raw_angle;           // degrees - Raw encoder reading
		float calibrated_angle;    // degrees - After calibration
		float actuator_length;     // mm - Computed actuator length
		bool is_valid;            // Sensor data validity
		bool magnet_detected;     // Magnet presence
		uint8_t status_flags;     // Sensor status flags
		hrt_abstime timestamp;    // Data timestamp
	};

	explicit BoomSensorInterface(ModuleParams *parent);

	/**
	 * @brief Initialize sensor interface
	 * @param encoder_instance AS5600 instance to use
	 * @return True if initialization successful
	 */
	bool initialize(int encoder_instance);

	/**
	 * @brief Update sensor readings
	 * @param data Output sensor data structure
	 * @return True if new valid data available
	 */
	bool update(SensorData &data);

	/**
	 * @brief Perform sensor calibration
	 * @param reference_angle Known reference angle in degrees
	 * @return True if calibration successful
	 */
	bool calibrate(float reference_angle);

	/**
	 * @brief Reset sensor to default calibration
	 */
	void reset_calibration();

	/**
	 * @brief Check sensor health
	 * @return True if sensor is healthy
	 */
	bool is_healthy() const;

	/**
	 * @brief Get time since last valid reading
	 * @return Time in microseconds
	 */
	hrt_abstime time_since_last_update() const;

private:
	// Sensor configuration
	int _encoder_instance{-1};
	float _calibration_scale{1.0f};
	float _calibration_offset{0.0f};
	bool _angle_reversed{false};

	// Sensor state
	SensorData _last_data{};
	bool _initialized{false};
	hrt_abstime _last_update_time{0};

	// uORB subscription
	uORB::Subscription _mag_encoder_sub{ORB_ID(sensor_mag_encoder)};

	// Calibration parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::BOOM_ENC_INST>) _param_encoder_instance,
		(ParamFloat<px4::params::BOOM_ENC_SCALE>) _param_encoder_scale,
		(ParamFloat<px4::params::BOOM_ENC_OFF>) _param_encoder_offset,
		(ParamInt<px4::params::BOOM_ENC_REV>) _param_angle_reverse
	)

	/**
	 * @brief Apply calibration to raw angle
	 * @param raw_angle Raw angle in degrees
	 * @return Calibrated angle in degrees
	 */
	float apply_calibration(float raw_angle) const;

	/**
	 * @brief Validate sensor data
	 * @param data Sensor data to validate
	 * @return True if data is valid
	 */
	bool validate_data(const SensorData &data) const;
};
