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
 * @file AS5600.cpp
 * @author PX4 Development Team
 *
 * Driver for the AS5600 magnetic rotary position sensor connected via I2C.
 */

#include "AS5600.h"
#include <px4_platform_common/module.h>
#include <drivers/drv_sensor.h>
#include <cstring>

AS5600::AS5600(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": single-sample")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comms errors"))
{
	_sensor_mag_encoder.device_id = this->get_device_id();
}

AS5600::~AS5600()
{
	ScheduleClear();
	perf_free(_cycle_perf);
	perf_free(_comms_errors);
}

void AS5600::exit_and_cleanup()
{
	I2CSPIDriverBase::exit_and_cleanup();
}

void AS5600::RunImpl()
{
	if (should_exit()) {
		PX4_INFO("stopping");
		return;
	}

	perf_begin(_cycle_perf);

	// PX4_INFO("AS5600: Reading sensor data...");

	const bool angle_ready = readAngle();
	const bool status_ready = readStatus();
	const bool magnitude_ready = readMagnitude();

	// PX4_INFO("AS5600: Read results - angle:%s, status:%s, magnitude:%s",
	// 	angle_ready ? "OK" : "FAIL",
	// 	status_ready ? "OK" : "FAIL",
	// 	magnitude_ready ? "OK" : "FAIL");

	if (angle_ready && status_ready && magnitude_ready) {
		// All sensor data successfully read
		if (_ready_counter == 0) {
			PX4_INFO("AS5600: Device ready - first successful read");
		}

		if (_ready_counter < MAX_READY_COUNTER) { _ready_counter++; }

		// Log every 100th reading for debugging
		static uint32_t log_counter = 0;
		if (++log_counter % 100 == 0) {
			double angle_deg = (_raw_angle * 360.0) / 4095.0;
			PX4_INFO("AS5600: angle=%.2f deg (raw:%u), mag=%u, agc=%u, status=0x%02X",
				angle_deg, _raw_angle, _magnitude, _agc, _status);
		}		// Prepare sensor report
		const hrt_abstime timestamp_sample = hrt_absolute_time();
		_sensor_mag_encoder.timestamp_sample = timestamp_sample;
		_sensor_mag_encoder.raw_angle = _raw_angle;
		_sensor_mag_encoder.angle = _raw_angle * AS5600_ANGLE_TO_RAD;

		// Status information
		_sensor_mag_encoder.magnet_detected = (_status & AS5600_STATUS_MAGNET_DETECTED) ? 1 : 0;
		_sensor_mag_encoder.magnet_too_strong = (_status & AS5600_STATUS_MAGNET_HIGH) ? 1 : 0;
		_sensor_mag_encoder.magnet_too_weak = (_status & AS5600_STATUS_MAGNET_LOW) ? 1 : 0;

		// Additional sensor data
		_sensor_mag_encoder.magnitude = _magnitude;
		_sensor_mag_encoder.automatic_gain_control = _agc;
		_sensor_mag_encoder.error_count = perf_event_count(_comms_errors);

		_sensor_mag_encoder.timestamp = hrt_absolute_time();

		_sensor_mag_encoder_pub.publish(_sensor_mag_encoder);

	} else {
		// Communication error
		PX4_INFO("AS5600: Communication error - failed to read sensor data");

		if (_ready_counter == 1) {
			PX4_ERR("AS5600: Device lost - communication failure");
		}

		if (_ready_counter > 0) { _ready_counter--; }

		perf_count(_comms_errors);

		// Log communication errors periodically
		static uint32_t error_log_counter = 0;
		if (++error_log_counter % 50 == 0) {
			PX4_WARN("AS5600: %lu consecutive communication errors", error_log_counter);
		}
	}

	perf_end(_cycle_perf);
}

void AS5600::print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Driver for the AS5600 magnetic rotary position sensor connected via I2C.

The AS5600 is a 12-bit contactless magnetic rotary position sensor with analog and PWM outputs.
This driver provides angle measurements in radians and status information about magnet detection.

)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("as5600", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_PARAMS_I2C_ADDRESS(0x36);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
}

void AS5600::print_status()
{
	I2CSPIDriverBase::print_status();
	perf_print_counter(_cycle_perf);
	perf_print_counter(_comms_errors);

	double angle_deg = (_raw_angle * 360.0) / 4095.0;
	PX4_INFO("Angle: %.2f deg (raw: %u)", angle_deg, _raw_angle);
	PX4_INFO("Magnitude: %u", _magnitude);
	PX4_INFO("AGC: %u", _agc);
	PX4_INFO("Status: 0x%02x", _status);

	if (_status & AS5600_STATUS_MAGNET_DETECTED) {
		PX4_INFO("Magnet: DETECTED");

		if (_status & AS5600_STATUS_MAGNET_HIGH) {
			PX4_WARN("Magnet: TOO STRONG");
		} else if (_status & AS5600_STATUS_MAGNET_LOW) {
			PX4_WARN("Magnet: TOO WEAK");
		} else {
			PX4_INFO("Magnet: OK");
		}
	} else {
		PX4_WARN("Magnet: NOT DETECTED");
	}
}

int AS5600::init()
{
	PX4_INFO("AS5600: Initializing magnetic encoder");

	int ret = I2C::init();

	if (ret != PX4_OK) {
		PX4_ERR("AS5600: I2C initialization failed: %d", ret);
		return ret;
	}

	PX4_INFO("AS5600: I2C initialized successfully");
	ScheduleOnInterval(SAMPLE_INTERVAL, SAMPLE_INTERVAL);
	PX4_INFO("AS5600: Scheduled with interval %llu us", SAMPLE_INTERVAL);

	return PX4_OK;
}

int AS5600::probe()
{
	PX4_INFO("AS5600: Probing device...");

	// Try to read the status register to verify the device is present
	uint8_t status;
	int ret = readReg(AS5600_STATUS_REG, &status, 1);

	if (ret != PX4_OK) {
		PX4_ERR("AS5600: Probe failed - cannot read status register (ret: %d)", ret);
		DEVICE_DEBUG("AS5600 not found");
		return ret;
	}

	PX4_INFO("AS5600: Device found! Initial status: 0x%02X", status);

	// Log initial magnet status
	if (status & AS5600_STATUS_MAGNET_DETECTED) {
		PX4_INFO("AS5600: Magnet detected");
		if (status & AS5600_STATUS_MAGNET_HIGH) {
			PX4_WARN("AS5600: Magnet too strong");
		} else if (status & AS5600_STATUS_MAGNET_LOW) {
			PX4_WARN("AS5600: Magnet too weak");
		} else {
			PX4_INFO("AS5600: Magnet strength OK");
		}
	} else {
		PX4_WARN("AS5600: No magnet detected");
	}

	return PX4_OK;
}

bool AS5600::readAngle()
{
	uint8_t data[2];
	int ret = readReg(AS5600_RAW_ANGLE_H_REG, data, 2);

	if (ret == PX4_OK) {
		uint16_t prev_angle = _raw_angle;
		_raw_angle = (data[0] << 8) | data[1];

		// Log significant angle changes
		if (abs((int)_raw_angle - (int)prev_angle) > 100)
		{
			PX4_INFO("AS5600: Large angle change - prev:%d, new:%d", prev_angle, _raw_angle);
		}

		return true;
	} else {
		PX4_INFO("AS5600: Failed to read angle register (ret: %d)", ret);
	}

	return false;
}

bool AS5600::readStatus()
{
	uint8_t prev_status = _status;
	int ret = readReg(AS5600_STATUS_REG, &_status, 1);

	if (ret == PX4_OK) {
		// Log status changes
		if (prev_status != _status) {
			PX4_INFO("AS5600: Status changed from 0x%02X to 0x%02X", prev_status, _status);

			// Log specific magnet status changes
			bool prev_detected = prev_status & AS5600_STATUS_MAGNET_DETECTED;
			bool curr_detected = _status & AS5600_STATUS_MAGNET_DETECTED;

			if (prev_detected != curr_detected) {
				if (curr_detected) {
					PX4_INFO("AS5600: Magnet detected");
				} else {
					PX4_WARN("AS5600: Magnet lost");
				}
			}

			// Check magnet strength changes
			if (curr_detected) {
				bool prev_high = prev_status & AS5600_STATUS_MAGNET_HIGH;
				bool curr_high = _status & AS5600_STATUS_MAGNET_HIGH;
				bool prev_low = prev_status & AS5600_STATUS_MAGNET_LOW;
				bool curr_low = _status & AS5600_STATUS_MAGNET_LOW;

				if (!prev_high && curr_high) {
					PX4_WARN("AS5600: Magnet too strong");
				} else if (!prev_low && curr_low) {
					PX4_WARN("AS5600: Magnet too weak");
				} else if ((prev_high || prev_low) && !(curr_high || curr_low)) {
					PX4_INFO("AS5600: Magnet strength OK");
				}
			}
		}
		return true;
	} else {
		PX4_INFO("AS5600: Failed to read status register (ret: %d)", ret);
	}

	return false;
}

bool AS5600::readMagnitude()
{
	uint8_t data[2];
	int ret = readReg(AS5600_MAGNITUDE_H_REG, data, 2);

	if (ret == PX4_OK) {
		uint16_t prev_magnitude = _magnitude;
		_magnitude = (data[0] << 8) | data[1];

		// Also read AGC value
		uint8_t prev_agc = _agc;
		ret = readReg(AS5600_AGC_REG, &_agc, 1);

		if (ret == PX4_OK) {
			// Log significant magnitude or AGC changes
			if (abs((int)_magnitude - (int)prev_magnitude) > 50) {
				PX4_INFO("AS5600: Magnitude change - prev:%d, new:%d", prev_magnitude, _magnitude);
			}
			if (abs((int)_agc - (int)prev_agc) > 10) {
				PX4_INFO("AS5600: AGC change - prev:%u, new:%u", prev_agc, _agc);
			}
			return true;
		} else {
			PX4_INFO("AS5600: Failed to read AGC register (ret: %d)", ret);
		}
	} else {
		PX4_INFO("AS5600: Failed to read magnitude register (ret: %d)", ret);
	}

	return false;
}

int AS5600::readReg(uint8_t addr, uint8_t *buf, size_t len)
{
	int ret = transfer(&addr, 1, buf, len);

	if (ret != PX4_OK) {
		PX4_INFO("AS5600: I2C transfer failed - addr:0x%02X, len:%zu, ret:%d", addr, len, ret);

		// Log specific register read failures
		static uint32_t fail_counter = 0;
		if (++fail_counter % 20 == 0) {
			PX4_WARN("AS5600: %lu I2C transfer failures", fail_counter);
		}
	}

	return ret;
}

int AS5600::writeReg(uint8_t addr, uint8_t *buf, size_t len)
{
	uint8_t cmd[len + 1];
	cmd[0] = addr;
	memcpy(&cmd[1], buf, len);

	int ret = transfer(cmd, len + 1, nullptr, 0);

	if (ret != PX4_OK) {
		PX4_INFO("AS5600: I2C write failed - addr:0x%02X, len:%zu, ret:%d", addr, len, ret);
	} else {
		PX4_INFO("AS5600: I2C write success - addr:0x%02X, len:%zu", addr, len);
	}

	return ret;
}
