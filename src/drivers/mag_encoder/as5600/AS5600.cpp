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
 *
 * Driver for the AS5600 magnetic rotary position sensor over I2C.
 *
 * @author PX4 Development Team
 */

#include "AS5600.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>

AS5600::AS5600(const I2CSPIDriverConfig &config) :
	I2C(config),
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
	_sample_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": read")),
	_comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comm_err"))
{
}

AS5600::~AS5600()
{
	stop();

	perf_free(_sample_perf);
	perf_free(_comms_errors);
}

int AS5600::init()
{
	int ret = I2C::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	if (ret == PX4_OK) {
		_device_id = get_device_id();
		start();
	}

	return ret;
}

int AS5600::probe()
{
	uint8_t status = 0;

	// Try to read the status register to verify device presence
	int ret = read_registers(AS5600_STATUS_REG, &status, 1);

	if (ret != PX4_OK) {
		DEVICE_DEBUG("probe failed");
		return ret;
	}

	return PX4_OK;
}

void AS5600::start()
{
	// Schedule the work item to run at 50Hz
	ScheduleOnInterval(20_ms);
}

void AS5600::stop()
{
	ScheduleClear();
}

void AS5600::RunImpl()
{
	perf_begin(_sample_perf);

	// Read sensor data
	bool success = read_angle_data() && read_status_data() && read_magnitude_data();

	if (!success) {
		perf_count(_comms_errors);
		_error_count++;
		perf_end(_sample_perf);
		return;
	}

	// Publish sensor data
	sensor_mag_encoder_s report{};

	const hrt_abstime timestamp_sample = hrt_absolute_time();
	report.timestamp_sample = timestamp_sample;
	report.device_id = _device_id;

	// Convert raw angle to radians
	report.raw_angle = static_cast<float>(_raw_angle);
	report.angle = static_cast<float>(_raw_angle) * AS5600_ANGLE_TO_RAD;

	// Apply offset parameter if configured
	if (_param_as5600_offset.get() != 0.0f) {
		report.angle += _param_as5600_offset.get();

		// Normalize to [0, 2*PI)
		while (report.angle < 0.0f) {
			report.angle += 2.0f * M_PI_F;
		}
		while (report.angle >= 2.0f * M_PI_F) {
			report.angle -= 2.0f * M_PI_F;
		}
	}

	// Status information
	report.magnet_detected = (_status & AS5600_STATUS_MAGNET_DETECTED) ? 1 : 0;
	report.magnet_too_strong = (_status & AS5600_STATUS_MAGNET_HIGH) ? 1 : 0;
	report.magnet_too_weak = (_status & AS5600_STATUS_MAGNET_LOW) ? 1 : 0;

	// Magnitude and AGC
	report.magnitude = static_cast<float>(_magnitude);
	report.automatic_gain_control = static_cast<float>(_agc);

	report.error_count = _error_count;
	report.timestamp = hrt_absolute_time();

	_sensor_mag_encoder_pub.publish(report);

	perf_end(_sample_perf);
}

bool AS5600::read_angle_data()
{
	uint8_t data[2];

	if (read_registers(AS5600_RAW_ANGLE_H_REG, data, 2) != PX4_OK) {
		return false;
	}

	_raw_angle = (static_cast<uint16_t>(data[0]) << 8) | data[1];
	return true;
}

bool AS5600::read_status_data()
{
	return read_registers(AS5600_STATUS_REG, &_status, 1) == PX4_OK;
}

bool AS5600::read_magnitude_data()
{
	uint8_t data[3];

	// Read AGC and Magnitude registers
	if (read_registers(AS5600_AGC_REG, data, 3) != PX4_OK) {
		return false;
	}

	_agc = data[0];
	_magnitude = (static_cast<uint16_t>(data[1]) << 8) | data[2];
	return true;
}

int AS5600::read_registers(uint8_t reg, uint8_t *data, uint8_t len)
{
	return transfer(&reg, 1, data, len);
}

int AS5600::write_register(uint8_t reg, uint8_t value)
{
	uint8_t cmd[2] = {reg, value};
	return transfer(cmd, sizeof(cmd), nullptr, 0);
}

void AS5600::print_status()
{
	I2CSPIDriverBase::print_status();

	perf_print_counter(_sample_perf);
	perf_print_counter(_comms_errors);

	printf("device_id: 0x%08X\n", _device_id);
	printf("error_count: %u\n", _error_count);
	printf("raw_angle: %u\n", _raw_angle);
	printf("magnitude: %u\n", _magnitude);
	printf("agc: %u\n", _agc);
	printf("status: 0x%02X\n", _status);
	printf("magnet_detected: %s\n", (_status & AS5600_STATUS_MAGNET_DETECTED) ? "yes" : "no");
	printf("magnet_too_strong: %s\n", (_status & AS5600_STATUS_MAGNET_HIGH) ? "yes" : "no");
	printf("magnet_too_weak: %s\n", (_status & AS5600_STATUS_MAGNET_LOW) ? "yes" : "no");
}

void AS5600::print_usage()
{
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description

Driver for the AS5600 magnetic rotary position sensor connected via I2C.

The sensor provides 12-bit angular position measurements and is commonly used for
measuring boom angles, gimbal positions, or other rotary applications.

### Examples

Attempt to start driver on any bus (start on bus where first sensor found).
$ as5600 start

Start driver on specified bus
$ as5600 start -X

Stop driver
$ as5600 stop

Test driver (start if not running)
$ as5600 test

Reset driver
$ as5600 reset

Print driver information
$ as5600 info
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("as5600", "driver");
	PRINT_MODULE_USAGE_SUBCATEGORY("mag_encoder");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAMS_I2C_SPI_DRIVER(true, false);
	PRINT_MODULE_USAGE_COMMAND("stop");
	PRINT_MODULE_USAGE_COMMAND("test");
	PRINT_MODULE_USAGE_COMMAND("reset");
	PRINT_MODULE_USAGE_COMMAND("info");
}
