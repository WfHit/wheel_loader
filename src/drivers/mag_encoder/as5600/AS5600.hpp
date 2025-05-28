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
 * @file AS5600.hpp
 *
 * Driver for the AS5600 magnetic rotary position sensor over I2C.
 *
 * @author PX4 Development Team
 */

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/drivers/device/i2c.h>
#include <lib/perf/perf_counter.h>
#include <uORB/PublicationMulti.hpp>
#include <uORB/topics/sensor_mag_encoder.h>

/* AS5600 Registers */
#define AS5600_ZMCO_REG			0x00  // ZMCO Register
#define AS5600_ZPOS_H_REG		0x01  // Zero Position High Byte
#define AS5600_ZPOS_L_REG		0x02  // Zero Position Low Byte
#define AS5600_MPOS_H_REG		0x03  // Maximum Position High Byte
#define AS5600_MPOS_L_REG		0x04  // Maximum Position Low Byte
#define AS5600_MANG_H_REG		0x05  // Maximum Angle High Byte
#define AS5600_MANG_L_REG		0x06  // Maximum Angle Low Byte
#define AS5600_CONF_H_REG		0x07  // Configuration High Byte
#define AS5600_CONF_L_REG		0x08  // Configuration Low Byte
#define AS5600_RAW_ANGLE_H_REG		0x0C  // Raw Angle High Byte
#define AS5600_RAW_ANGLE_L_REG		0x0D  // Raw Angle Low Byte
#define AS5600_ANGLE_H_REG		0x0E  // Angle High Byte
#define AS5600_ANGLE_L_REG		0x0F  // Angle Low Byte
#define AS5600_STATUS_REG		0x0B  // Status Register
#define AS5600_AGC_REG			0x1A  // Automatic Gain Control
#define AS5600_MAGNITUDE_H_REG		0x1B  // Magnitude High Byte
#define AS5600_MAGNITUDE_L_REG		0x1C  // Magnitude Low Byte
#define AS5600_BURN_REG			0xFF  // Burn Commands

/* AS5600 Status Register Bits */
#define AS5600_STATUS_MAGNET_HIGH	(1 << 3)  // Magnet too strong
#define AS5600_STATUS_MAGNET_LOW	(1 << 4)  // Magnet too weak
#define AS5600_STATUS_MAGNET_DETECTED	(1 << 5)  // Magnet detected

/* AS5600 I2C Address */
#define AS5600_I2C_ADDR			0x36

/* AS5600 Constants */
#define AS5600_MAX_ANGLE_VALUE		4095  // 12-bit resolution
#define AS5600_ANGLE_TO_RAD		(2.0f * M_PI_F / AS5600_MAX_ANGLE_VALUE)

class AS5600 : public device::I2C, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	AS5600(const I2CSPIDriverConfig &config);
	virtual ~AS5600();

	static void print_usage();
	int init() override;
	void print_status() override;

private:
	void start();
	void stop();
	void RunImpl();

	int probe() override;
	int read_registers(uint8_t reg, uint8_t *data, uint8_t len);
	int write_register(uint8_t reg, uint8_t value);

	bool read_angle_data();
	bool read_status_data();
	bool read_magnitude_data();

	uint32_t _device_id{0};

	uORB::PublicationMulti<sensor_mag_encoder_s> _sensor_mag_encoder_pub{ORB_ID(sensor_mag_encoder)};

	perf_counter_t _sample_perf;
	perf_counter_t _comms_errors;

	uint32_t _error_count{0};

	// Sensor data
	uint16_t _raw_angle{0};
	uint16_t _magnitude{0};
	uint8_t _agc{0};
	uint8_t _status{0};

	DEFINE_PARAMETERS(
		(ParamInt<px4::params::SENS_EN_AS5600>) _param_sens_en_as5600,
		(ParamInt<px4::params::AS5600_I2C_BUS>) _param_as5600_i2c_bus,
		(ParamFloat<px4::params::AS5600_OFFSET>) _param_as5600_offset
	)
};
