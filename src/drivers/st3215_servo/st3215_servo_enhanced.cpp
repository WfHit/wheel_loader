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
 * @file st3215_servo_enhanced.cpp
 * @author PX4 Development Team
 *
 * Enhanced SCServo protocol functions for ST3215 smart servo
 */

#include "st3215_servo.hpp"

#include <px4_platform_common/log.h>
#include <lib/mathlib/mathlib.h>
#include <cmath>

bool ST3215Servo::write_position_ex(uint8_t servo_id, float position, float speed, uint8_t acceleration)
{
	// Convert position from radians to servo units (0-4095)
	float position_deg = position * 180.0f / M_PI_F; // Convert to degrees
	if (position_deg < 0) position_deg += 360.0f; // Ensure positive
	uint16_t servo_position = (uint16_t)(position_deg * 4095.0f / 360.0f);

	// Convert speed from rad/s to servo units
	float speed_deg_s = fabsf(speed) * 180.0f / M_PI_F; // Convert to deg/s
	uint16_t servo_speed = (uint16_t)(speed_deg_s / 0.114f); // 0.114 deg/s per unit

	// Support negative speeds (SCServo protocol uses bit 15 for direction)
	if (speed < 0) {
		servo_speed |= 0x8000; // Set direction bit
	}

	// Write acceleration first
	if (!write_byte(servo_id, ST3215_REG_ACC, acceleration)) {
		return false;
	}

	// Write position
	if (!write_word(servo_id, ST3215_REG_GOAL_POSITION_L, servo_position)) {
		return false;
	}

	// Write speed
	return write_word(servo_id, ST3215_REG_MOVING_SPEED_L, servo_speed);
}

bool ST3215Servo::set_wheel_mode(uint8_t servo_id)
{
	// Set angle limits to 0 to enable wheel mode
	if (!write_word(servo_id, ST3215_REG_CW_ANGLE_LIMIT_L, 0)) {
		return false;
	}
	return write_word(servo_id, ST3215_REG_CCW_ANGLE_LIMIT_L, 0);
}

bool ST3215Servo::write_speed(uint8_t servo_id, float speed, uint8_t acceleration)
{
	// Convert speed from rad/s to servo units
	float speed_deg_s = fabsf(speed) * 180.0f / M_PI_F; // Convert to deg/s
	uint16_t servo_speed = (uint16_t)(speed_deg_s / 0.114f); // 0.114 deg/s per unit

	// Support negative speeds (direction bit)
	if (speed < 0) {
		servo_speed |= 0x8000; // Set direction bit
	}

	// Write acceleration first
	if (!write_byte(servo_id, ST3215_REG_ACC, acceleration)) {
		return false;
	}

	// Write speed to moving speed register
	return write_word(servo_id, ST3215_REG_MOVING_SPEED_L, servo_speed);
}

bool ST3215Servo::set_torque_limit(uint8_t servo_id, uint16_t torque_limit)
{
	// Torque limit is in range 0-1023
	if (torque_limit > 1023) {
		torque_limit = 1023;
	}
	return write_word(servo_id, ST3215_REG_TORQUE_LIMIT_L, torque_limit);
}

bool ST3215Servo::feedback(uint8_t servo_id)
{
	// Read all feedback data at once using bulk read for efficiency
	// This is more efficient than individual reads
	uint8_t packet[9];
	packet[0] = ST3215_HEADER;
	packet[1] = ST3215_HEADER2;
	packet[2] = servo_id;
	packet[3] = 5; // Length
	packet[4] = ST3215_CMD_READ;
	packet[5] = ST3215_REG_PRESENT_POSITION_L; // Start register
	packet[6] = 10; // Read 10 bytes (position, speed, load, voltage, temp)
	packet[7] = calculate_checksum(packet + 2, 5);

	if (!send_packet(packet, sizeof(packet))) {
		return false;
	}

	px4_usleep(3000); // 3ms delay for bulk read

	int bytes_received = receive_packet(_rx_buffer, BUFFER_SIZE);

	if (bytes_received >= 16 && // Header + ID + Length + Error + 10 data bytes + checksum
	    _rx_buffer[0] == ST3215_HEADER &&
	    _rx_buffer[1] == ST3215_HEADER2 &&
	    _rx_buffer[2] == servo_id) {

		_error_status = _rx_buffer[4];
		if (_error_status == ST3215_ERROR_NONE) {
			// Parse feedback data
			uint16_t position = (_rx_buffer[6] << 8) | _rx_buffer[5];
			uint16_t speed = (_rx_buffer[8] << 8) | _rx_buffer[7];
			uint16_t load = (_rx_buffer[10] << 8) | _rx_buffer[9];
			uint8_t voltage = _rx_buffer[11];
			uint8_t temperature = _rx_buffer[12];

			// Convert and store values
			float position_deg = (float)position * 360.0f / 4095.0f;
			if (position_deg > 180.0f) position_deg -= 360.0f;
			_current_position = position_deg * M_PI_F / 180.0f;

			float speed_deg_s = (float)speed * 0.114f;
			_current_speed = speed_deg_s * M_PI_F / 180.0f;

			_current_load = (float)load / 10.24f;
			_current_voltage = voltage;
			_current_temperature = temperature;

			return true;
		}
	}

	return false;
}

bool ST3215Servo::set_position_pid(uint8_t servo_id, uint8_t p_gain, uint8_t i_gain, uint8_t d_gain)
{
	// Set P gain
	if (!write_byte(servo_id, ST3215_REG_P_GAIN, p_gain)) {
		return false;
	}

	// Set I gain
	if (!write_byte(servo_id, ST3215_REG_I_GAIN, i_gain)) {
		return false;
	}

	// Set D gain
	return write_byte(servo_id, ST3215_REG_D_GAIN, d_gain);
}

bool ST3215Servo::set_velocity_pid(uint8_t servo_id, uint8_t p_gain, uint8_t i_gain)
{
	// Example implementation: write velocity PID gains to servo registers
	bool ok = true;
	ok &= write_byte(servo_id, 0x1E, p_gain); // P gain
	ok &= write_byte(servo_id, 0x1F, i_gain); // I gain
	return ok;
}

bool ST3215Servo::set_compliance_params(uint8_t servo_id, uint8_t cw_margin, uint8_t ccw_margin, uint8_t cw_slope, uint8_t ccw_slope)
{
	// Example implementation: write compliance parameters to servo registers
	bool ok = true;
	ok &= write_byte(servo_id, 0x1A, cw_margin); // CW margin
	ok &= write_byte(servo_id, 0x1B, ccw_margin); // CCW margin
	ok &= write_byte(servo_id, 0x1C, cw_slope); // CW slope
	ok &= write_byte(servo_id, 0x1D, ccw_slope); // CCW slope
	return ok;
}

bool ST3215Servo::set_return_delay(uint8_t servo_id, uint8_t delay_time)
{
	// Return delay time in 2us units
	return write_byte(servo_id, ST3215_REG_RETURN_DELAY, delay_time);
}

bool ST3215Servo::set_status_return_level(uint8_t servo_id, uint8_t level)
{
	// Status return level: 0=none, 1=read only, 2=all
	if (level > 2) {
		level = 2;
	}
	return write_byte(servo_id, ST3215_REG_STATUS_RETURN_LEVEL, level);
}

bool ST3215Servo::set_alarm_led(uint8_t servo_id, uint8_t alarm_led)
{
	return write_byte(servo_id, ST3215_REG_ALARM_LED, alarm_led);
}

bool ST3215Servo::set_alarm_shutdown(uint8_t servo_id, uint8_t alarm_shutdown)
{
	return write_byte(servo_id, ST3215_REG_ALARM_SHUTDOWN, alarm_shutdown);
}

bool ST3215Servo::set_angle_limits(uint8_t servo_id, float min_angle, float max_angle)
{
	// Convert angles from radians to servo units
	float min_deg = min_angle * 180.0f / M_PI_F;
	float max_deg = max_angle * 180.0f / M_PI_F;

	// Ensure positive angles
	if (min_deg < 0) min_deg += 360.0f;
	if (max_deg < 0) max_deg += 360.0f;

	uint16_t min_servo = (uint16_t)(min_deg * 4095.0f / 360.0f);
	uint16_t max_servo = (uint16_t)(max_deg * 4095.0f / 360.0f);

	// Set minimum angle limit (CW)
	if (!write_word(servo_id, ST3215_REG_CW_ANGLE_LIMIT_L, min_servo)) {
		return false;
	}

	// Set maximum angle limit (CCW)
	return write_word(servo_id, ST3215_REG_CCW_ANGLE_LIMIT_L, max_servo);
}

bool ST3215Servo::set_voltage_limits(uint8_t servo_id, uint8_t min_voltage, uint8_t max_voltage)
{
	// Set minimum voltage limit (0.1V units)
	if (!write_byte(servo_id, ST3215_REG_MIN_VOLTAGE_LIMIT, min_voltage)) {
		return false;
	}

	// Set maximum voltage limit (0.1V units)
	return write_byte(servo_id, ST3215_REG_MAX_VOLTAGE_LIMIT, max_voltage);
}

bool ST3215Servo::set_temperature_limit(uint8_t servo_id, uint8_t max_temperature)
{
	// Set maximum temperature limit in Celsius
	return write_byte(servo_id, ST3215_REG_MAX_TEMP_LIMIT, max_temperature);
}
