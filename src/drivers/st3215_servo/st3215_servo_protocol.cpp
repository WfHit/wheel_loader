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
 * @file st3215_servo_protocol.cpp
 * @author PX4 Development Team
 *
 * Communication protocol functions for ST3215 smart servo
 */

#include "st3215_servo.hpp"

#include <px4_platform_common/log.h>
#include <cstring>
#include <cerrno>
#include <termios.h>

bool ST3215Servo::open_serial_port()
{
	PX4_INFO("Attempting to open serial port: %s", _serial_port);

	_serial_fd = ::open(_serial_port, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_serial_fd < 0) {
		PX4_ERR("Failed to open %s: %s", _serial_port, strerror(errno));
		return false;
	}

	PX4_DEBUG("Serial port opened successfully, fd: %d", _serial_fd);

	// Configure serial port
	struct termios tty;

	if (tcgetattr(_serial_fd, &tty) != 0) {
		PX4_ERR("Failed to get serial attributes: %s", strerror(errno));
		close_serial_port();
		return false;
	}

	// Set baud rate from parameter
	speed_t baud_rate;
	int32_t param_baud = _param_baudrate.get();

	switch (param_baud) {
	case 9600:
		baud_rate = B9600;
		break;
	case 19200:
		baud_rate = B19200;
		break;
	case 38400:
		baud_rate = B38400;
		break;
	case 57600:
		baud_rate = B57600;
		break;
	case 115200:
		baud_rate = B115200;
		break;
	case 1000000:
		baud_rate = B1000000;
		break;
	default:
		PX4_WARN("Unsupported baud rate %ld, using 1000000", param_baud);
		baud_rate = B1000000;
		param_baud = 1000000;
		break;
	}

	if (cfsetospeed(&tty, baud_rate) != 0 || cfsetispeed(&tty, baud_rate) != 0) {
		PX4_ERR("Failed to set baud rate to %ld", param_baud);
		close_serial_port();
		return false;
	}

	// 8N1
	tty.c_cflag &= ~PARENB;  // No parity
	tty.c_cflag &= ~CSTOPB;  // One stop bit
	tty.c_cflag &= ~CSIZE;   // Clear size mask
	tty.c_cflag |= CS8;      // 8 data bits

	// No flow control
	tty.c_cflag &= ~CRTSCTS;

	// Enable receiver, ignore modem control lines
	tty.c_cflag |= CREAD | CLOCAL;

	// Disable canonical mode, echo, echoe, and echok
	tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ECHOK);

	// Disable software flow control
	tty.c_iflag &= ~(IXON | IXOFF | IXANY);

	// Raw output
	tty.c_oflag &= ~OPOST;

	// Set timeouts
	tty.c_cc[VMIN] = 0;   // Non-blocking
	tty.c_cc[VTIME] = 1;  // 100ms timeout

	if (tcsetattr(_serial_fd, TCSANOW, &tty) != 0) {
		PX4_ERR("Failed to set serial attributes: %s", strerror(errno));
		close_serial_port();
		return false;
	}

	// Flush any existing data
	tcflush(_serial_fd, TCIOFLUSH);

	PX4_INFO("Opened %s successfully (%ld baud, 8N1)", _serial_port, param_baud);
	return true;
}

void ST3215Servo::close_serial_port()
{
	if (_serial_fd >= 0) {
		::close(_serial_fd);
		_serial_fd = -1;
	}
}

bool ST3215Servo::send_packet(const uint8_t *data, uint8_t length)
{
	if (_serial_fd < 0) {
		PX4_ERR("Serial port not open, fd = %d", _serial_fd);
		return false;
	}

	ssize_t bytes_written = ::write(_serial_fd, data, length);

	if (bytes_written < 0) {
		PX4_ERR("Write failed: expected %d bytes, wrote %d bytes (%s)",
			length, (int)bytes_written, strerror(errno));
		perf_count(_comms_error_perf);

		// If we get EBADF (Bad file descriptor), the connection is lost
		if (errno == EBADF || errno == ENOTCONN) {
			PX4_ERR("Serial connection lost, closing port");
			close_serial_port();
		}
		return false;
	}

	if (bytes_written != length) {
		PX4_WARN("Partial write: expected %d bytes, wrote %d bytes",
			length, (int)bytes_written);
		perf_count(_comms_error_perf);
		return false;
	}

	// Force transmission
	if (tcdrain(_serial_fd) != 0) {
		PX4_WARN("Failed to drain serial port: %s", strerror(errno));
	}

	PX4_DEBUG("Sent %d bytes successfully", length);
	return true;
}

int ST3215Servo::receive_packet(uint8_t *buffer, uint8_t max_length)
{
	if (_serial_fd < 0) {
		PX4_ERR("Serial port not open, fd = %d", _serial_fd);
		return -1;
	}

	// Clear input buffer first
	memset(buffer, 0, max_length);

	ssize_t bytes_read = ::read(_serial_fd, buffer, max_length);

	if (bytes_read < 0) {
		if (errno != EAGAIN && errno != EWOULDBLOCK) {
			PX4_WARN("Read failed: %s", strerror(errno));
			perf_count(_comms_error_perf);

			// If we get EBADF (Bad file descriptor), the connection is lost
			if (errno == EBADF || errno == ENOTCONN) {
				PX4_ERR("Serial connection lost during read, closing port");
				close_serial_port();
			}
		}
		return -1;
	}

	PX4_DEBUG("Received %d bytes", (int)bytes_read);
	return (int)bytes_read;
}

uint8_t ST3215Servo::calculate_checksum(const uint8_t *data, uint8_t length)
{
	uint8_t checksum = 0;

	for (uint8_t i = 0; i < length; i++) {
		checksum += data[i];
	}

	return ~checksum;
}

bool ST3215Servo::ping_servo(uint8_t servo_id)
{
	uint8_t packet[6];
	packet[0] = ST3215_HEADER;
	packet[1] = ST3215_HEADER2;
	packet[2] = servo_id;
	packet[3] = 2; // Length
	packet[4] = ST3215_CMD_PING;
	packet[5] = calculate_checksum(packet + 2, 3);

	PX4_INFO("Sending ping to servo ID %d", servo_id);
	PX4_INFO("Ping packet: %02X %02X %02X %02X %02X %02X",
		packet[0], packet[1], packet[2], packet[3], packet[4], packet[5]);

	if (!send_packet(packet, sizeof(packet))) {
		PX4_WARN("Failed to send ping packet");
		return false;
	}

	// Wait for response
	px4_usleep(2000); // 2ms delay

	int bytes_received = receive_packet(_rx_buffer, BUFFER_SIZE);

	PX4_DEBUG("Ping response: %d bytes received", bytes_received);
	if (bytes_received > 0) {
		PX4_DEBUG("Response packet:");
		for (int i = 0; i < bytes_received; i++) {
			PX4_DEBUG("  [%d]: 0x%02X", i, _rx_buffer[i]);
		}
	}

	// Response should be: FF FF ID 02 00 ~(ID+02+00)
	if (bytes_received >= 6 &&
	    _rx_buffer[0] == ST3215_HEADER &&
	    _rx_buffer[1] == ST3215_HEADER2 &&
	    _rx_buffer[2] == servo_id) {
		_error_status = _rx_buffer[4];
		PX4_DEBUG("Ping successful, error status: 0x%02X", _error_status);
		return true;
	}

	PX4_WARN("Ping failed - invalid response format");
	if (bytes_received >= 3) {
		PX4_WARN("Expected header: 0xFF 0xFF 0x%02X, got: 0x%02X 0x%02X 0x%02X",
			servo_id, _rx_buffer[0], _rx_buffer[1], _rx_buffer[2]);
	}

	return false;
}

bool ST3215Servo::read_byte(uint8_t servo_id, uint8_t reg_addr, uint8_t *value)
{
	uint8_t packet[8];
	packet[0] = ST3215_HEADER;
	packet[1] = ST3215_HEADER2;
	packet[2] = servo_id;
	packet[3] = 4; // Length
	packet[4] = ST3215_CMD_READ;
	packet[5] = reg_addr;
	packet[6] = 1; // Read 1 byte
	packet[7] = calculate_checksum(packet + 2, 5);

	if (!send_packet(packet, sizeof(packet))) {
		return false;
	}

	px4_usleep(2000); // 2ms delay

	int bytes_received = receive_packet(_rx_buffer, BUFFER_SIZE);

	if (bytes_received >= 7 &&
	    _rx_buffer[0] == ST3215_HEADER &&
	    _rx_buffer[1] == ST3215_HEADER2 &&
	    _rx_buffer[2] == servo_id) {
		_error_status = _rx_buffer[4];
		if (_error_status == ST3215_ERROR_NONE) {
			*value = _rx_buffer[5];
			return true;
		}
	}

	return false;
}

bool ST3215Servo::read_word(uint8_t servo_id, uint8_t reg_addr, uint16_t *value)
{
	uint8_t packet[8];
	packet[0] = ST3215_HEADER;
	packet[1] = ST3215_HEADER2;
	packet[2] = servo_id;
	packet[3] = 4; // Length
	packet[4] = ST3215_CMD_READ;
	packet[5] = reg_addr;
	packet[6] = 2; // Read 2 bytes
	packet[7] = calculate_checksum(packet + 2, 5);

	if (!send_packet(packet, sizeof(packet))) {
		return false;
	}

	px4_usleep(2000); // 2ms delay

	int bytes_received = receive_packet(_rx_buffer, BUFFER_SIZE);

	if (bytes_received >= 8 &&
	    _rx_buffer[0] == ST3215_HEADER &&
	    _rx_buffer[1] == ST3215_HEADER2 &&
	    _rx_buffer[2] == servo_id) {
		_error_status = _rx_buffer[4];
		if (_error_status == ST3215_ERROR_NONE) {
			*value = (_rx_buffer[6] << 8) | _rx_buffer[5];
			return true;
		}
	}

	return false;
}

bool ST3215Servo::write_byte(uint8_t servo_id, uint8_t reg_addr, uint8_t value)
{
	uint8_t packet[8];
	packet[0] = ST3215_HEADER;
	packet[1] = ST3215_HEADER2;
	packet[2] = servo_id;
	packet[3] = 4; // Length
	packet[4] = ST3215_CMD_WRITE;
	packet[5] = reg_addr;
	packet[6] = value;
	packet[7] = calculate_checksum(packet + 2, 5);

	if (!send_packet(packet, sizeof(packet))) {
		return false;
	}

	px4_usleep(2000); // 2ms delay

	int bytes_received = receive_packet(_rx_buffer, BUFFER_SIZE);

	if (bytes_received >= 6 &&
	    _rx_buffer[0] == ST3215_HEADER &&
	    _rx_buffer[1] == ST3215_HEADER2 &&
	    _rx_buffer[2] == servo_id) {
		_error_status = _rx_buffer[4];
		return (_error_status == ST3215_ERROR_NONE);
	}

	return false;
}

bool ST3215Servo::write_word(uint8_t servo_id, uint8_t reg_addr, uint16_t value)
{
	uint8_t packet[9];
	packet[0] = ST3215_HEADER;
	packet[1] = ST3215_HEADER2;
	packet[2] = servo_id;
	packet[3] = 5; // Length
	packet[4] = ST3215_CMD_WRITE;
	packet[5] = reg_addr;
	packet[6] = value & 0xFF;       // Low byte
	packet[7] = (value >> 8) & 0xFF; // High byte
	packet[8] = calculate_checksum(packet + 2, 6);

	if (!send_packet(packet, sizeof(packet))) {
		return false;
	}

	px4_usleep(2000); // 2ms delay

	int bytes_received = receive_packet(_rx_buffer, BUFFER_SIZE);

	if (bytes_received >= 6 &&
	    _rx_buffer[0] == ST3215_HEADER &&
	    _rx_buffer[1] == ST3215_HEADER2 &&
	    _rx_buffer[2] == servo_id) {
		_error_status = _rx_buffer[4];
		return (_error_status == ST3215_ERROR_NONE);
	}

	return false;
}

bool ST3215Servo::set_torque_enable(uint8_t servo_id, bool enable)
{
	return write_byte(servo_id, ST3215_REG_TORQUE_ENABLE, enable ? 1 : 0);
}

bool ST3215Servo::set_led(uint8_t servo_id, uint8_t state)
{
	return write_byte(servo_id, ST3215_REG_LED, state);
}

bool ST3215Servo::read_full_status(uint8_t servo_id)
{
	uint16_t position, speed, load;
	uint8_t voltage, temperature, moving;

	// Read position
	if (read_word(servo_id, ST3215_REG_PRESENT_POSITION_L, &position)) {
		// Convert from servo units (0-4095) to radians (-π to π)
		float position_deg = (float)position * 360.0f / 4095.0f;
		if (position_deg > 180.0f) position_deg -= 360.0f;
		_current_position = position_deg * M_PI_F / 180.0f;
	}

	// Read speed
	if (read_word(servo_id, ST3215_REG_PRESENT_SPEED_L, &speed)) {
		// Convert speed to rad/s (speed unit depends on servo configuration)
		float speed_deg_s = (float)speed * 0.114f; // 0.114 deg/s per unit
		_current_speed = speed_deg_s * M_PI_F / 180.0f;
	}

	// Read load
	if (read_word(servo_id, ST3215_REG_PRESENT_LOAD_L, &load)) {
		_current_load = (float)load / 10.24f; // Convert to percentage
	}

	// Read voltage
	if (read_byte(servo_id, ST3215_REG_PRESENT_VOLTAGE, &voltage)) {
		_current_voltage = voltage; // 0.1V units
	}

	// Read temperature
	if (read_byte(servo_id, ST3215_REG_PRESENT_TEMP, &temperature)) {
		_current_temperature = temperature;
	}

	// Read moving status
	if (read_byte(servo_id, ST3215_REG_MOVING, &moving)) {
		_is_moving = (moving != 0);
	}

	// Read torque enable status
	uint8_t torque_enable;
	if (read_byte(servo_id, ST3215_REG_TORQUE_ENABLE, &torque_enable)) {
		_servo_enabled = (torque_enable != 0);
	}

	return true;
}

bool ST3215Servo::set_servo_id(uint8_t current_id, uint8_t new_id)
{
	return write_byte(current_id, ST3215_REG_ID, new_id);
}

bool ST3215Servo::set_baud_rate(uint8_t servo_id, uint8_t baud_rate)
{
	// Baud rate values: 0=9600, 1=19200, 2=38400, 3=57600, 4=115200, 5=200000, 6=250000, 7=400000, 8=500000, 9=1000000
	if (baud_rate > 9) {
		return false;
	}
	return write_byte(servo_id, ST3215_REG_BAUD_RATE, baud_rate);
}

bool ST3215Servo::factory_reset(uint8_t servo_id)
{
	uint8_t packet[6];
	packet[0] = ST3215_HEADER;
	packet[1] = ST3215_HEADER2;
	packet[2] = servo_id;
	packet[3] = 2; // Length
	packet[4] = ST3215_CMD_RESET;
	packet[5] = calculate_checksum(packet + 2, 3);

	if (!send_packet(packet, sizeof(packet))) {
		return false;
	}

	px4_usleep(5000); // 5ms delay for reset

	int bytes_received = receive_packet(_rx_buffer, BUFFER_SIZE);

	if (bytes_received >= 6 &&
	    _rx_buffer[0] == ST3215_HEADER &&
	    _rx_buffer[1] == ST3215_HEADER2 &&
	    _rx_buffer[2] == servo_id) {
		_error_status = _rx_buffer[4];
		return (_error_status == ST3215_ERROR_NONE);
	}

	return false;
}
