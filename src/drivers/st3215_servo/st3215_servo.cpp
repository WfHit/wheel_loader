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
 * @file st3215_servo.cpp
 * @author PX4 Development Team
 *
 * Main entry point for ST3215 smart servo driver
 *
 * This file contains only the module entry point.
 * The actual implementation is split across multiple files:
 * - st3215_servo_core.cpp: Core driver functionality
 * - st3215_servo_protocol.cpp: Communication protocol
 * - st3215_servo_enhanced.cpp: Enhanced SCServo functions
 * - st3215_servo_commands.cpp: Command line interface
 * - st3215_servo_test.cpp: Test utilities
 */

#include "st3215_servo.hpp"
#include <cmath>

ST3215Servo::ST3215Servo(const char *serial_port) :
	ModuleBase<ST3215Servo>(),
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::lp_default),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": loop")),
	_comms_error_perf(perf_alloc(PC_COUNT, MODULE_NAME": comm_err"))
{
	// Set the serial port
	strncpy(_serial_port, serial_port, SERIAL_PORT_MAX_LEN - 1);
	_serial_port[SERIAL_PORT_MAX_LEN - 1] = '\0';
}

ST3215Servo::~ST3215Servo()
{
	close_serial_port();
	perf_free(_loop_perf);
	perf_free(_comms_error_perf);
}

bool ST3215Servo::init()
{
	if (!open_serial_port()) {
		PX4_ERR("Failed to open serial port");
		return false;
	}

	// Start work queue
	ScheduleOnInterval(SCHEDULE_INTERVAL);

	return true;
}

void ST3215Servo::Run()
{
	if (should_exit()) {
		PX4_DEBUG("ST3215 servo driver stopping...");
		ScheduleClear();
		close_serial_port();
		return;
	}

	perf_begin(_loop_perf);

	// Update parameters
	updateParams();

	// Process commands
	process_commands();

	// Read servo status
	if (read_servo_status()) {
		publish_feedback();
	}

	perf_end(_loop_perf);
}

void ST3215Servo::process_commands()
{
	robotic_servo_command_s cmd;

	if (_servo_command_sub.update(&cmd)) {
		// Check if command is for our servo
		if (cmd.id == _param_servo_id.get()) {
			float target_position = math::constrain(cmd.goal_position,
								_param_min_position.get(),
								_param_max_position.get());

			float speed = math::constrain(cmd.goal_velocity, 0.0f, _param_max_speed.get());

			if (send_position_command(target_position, speed)) {
				_target_position = target_position;
				_last_command_time = hrt_absolute_time();
			}
		}
	}
}

void ST3215Servo::publish_feedback()
{
	robotic_servo_feedback_s feedback{};

	feedback.timestamp = hrt_absolute_time();
	feedback.id = _param_servo_id.get();
	feedback.position = _current_position;
	feedback.velocity = _current_speed;
	feedback.load = _current_load;
	feedback.temperature = _current_temperature;
	feedback.torque_enabled = _servo_enabled;
	feedback.goal_position = _target_position;

	_servo_feedback_pub.publish(feedback);
}

bool ST3215Servo::send_position_command(float position, float speed)
{
	uint8_t servo_id = _param_servo_id.get();

	// Convert position from radians to servo units (0-4095)
	float position_deg = position * 180.0f / M_PI_F; // Convert to degrees
	if (position_deg < 0) position_deg += 360.0f; // Ensure positive
	uint16_t servo_position = (uint16_t)(position_deg * 4095.0f / 360.0f);

	// Convert speed from rad/s to servo units
	float speed_deg_s = speed * 180.0f / M_PI_F; // Convert to deg/s
	uint16_t servo_speed = (uint16_t)(speed_deg_s / 0.114f); // 0.114 deg/s per unit

	// Send position command
	if (!write_word(servo_id, ST3215_REG_GOAL_POSITION_L, servo_position)) {
		return false;
	}

	// Send speed command
	return write_word(servo_id, ST3215_REG_MOVING_SPEED_L, servo_speed);
}

bool ST3215Servo::read_servo_status()
{
	uint8_t servo_id = _param_servo_id.get();

	// Ping servo first to check connectivity
	if (!ping_servo(servo_id)) {
		_connection_ok = false;
		return false;
	}

	_connection_ok = true;
	_last_ping_time = hrt_absolute_time();

	// Read full status
	return read_full_status(servo_id);
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

	PX4_DEBUG("Sending ping to servo ID %d", servo_id);
	PX4_DEBUG("Ping packet: %02X %02X %02X %02X %02X %02X",
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

uint8_t ST3215Servo::calculate_checksum(const uint8_t *data, uint8_t length)
{
	uint8_t checksum = 0;

	for (uint8_t i = 0; i < length; i++) {
		checksum += data[i];
	}

	return ~checksum;
}

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
		PX4_WARN("Serial port not open");
		return false;
	}

	ssize_t bytes_written = ::write(_serial_fd, data, length);

	if (bytes_written != length) {
		PX4_WARN("Write failed: expected %d bytes, wrote %d bytes (%s)",
			length, (int)bytes_written, strerror(errno));
		perf_count(_comms_error_perf);
		return false;
	}

	// Force transmission
	tcdrain(_serial_fd);

	PX4_DEBUG("Sent %d bytes successfully", length);
	return true;
}

int ST3215Servo::receive_packet(uint8_t *buffer, uint8_t max_length)
{
	if (_serial_fd < 0) {
		PX4_WARN("Serial port not open");
		return -1;
	}

	// Clear input buffer first
	memset(buffer, 0, max_length);

	ssize_t bytes_read = ::read(_serial_fd, buffer, max_length);

	if (bytes_read < 0) {
		if (errno != EAGAIN && errno != EWOULDBLOCK) {
			PX4_WARN("Read failed: %s", strerror(errno));
			perf_count(_comms_error_perf);
		}
		return -1;
	}

	PX4_DEBUG("Received %d bytes", (int)bytes_read);
	return (int)bytes_read;
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

// Enhanced SCServo protocol functions

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
		servo_speed |= (1 << 15); // Set direction bit
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
	// Set wheel mode by setting CW and CCW angle limits to 0
	// This puts the servo in continuous rotation mode
	if (!write_word(servo_id, ST3215_REG_CW_ANGLE_LIMIT_L, 0)) {
		return false;
	}

	return write_word(servo_id, ST3215_REG_CCW_ANGLE_LIMIT_L, 0);
}

bool ST3215Servo::write_speed(uint8_t servo_id, float speed, uint8_t acceleration)
{
	// First set acceleration (if register exists)
	if (!write_byte(servo_id, ST3215_REG_ACC, acceleration)) {
		// If acceleration register doesn't exist, continue anyway
		PX4_DEBUG("Could not set acceleration, continuing...");
	}

	// Convert speed from rad/s to servo units
	float speed_deg_s = fabsf(speed) * 180.0f / M_PI_F; // Convert to deg/s
	uint16_t servo_speed = (uint16_t)(speed_deg_s / 0.114f); // 0.114 deg/s per unit

	// Support negative speeds (SCServo protocol uses bit 15 for direction)
	if (speed < 0) {
		servo_speed |= (1 << 15); // Set direction bit for CCW
	}

	// Send speed command to goal speed register
	return write_word(servo_id, ST3215_REG_MOVING_SPEED_L, servo_speed);
}

bool ST3215Servo::set_torque_limit(uint8_t servo_id, uint16_t torque_limit)
{
	// Torque limit: 0-1000 where 1000 = 100%
	if (torque_limit > 1000) {
		torque_limit = 1000;
	}

	return write_word(servo_id, ST3215_REG_TORQUE_LIMIT_L, torque_limit);
}

bool ST3215Servo::feedback(uint8_t servo_id)
{
	// Read all feedback data at once (like SCServo FeedBack function)
	// Read from present position to present current (15 bytes total)
	uint8_t packet[8];
	packet[0] = ST3215_HEADER;
	packet[1] = ST3215_HEADER2;
	packet[2] = servo_id;
	packet[3] = 4; // Length
	packet[4] = ST3215_CMD_READ;
	packet[5] = ST3215_REG_PRESENT_POSITION_L; // Start address
	packet[6] = 15; // Read 15 bytes (position, speed, load, voltage, temp, moving, etc.)
	packet[7] = calculate_checksum(packet + 2, 5);

	if (!send_packet(packet, sizeof(packet))) {
		return false;
	}

	px4_usleep(2000); // 2ms delay

	int bytes_received = receive_packet(_rx_buffer, BUFFER_SIZE);

	if (bytes_received >= 21 && // Header(2) + ID(1) + Length(1) + Error(1) + Data(15) + Checksum(1)
	    _rx_buffer[0] == ST3215_HEADER &&
	    _rx_buffer[1] == ST3215_HEADER2 &&
	    _rx_buffer[2] == servo_id) {
		_error_status = _rx_buffer[4];
		if (_error_status == ST3215_ERROR_NONE) {
			// Parse feedback data
			uint16_t position = (_rx_buffer[6] << 8) | _rx_buffer[5];
			uint16_t speed = (_rx_buffer[8] << 8) | _rx_buffer[7];
			uint16_t load = (_rx_buffer[10] << 8) | _rx_buffer[9];

			// Convert position (handle negative positions using bit 15)
			if (position & (1 << 15)) {
				position = -(position & ~(1 << 15));
			}
			float position_deg = (float)position * 360.0f / 4095.0f;
			if (position_deg > 180.0f) position_deg -= 360.0f;
			_current_position = position_deg * M_PI_F / 180.0f;

			// Convert speed (handle direction using bit 15)
			if (speed & (1 << 15)) {
				speed = -(speed & ~(1 << 15));
			}
			float speed_deg_s = (float)speed * 0.114f; // 0.114 deg/s per unit
			_current_speed = speed_deg_s * M_PI_F / 180.0f;

			// Convert load (handle direction using bit 10)
			if (load & (1 << 10)) {
				load = -(load & ~(1 << 10));
			}
			_current_load = (float)load / 10.24f; // Convert to percentage

			// Extract other data
			_current_voltage = _rx_buffer[11]; // Voltage in 0.1V units
			_current_temperature = _rx_buffer[12]; // Temperature in Celsius
			_is_moving = (_rx_buffer[15] != 0); // Moving status

			return true;
		}
	}

	return false;
}

int ST3215Servo::read_current(uint8_t servo_id)
{
	// ST3215 doesn't have dedicated current registers like newer servos
	// This function would need hardware that supports current measurement
	// For now, return -1 to indicate not supported
	PX4_WARN("Current reading not supported on ST3215");
	return -1;
}

bool ST3215Servo::unlock_eprom(uint8_t servo_id)
{
	// Unlock EPROM by writing 0 to lock register
	return write_byte(servo_id, ST3215_REG_LOCK, 0);
}

bool ST3215Servo::lock_eprom(uint8_t servo_id)
{
	// Lock EPROM by writing 1 to lock register
	return write_byte(servo_id, ST3215_REG_LOCK, 1);
}

bool ST3215Servo::calibrate_center(uint8_t servo_id)
{
	// Calibrate center position by writing special value to torque enable register
	// This follows the SCServo CalibrationOfs function
	return write_byte(servo_id, ST3215_REG_TORQUE_ENABLE, 128);
}

bool ST3215Servo::uart_test(const char *port1, const char *port2, uint32_t baud_rate)
{
	PX4_INFO("=== UART Test Mode ===");
	PX4_INFO("Testing communication between %s and %s at %u baud", port1, port2, baud_rate);

	// Open and configure both ports
	int fd1, fd2;
	if (!uart_test_open_ports(port1, port2, baud_rate, &fd1, &fd2)) {
		return false;
	}

	// Test communication forward and reverse
	int success_count = uart_test_communication(fd1, fd2);

	// Close ports
	uart_test_close_ports(fd1, fd2);

	// Summary
	uart_test_print_results(success_count);

	return (success_count >= 4); // At least 4 out of 5 tests should pass
}

bool ST3215Servo::uart_test_open_ports(const char *port1, const char *port2, uint32_t baud_rate, int *fd1, int *fd2)
{
	// Open first serial port
	*fd1 = ::open(port1, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (*fd1 < 0) {
		PX4_ERR("Failed to open %s: %s", port1, strerror(errno));
		return false;
	}

	// Open second serial port
	*fd2 = ::open(port2, O_RDWR | O_NOCTTY | O_NONBLOCK);
	if (*fd2 < 0) {
		PX4_ERR("Failed to open %s: %s", port2, strerror(errno));
		::close(*fd1);
		return false;
	}

	// Configure both ports with same settings
	if (!uart_test_configure_port(*fd1, baud_rate) || !uart_test_configure_port(*fd2, baud_rate)) {
		::close(*fd1);
		::close(*fd2);
		return false;
	}

	// Flush any existing data
	tcflush(*fd1, TCIOFLUSH);
	tcflush(*fd2, TCIOFLUSH);

	PX4_INFO("Both ports configured successfully (%u baud, 8N1)", baud_rate);
	return true;
}

void ST3215Servo::uart_test_close_ports(int fd1, int fd2)
{
	::close(fd1);
	::close(fd2);
}

bool ST3215Servo::uart_test_configure_port(int fd, uint32_t baud_rate)
{
	struct termios tty;

	if (tcgetattr(fd, &tty) != 0) {
		PX4_ERR("Failed to get serial attributes: %s", strerror(errno));
		return false;
	}

	// Convert baud rate to termios speed_t
	speed_t speed;
	switch (baud_rate) {
	case 9600:
		speed = B9600;
		break;
	case 19200:
		speed = B19200;
		break;
	case 38400:
		speed = B38400;
		break;
	case 57600:
		speed = B57600;
		break;
	case 115200:
		speed = B115200;
		break;
	case 230400:
		speed = B230400;
		break;
	case 460800:
		speed = B460800;
		break;
	case 500000:
		speed = B500000;
		break;
	case 576000:
		speed = B576000;
		break;
	case 921600:
		speed = B921600;
		break;
	case 1000000:
		speed = B1000000;
		break;
	case 1152000:
		speed = B1152000;
		break;
	case 1500000:
		speed = B1500000;
		break;
	case 2000000:
		speed = B2000000;
		break;
	case 2500000:
		speed = B2500000;
		break;
	case 3000000:
		speed = B3000000;
		break;
	case 3500000:
		speed = B3500000;
		break;
	case 4000000:
		speed = B4000000;
		break;
	default:
		PX4_ERR("Unsupported baud rate: %u", baud_rate);
		return false;
	}

	// Set baud rate
	if (cfsetospeed(&tty, speed) != 0 || cfsetispeed(&tty, speed) != 0) {
		PX4_ERR("Failed to set baud rate to %u", baud_rate);
		return false;
	}

	// Configure 8N1
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

	if (tcsetattr(fd, TCSANOW, &tty) != 0) {
		PX4_ERR("Failed to set serial attributes: %s", strerror(errno));
		return false;
	}

	return true;
}

int ST3215Servo::uart_test_communication(int fd1, int fd2)
{
	const char *test_messages[] = {
		"Hello from port 1!",
		"Test message 123",
		"UART communication test",
		"Final test message"
	};
	const int num_messages = sizeof(test_messages) / sizeof(test_messages[0]);

	int success_count = 0;

	// Test forward direction (port 1 to port 2)
	for (int i = 0; i < num_messages; i++) {
		if (uart_test_send_receive(fd1, fd2, test_messages[i], i + 1, num_messages)) {
			success_count++;
		}
		px4_usleep(100000); // 100ms between tests
	}

	// Test reverse direction (port 2 to port 1)
	PX4_INFO("Testing reverse direction...");
	const char *reverse_msg = "Reverse test message";
	if (uart_test_send_receive(fd2, fd1, reverse_msg, 0, 0)) {
		success_count++;
	}

	return success_count;
}

bool ST3215Servo::uart_test_send_receive(int tx_fd, int rx_fd, const char *message, int test_num, int total_tests)
{
	if (test_num > 0) {
		PX4_INFO("Test %d/%d: Sending '%s'", test_num, total_tests, message);
	} else {
		PX4_INFO("Sending '%s'", message);
	}

	// Send message
	size_t msg_len = strlen(message);
	ssize_t bytes_written = ::write(tx_fd, message, msg_len);

	if (bytes_written != (ssize_t)msg_len) {
		PX4_WARN("  Write failed: expected %zu bytes, wrote %zd bytes", msg_len, bytes_written);
		return false;
	}

	// Force transmission
	tcdrain(tx_fd);

	// Wait for data to arrive
	px4_usleep(10000); // 10ms

	// Read response
	uint8_t rx_buffer[256];
	memset(rx_buffer, 0, sizeof(rx_buffer));

	ssize_t bytes_read = ::read(rx_fd, rx_buffer, sizeof(rx_buffer) - 1);

	if (bytes_read > 0) {
		rx_buffer[bytes_read] = '\0'; // Null terminate
		PX4_INFO("  Received %zd bytes: '%s'", bytes_read, (char*)rx_buffer);

		// Check if received message matches sent message
		if (bytes_read == (ssize_t)msg_len && memcmp(message, rx_buffer, msg_len) == 0) {
			PX4_INFO("  ✓ Message match - SUCCESS");
			return true;
		} else {
			PX4_WARN("  ✗ Message mismatch - FAILED");
			PX4_WARN("    Expected: '%s' (%zu bytes)", message, msg_len);
			PX4_WARN("    Received: '%s' (%zd bytes)", (char*)rx_buffer, bytes_read);
			return false;
		}
	} else {
		PX4_WARN("  ✗ No data received - FAILED");
		return false;
	}
}

void ST3215Servo::uart_test_print_results(int success_count)
{
	const int total_tests = 5; // 4 forward + 1 reverse

	PX4_INFO("=== UART Test Results ===");
	PX4_INFO("Successful tests: %d/%d", success_count, total_tests);
	PX4_INFO("Success rate: %.1f%%", (double)success_count / total_tests * 100.0);

	if (success_count >= 4) {
		PX4_INFO("✓ Tests PASSED - UART communication working correctly");
	} else {
		PX4_WARN("✗ Tests FAILED - Check connections and port configuration");
		PX4_INFO("Troubleshooting suggestions:");
		PX4_INFO("1. Verify physical connections (TX1->RX2, RX1->TX2, GND-GND)");
		PX4_INFO("2. Check if both ports exist and are accessible");
		PX4_INFO("3. Ensure no other processes are using the ports");
		PX4_INFO("4. Try different baud rates or port settings");
		PX4_INFO("5. Check signal levels and cable integrity");
	}
}

int ST3215Servo::print_status()
{
	PX4_INFO("ST3215 Servo Driver Status:");
	PX4_INFO("Serial port: %s", _serial_port);
	PX4_INFO("Servo ID: %ld", _param_servo_id.get());
	PX4_INFO("Connection: %s", _connection_ok ? "OK" : "FAILED");

	if (_connection_ok) {
		PX4_INFO("Current position: %.3f rad (%.1f deg)",
			(double)_current_position,
			(double)(_current_position * 180.0f / M_PI_F));
		PX4_INFO("Target position: %.3f rad (%.1f deg)",
			(double)_target_position,
			(double)(_target_position * 180.0f / M_PI_F));
		PX4_INFO("Current speed: %.3f rad/s (%.1f deg/s)",
			(double)_current_speed,
			(double)(_current_speed * 180.0f / M_PI_F));
		PX4_INFO("Load: %.1f%%", (double)_current_load);
		PX4_INFO("Temperature: %d°C", _current_temperature);
		PX4_INFO("Voltage: %.1fV", (double)_current_voltage / 10.0);
		PX4_INFO("Torque enabled: %s", _servo_enabled ? "Yes" : "No");
		PX4_INFO("Moving: %s", _is_moving ? "Yes" : "No");

		if (_error_status != ST3215_ERROR_NONE) {
			PX4_WARN("Error status: 0x%02X", _error_status);
			if (_error_status & ST3215_ERROR_INPUT_VOLTAGE) PX4_WARN("  - Input voltage error");
			if (_error_status & ST3215_ERROR_ANGLE_LIMIT) PX4_WARN("  - Angle limit error");
			if (_error_status & ST3215_ERROR_OVERHEATING) PX4_WARN("  - Overheating error");
			if (_error_status & ST3215_ERROR_RANGE) PX4_WARN("  - Range error");
			if (_error_status & ST3215_ERROR_CHECKSUM) PX4_WARN("  - Checksum error");
			if (_error_status & ST3215_ERROR_OVERLOAD) PX4_WARN("  - Overload error");
			if (_error_status & ST3215_ERROR_INSTRUCTION) PX4_WARN("  - Instruction error");
		}

		uint64_t last_ping_age = hrt_absolute_time() - _last_ping_time;
		PX4_INFO("Last ping: %llu us ago", last_ping_age);
	}

	perf_print_counter(_loop_perf);
	perf_print_counter(_comms_error_perf);

	return 0;
}

int ST3215Servo::task_spawn(int argc, char *argv[])
{
	const char *serial_port = "/dev/ttyS1"; // Default serial port

	// Parse command line arguments
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			serial_port = myoptarg;
			break;
		case '?':
			PX4_WARN("Unknown option");
			return PX4_ERROR;
		}
	}

	ST3215Servo *instance = new ST3215Servo(serial_port);

	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;

	return PX4_ERROR;
}

ST3215Servo *ST3215Servo::instantiate(int argc, char *argv[])
{
	const char *serial_port = "/dev/ttyS1"; // Default serial port

	// Parse command line arguments
	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			serial_port = myoptarg;
			break;
		}
	}

	return new ST3215Servo(serial_port);
}

int ST3215Servo::custom_command(int argc, char *argv[])
{
	if (argc < 1) {
		return print_usage("missing command");
	}

	const char *command = argv[0];

	// Check if driver is running for commands that need it
	if (!is_running() && strcmp(command, "start") != 0) {
		PX4_ERR("Driver not running. Start it first with 'st3215_servo start'");
		return PX4_ERROR;
	}

	ST3215Servo *instance = static_cast<ST3215Servo*>(_object.load());
	if (!instance && strcmp(command, "start") != 0) {
		PX4_ERR("No driver instance found");
		return PX4_ERROR;
	}

	if (!strcmp(command, "ping")) {
		uint8_t servo_id = 1; // Default servo ID

		if (argc >= 2) {
			servo_id = atoi(argv[1]);
		}

		if (instance->ping_servo(servo_id)) {
			PX4_INFO("Servo ID %d: PING OK", servo_id);
			return PX4_OK;
		} else {
			PX4_ERR("Servo ID %d: PING FAILED", servo_id);
			return PX4_ERROR;
		}
	}

	if (!strcmp(command, "enable")) {
		uint8_t servo_id = instance->_param_servo_id.get();

		if (argc >= 2) {
			servo_id = atoi(argv[1]);
		}

		if (instance->set_torque_enable(servo_id, true)) {
			PX4_INFO("Servo ID %d: Torque ENABLED", servo_id);
			return PX4_OK;
		} else {
			PX4_ERR("Failed to enable servo torque");
			return PX4_ERROR;
		}
	}

	if (!strcmp(command, "disable")) {
		uint8_t servo_id = instance->_param_servo_id.get();

		if (argc >= 2) {
			servo_id = atoi(argv[1]);
		}

		if (instance->set_torque_enable(servo_id, false)) {
			PX4_INFO("Servo ID %d: Torque DISABLED", servo_id);
			return PX4_OK;
		} else {
			PX4_ERR("Failed to disable servo torque");
			return PX4_ERROR;
		}
	}

	if (!strcmp(command, "led")) {
		if (argc < 2) {
			return print_usage("led command requires state argument (0=off, 1=on, 2=blink)");
		}

		uint8_t servo_id = instance->_param_servo_id.get();
		uint8_t led_state = atoi(argv[1]);

		if (argc >= 3) {
			servo_id = atoi(argv[2]);
		}

		if (instance->set_led(servo_id, led_state)) {
			PX4_INFO("Servo ID %d: LED set to %d", servo_id, led_state);
			return PX4_OK;
		} else {
			PX4_ERR("Failed to set LED state");
			return PX4_ERROR;
		}
	}

	if (!strcmp(command, "read")) {
		if (argc < 2) {
			return print_usage("read command requires register address");
		}

		uint8_t servo_id = instance->_param_servo_id.get();
		uint8_t reg_addr = strtol(argv[1], nullptr, 0); // Support hex with 0x prefix
		bool read_word = false;

		if (argc >= 3) {
			if (!strcmp(argv[2], "word")) {
				read_word = true;
			} else {
				servo_id = atoi(argv[2]);
			}
		}

		if (argc >= 4) {
			if (!strcmp(argv[3], "word")) {
				read_word = true;
			}
		}

		if (read_word) {
			uint16_t value;
			if (instance->read_word(servo_id, reg_addr, &value)) {
				PX4_INFO("Servo ID %d, Reg 0x%02X: 0x%04X (%d)", servo_id, reg_addr, value, value);
				return PX4_OK;
			}
		} else {
			uint8_t value;
			if (instance->read_byte(servo_id, reg_addr, &value)) {
				PX4_INFO("Servo ID %d, Reg 0x%02X: 0x%02X (%d)", servo_id, reg_addr, value, value);
				return PX4_OK;
			}
		}

		PX4_ERR("Failed to read register 0x%02X", reg_addr);
		return PX4_ERROR;
	}

	if (!strcmp(command, "write")) {
		if (argc < 3) {
			return print_usage("write command requires register address and value");
		}

		uint8_t servo_id = instance->_param_servo_id.get();
		uint8_t reg_addr = strtol(argv[1], nullptr, 0); // Support hex with 0x prefix
		uint16_t value = strtol(argv[2], nullptr, 0);
		bool write_word = false;

		if (argc >= 4) {
			if (!strcmp(argv[3], "word")) {
				write_word = true;
			} else {
				servo_id = atoi(argv[3]);
			}
		}

		if (argc >= 5) {
			if (!strcmp(argv[4], "word")) {
				write_word = true;
			}
		}

		bool success;
		if (write_word) {
			success = instance->write_word(servo_id, reg_addr, value);
		} else {
			success = instance->write_byte(servo_id, reg_addr, (uint8_t)value);
		}

		if (success) {
			PX4_INFO("Servo ID %d, Reg 0x%02X: Written %s 0x%04X",
				servo_id, reg_addr, write_word ? "word" : "byte", value);
			return PX4_OK;
		} else {
			PX4_ERR("Failed to write register 0x%02X", reg_addr);
			return PX4_ERROR;
		}
	}

	if (!strcmp(command, "reset")) {
		uint8_t servo_id = instance->_param_servo_id.get();

		if (argc >= 2) {
			servo_id = atoi(argv[1]);
		}

		if (instance->factory_reset(servo_id)) {
			PX4_INFO("Servo ID %d: Factory reset successful", servo_id);
			return PX4_OK;
		} else {
			PX4_ERR("Factory reset failed");
			return PX4_ERROR;
		}
	}

	if (!strcmp(command, "diagnose")) {
		uint8_t servo_id = instance->_param_servo_id.get();

		if (argc >= 2) {
			servo_id = atoi(argv[1]);
		}

		PX4_INFO("=== ST3215 Servo Diagnostic ===");
		PX4_INFO("Serial port: %s", instance->_serial_port);
		PX4_INFO("Serial FD: %d", instance->_serial_fd);
		PX4_INFO("Target servo ID: %d", servo_id);

		// Check if serial port is open
		if (instance->_serial_fd < 0) {
			PX4_ERR("Serial port is not open!");
			return PX4_ERROR;
		}

		// Try multiple ping attempts
		int success_count = 0;
		const int ping_attempts = 5;

		for (int i = 0; i < ping_attempts; i++) {
			PX4_INFO("Ping attempt %d/%d...", i + 1, ping_attempts);
			if (instance->ping_servo(servo_id)) {
				success_count++;
				PX4_INFO("  SUCCESS");
			} else {
				PX4_INFO("  FAILED");
			}
			px4_usleep(100000); // 100ms between attempts
		}

		PX4_INFO("Ping success rate: %d/%d (%.1f%%)",
			success_count, ping_attempts,
			(double)success_count / ping_attempts * 100.0);

		// Try different servo IDs (scan for servos)
		PX4_INFO("=== Servo ID Scan ===");
		bool found_any = false;
		for (uint8_t id = 1; id <= 10; id++) {
			if (instance->ping_servo(id)) {
				PX4_INFO("Found servo at ID %d", id);
				found_any = true;
			}
			px4_usleep(50000); // 50ms between scans
		}

		if (!found_any) {
			PX4_WARN("No servos found in ID range 1-10");
			PX4_INFO("Troubleshooting suggestions:");
			PX4_INFO("1. Check physical connections (power, TX/RX)");
			PX4_INFO("2. Verify servo is powered on");
			PX4_INFO("3. Check baud rate (default: 1000000)");
			PX4_INFO("4. Try different servo ID");
			PX4_INFO("5. Check if servo uses different protocol");
		}

		return PX4_OK;
	}

	if (!strcmp(command, "stop")) {
		PX4_INFO("Stopping ST3215 servo driver...");
		if (instance) {
			// Signal the driver to stop gracefully
			instance->request_stop();
		}
		return PX4_OK;
	}

	if (!strcmp(command, "writepos")) {
		if (argc < 3) {
			return print_usage("writepos command requires position and speed arguments");
		}

		uint8_t servo_id = instance->_param_servo_id.get();
		float position = strtof(argv[1], nullptr); // Position in radians
		float speed = strtof(argv[2], nullptr);    // Speed in rad/s
		uint8_t acceleration = 0;

		if (argc >= 4) {
			if (!strcmp(argv[3], "acc")) {
				if (argc >= 5) {
					acceleration = atoi(argv[4]);
				}
			} else {
				servo_id = atoi(argv[3]);
				if (argc >= 5 && !strcmp(argv[4], "acc") && argc >= 6) {
					acceleration = atoi(argv[5]);
				}
			}
		}

		if (instance->write_position_ex(servo_id, position, speed, acceleration)) {
			PX4_INFO("Servo ID %d: Position command sent (pos=%.3f rad, speed=%.3f rad/s, acc=%d)",
				servo_id, (double)position, (double)speed, acceleration);
			return PX4_OK;
		} else {
			PX4_ERR("Failed to send position command");
			return PX4_ERROR;
		}
	}

	if (!strcmp(command, "wheel")) {
		uint8_t servo_id = instance->_param_servo_id.get();

		if (argc >= 2) {
			servo_id = atoi(argv[1]);
		}

		if (instance->set_wheel_mode(servo_id)) {
			PX4_INFO("Servo ID %d: Wheel mode enabled", servo_id);
			return PX4_OK;
		} else {
			PX4_ERR("Failed to enable wheel mode");
			return PX4_ERROR;
		}
	}

	if (!strcmp(command, "speed")) {
		if (argc < 2) {
			return print_usage("speed command requires speed argument (rad/s)");
		}

		uint8_t servo_id = instance->_param_servo_id.get();
		float speed = strtof(argv[1], nullptr); // Speed in rad/s
		uint8_t acceleration = 0;

		if (argc >= 3) {
			if (!strcmp(argv[2], "acc")) {
				if (argc >= 4) {
					acceleration = atoi(argv[3]);
				}
			} else {
				servo_id = atoi(argv[2]);
				if (argc >= 4 && !strcmp(argv[3], "acc") && argc >= 5) {
					acceleration = atoi(argv[4]);
				}
			}
		}

		if (instance->write_speed(servo_id, speed, acceleration)) {
			PX4_INFO("Servo ID %d: Speed command sent (speed=%.3f rad/s, acc=%d)",
				servo_id, (double)speed, acceleration);
			return PX4_OK;
		} else {
			PX4_ERR("Failed to send speed command");
			return PX4_ERROR;
		}
	}

	if (!strcmp(command, "torque_limit")) {
		if (argc < 2) {
			return print_usage("torque_limit command requires torque value (0-1000)");
		}

		uint8_t servo_id = instance->_param_servo_id.get();
		uint16_t torque_limit = atoi(argv[1]);

		if (argc >= 3) {
			servo_id = atoi(argv[2]);
		}

		if (instance->set_torque_limit(servo_id, torque_limit)) {
			PX4_INFO("Servo ID %d: Torque limit set to %d", servo_id, torque_limit);
			return PX4_OK;
		} else {
			PX4_ERR("Failed to set torque limit");
			return PX4_ERROR;
		}
	}

	if (!strcmp(command, "feedback")) {
		uint8_t servo_id = instance->_param_servo_id.get();

		if (argc >= 2) {
			servo_id = atoi(argv[1]);
		}

		if (instance->feedback(servo_id)) {
			PX4_INFO("Servo ID %d: Feedback read successfully", servo_id);
			PX4_INFO("  Position: %.3f rad (%.1f deg)",
				(double)instance->_current_position,
				(double)(instance->_current_position * 180.0f / M_PI_F));
			PX4_INFO("  Speed: %.3f rad/s (%.1f deg/s)",
				(double)instance->_current_speed,
				(double)(instance->_current_speed * 180.0f / M_PI_F));
			PX4_INFO("  Load: %.1f%%", (double)instance->_current_load);
			PX4_INFO("  Voltage: %.1fV", (double)instance->_current_voltage / 10.0);
			PX4_INFO("  Temperature: %d°C", instance->_current_temperature);
			PX4_INFO("  Moving: %s", instance->_is_moving ? "Yes" : "No");
			return PX4_OK;
		} else {
			PX4_ERR("Failed to read feedback");
			return PX4_ERROR;
		}
	}

	if (!strcmp(command, "unlock")) {
		uint8_t servo_id = instance->_param_servo_id.get();

		if (argc >= 2) {
			servo_id = atoi(argv[1]);
		}

		if (instance->unlock_eprom(servo_id)) {
			PX4_INFO("Servo ID %d: EPROM unlocked", servo_id);
			return PX4_OK;
		} else {
			PX4_ERR("Failed to unlock EPROM");
			return PX4_ERROR;
		}
	}

	if (!strcmp(command, "lock")) {
		uint8_t servo_id = instance->_param_servo_id.get();

		if (argc >= 2) {
			servo_id = atoi(argv[1]);
		}

		if (instance->lock_eprom(servo_id)) {
			PX4_INFO("Servo ID %d: EPROM locked", servo_id);
			return PX4_OK;
		} else {
			PX4_ERR("Failed to lock EPROM");
			return PX4_ERROR;
		}
	}

	if (!strcmp(command, "calibrate")) {
		uint8_t servo_id = instance->_param_servo_id.get();

		if (argc >= 2) {
			servo_id = atoi(argv[1]);
		}

		if (instance->calibrate_center(servo_id)) {
			PX4_INFO("Servo ID %d: Center position calibrated", servo_id);
			return PX4_OK;
		} else {
			PX4_ERR("Failed to calibrate center position");
			return PX4_ERROR;
		}
	}

	if (!strcmp(command, "uart_test")) {
		if (argc < 3) {
			return print_usage("uart_test command requires two serial port paths");
		}

		const char *port1 = argv[1];
		const char *port2 = argv[2];
		uint32_t baud_rate = 115200; // Default baud rate

		// Check for optional baud rate parameter
		if (argc >= 4) {
			baud_rate = strtoul(argv[3], nullptr, 10);
			if (baud_rate == 0) {
				PX4_ERR("Invalid baud rate: %s", argv[3]);
				return PX4_ERROR;
			}
		}

		// Create a temporary instance for the test (don't need the driver running)
		ST3215Servo test_instance("/dev/null");

		if (test_instance.uart_test(port1, port2, baud_rate)) {
			PX4_INFO("UART test completed successfully");
			return PX4_OK;
		} else {
			PX4_ERR("UART test failed");
			return PX4_ERROR;
		}
	}

	return print_usage("unknown command");
}

int ST3215Servo::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver for ST3215 smart servo with full protocol support.

The driver communicates with the servo via UART and provides:
- Position and speed control
- Comprehensive feedback (position, speed, load, temperature, voltage)
- Ping functionality to test connectivity
- Direct register read/write operations
- Torque enable/disable control
- LED control
- Factory reset capability

### Examples
Start the driver with default serial port (/dev/ttyS1):
$ st3215_servo start

Start the driver with custom serial port:
$ st3215_servo start -d /dev/ttyS1

Check status:
$ st3215_servo status

Ping servo (test connectivity):
$ st3215_servo ping [servo_id]

Enable/disable servo torque:
$ st3215_servo enable [servo_id]
$ st3215_servo disable [servo_id]

Control LED (0=off, 1=on, 2=blink):
$ st3215_servo led 1 [servo_id]

Read register (byte or word):
$ st3215_servo read 0x18 [servo_id] [word]
$ st3215_servo read 0x24 word

Write register (byte or word):
$ st3215_servo write 0x18 1 [servo_id] [word]
$ st3215_servo write 0x1E 2048 word

Factory reset servo:
$ st3215_servo reset [servo_id]

Enhanced position control with acceleration:
$ st3215_servo writepos 1.57 2.0 [servo_id] [acc 50]

Enable wheel mode (continuous rotation):
$ st3215_servo wheel [servo_id]

Control speed in wheel mode:
$ st3215_servo speed 3.14 [servo_id] [acc 50]

Set torque limit (0-1000):
$ st3215_servo torque_limit 500 [servo_id]

Read comprehensive feedback:
$ st3215_servo feedback [servo_id]

Unlock/lock EPROM:
$ st3215_servo unlock [servo_id]
$ st3215_servo lock [servo_id]

Calibrate center position:
$ st3215_servo calibrate [servo_id]

Test UART communication between two ports:
$ st3215_servo uart_test /dev/ttyS1 /dev/ttyS2 [baud_rate]

Examples:
$ st3215_servo uart_test /dev/ttyS1 /dev/ttyS2          # Default 115200 baud
$ st3215_servo uart_test /dev/ttyS1 /dev/ttyS2 9600     # 9600 baud
$ st3215_servo uart_test /dev/ttyS1 /dev/ttyS2 1000000  # 1M baud

Stop the driver:
$ st3215_servo stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("st3215_servo", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the driver");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS1", "<device>", "Serial device", true);
	PRINT_MODULE_USAGE_COMMAND_DESCR("ping", "Ping servo to test connectivity");
	PRINT_MODULE_USAGE_COMMAND_DESCR("enable", "Enable servo torque");
	PRINT_MODULE_USAGE_COMMAND_DESCR("disable", "Disable servo torque");
	PRINT_MODULE_USAGE_COMMAND_DESCR("led", "Control servo LED (0=off, 1=on, 2=blink)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("read", "Read servo register (add 'word' for 16-bit read)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("write", "Write servo register (add 'word' for 16-bit write)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("reset", "Factory reset servo");
	PRINT_MODULE_USAGE_COMMAND_DESCR("writepos", "Enhanced position control with acceleration");
	PRINT_MODULE_USAGE_COMMAND_DESCR("wheel", "Enable wheel mode (continuous rotation)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("speed", "Control speed in wheel mode");
	PRINT_MODULE_USAGE_COMMAND_DESCR("torque_limit", "Set torque limit (0-1000)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("feedback", "Read comprehensive servo feedback");
	PRINT_MODULE_USAGE_COMMAND_DESCR("unlock", "Unlock EPROM for writing");
	PRINT_MODULE_USAGE_COMMAND_DESCR("lock", "Lock EPROM to prevent writes");
	PRINT_MODULE_USAGE_COMMAND_DESCR("calibrate", "Calibrate servo center position");
	PRINT_MODULE_USAGE_COMMAND_DESCR("uart_test", "Test UART communication between two ports (optional baud rate)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("diagnose", "Run comprehensive connection diagnostic");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}
