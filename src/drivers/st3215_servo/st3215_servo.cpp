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
 * Driver for ST3215 smart servo
 */

#include "st3215_servo.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <lib/mathlib/mathlib.h>
#include <cstring>
#include <cerrno>

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
	if (argc < 2) {
		return print_usage("missing command");
	}

	const char *command = argv[1];

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

		if (argc >= 3) {
			servo_id = atoi(argv[2]);
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

		if (argc >= 3) {
			servo_id = atoi(argv[2]);
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

		if (argc >= 3) {
			servo_id = atoi(argv[2]);
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
		if (argc < 3) {
			return print_usage("led command requires state argument (0=off, 1=on, 2=blink)");
		}

		uint8_t servo_id = instance->_param_servo_id.get();
		uint8_t led_state = atoi(argv[2]);

		if (argc >= 4) {
			servo_id = atoi(argv[3]);
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
		if (argc < 3) {
			return print_usage("read command requires register address");
		}

		uint8_t servo_id = instance->_param_servo_id.get();
		uint8_t reg_addr = strtol(argv[2], nullptr, 0); // Support hex with 0x prefix
		bool read_word = false;

		if (argc >= 4) {
			if (!strcmp(argv[3], "word")) {
				read_word = true;
			} else {
				servo_id = atoi(argv[3]);
			}
		}

		if (argc >= 5) {
			if (!strcmp(argv[4], "word")) {
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
		if (argc < 4) {
			return print_usage("write command requires register address and value");
		}

		uint8_t servo_id = instance->_param_servo_id.get();
		uint8_t reg_addr = strtol(argv[2], nullptr, 0); // Support hex with 0x prefix
		uint16_t value = strtol(argv[3], nullptr, 0);
		bool write_word = false;

		if (argc >= 5) {
			if (!strcmp(argv[4], "word")) {
				write_word = true;
			} else {
				servo_id = atoi(argv[4]);
			}
		}

		if (argc >= 6) {
			if (!strcmp(argv[5], "word")) {
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

		if (argc >= 3) {
			servo_id = atoi(argv[2]);
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

		if (argc >= 3) {
			servo_id = atoi(argv[2]);
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
	PRINT_MODULE_USAGE_COMMAND_DESCR("diagnose", "Run comprehensive connection diagnostic");
	PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop the driver");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}
