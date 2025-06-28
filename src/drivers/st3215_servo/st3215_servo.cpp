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
		ScheduleClear();
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
	// Convert position from radians to servo units (0-4095)
	// ST3215 range is typically 0-4095 for 360 degrees
	float position_deg = position * 180.0f / M_PI_F; // Convert to degrees
	uint16_t servo_position = (uint16_t)((position_deg + 180.0f) * 4095.0f / 360.0f);

	// Convert speed from rad/s to servo units
	float speed_deg_s = speed * 180.0f / M_PI_F; // Convert to deg/s
	uint16_t servo_speed = (uint16_t)(speed_deg_s * 4095.0f / 360.0f);

	// Build command packet
	uint8_t packet[11];
	packet[0] = ST3215_HEADER;
	packet[1] = ST3215_HEADER2;
	packet[2] = _param_servo_id.get();
	packet[3] = 7; // Length
	packet[4] = ST3215_CMD_WRITE;
	packet[5] = ST3215_REG_GOAL_POSITION;
	packet[6] = servo_position & 0xFF;
	packet[7] = (servo_position >> 8) & 0xFF;
	packet[8] = servo_speed & 0xFF;
	packet[9] = (servo_speed >> 8) & 0xFF;
	packet[10] = calculate_checksum(packet + 2, 8);

	return send_packet(packet, sizeof(packet));
}

bool ST3215Servo::read_servo_status()
{
	// Read position
	uint8_t read_packet[8];
	read_packet[0] = ST3215_HEADER;
	read_packet[1] = ST3215_HEADER2;
	read_packet[2] = _param_servo_id.get();
	read_packet[3] = 4; // Length
	read_packet[4] = ST3215_CMD_READ;
	read_packet[5] = ST3215_REG_PRESENT_POSITION;
	read_packet[6] = 2; // Read 2 bytes
	read_packet[7] = calculate_checksum(read_packet + 2, 5);

	if (!send_packet(read_packet, sizeof(read_packet))) {
		return false;
	}

	// Receive response
	int bytes_received = receive_packet(_rx_buffer, BUFFER_SIZE);

	if (bytes_received >= 8) {
		uint16_t position = (_rx_buffer[6] << 8) | _rx_buffer[5];
		// Convert from servo units to radians
		float position_deg = (float)position * 360.0f / 4095.0f - 180.0f;
		_current_position = position_deg * M_PI_F / 180.0f; // Convert to radians
		return true;
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
	_serial_fd = ::open(_serial_port, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_serial_fd < 0) {
		PX4_ERR("Failed to open %s", _serial_port);
		return false;
	}

	// Configure serial port
	struct termios tty;

	if (tcgetattr(_serial_fd, &tty) != 0) {
		PX4_ERR("Failed to get serial attributes");
		close_serial_port();
		return false;
	}

	// Set baud rate to 1000000
	cfsetospeed(&tty, B1000000);
	cfsetispeed(&tty, B1000000);

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
		PX4_ERR("Failed to set serial attributes");
		close_serial_port();
		return false;
	}

	PX4_INFO("Opened %s successfully", _serial_port);
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
		return false;
	}

	ssize_t bytes_written = ::write(_serial_fd, data, length);

	if (bytes_written != length) {
		perf_count(_comms_error_perf);
		return false;
	}

	return true;
}

int ST3215Servo::receive_packet(uint8_t *buffer, uint8_t max_length)
{
	if (_serial_fd < 0) {
		return -1;
	}

	ssize_t bytes_read = ::read(_serial_fd, buffer, max_length);

	if (bytes_read < 0) {
		perf_count(_comms_error_perf);
		return -1;
	}

	return (int)bytes_read;
}

int ST3215Servo::print_status()
{
	PX4_INFO("ST3215 Servo Driver Status:");
	PX4_INFO("Serial port: %s", _serial_port);
	PX4_INFO("Servo ID: %ld", _param_servo_id.get());
	PX4_INFO("Current position: %.3f rad (%.1f deg)", (double)_current_position, (double)(_current_position * 180.0f / M_PI_F));
	PX4_INFO("Target position: %.3f rad (%.1f deg)", (double)_target_position, (double)(_target_position * 180.0f / M_PI_F));
	PX4_INFO("Current speed: %.3f rad/s (%.1f deg/s)", (double)_current_speed, (double)(_current_speed * 180.0f / M_PI_F));
	PX4_INFO("Load: %.1f%%", (double)_current_load);
	PX4_INFO("Temperature: %d°C", _current_temperature);
	PX4_INFO("Enabled: %s", _servo_enabled ? "Yes" : "No");

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
Driver for ST3215 smart servo.

The driver communicates with the servo via UART and provides:
- Position control
- Speed control
- Feedback (position, speed, load, temperature)

### Examples
Start the driver with default serial port (/dev/ttyS1):
$ st3215_servo start

Start the driver with custom serial port:
$ st3215_servo start -d /dev/ttyS1

Check status:
$ st3215_servo status

Stop the driver:
$ st3215_servo stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("st3215_servo", "driver");
	PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the driver");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS1", "<device>", "Serial device", true);
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}
