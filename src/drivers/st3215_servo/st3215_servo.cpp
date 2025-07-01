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
 * Simplified ST3215 smart servo driver implementation
 * Uses SerialPort API like UWB SR150 for robust UART communication
 */

#include "st3215_servo.hpp"
#include <px4_platform_common/log.h>
#include <px4_platform_common/getopt.h>
#include <lib/mathlib/mathlib.h>
#include <drivers/drv_hrt.h>
#include <cstring>
#include <errno.h>
#include <sys/ioctl.h>
#include <unistd.h>

ST3215Servo::ST3215Servo(const char *serial_port) :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(serial_port)),
	_loop_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": loop")),
	_comms_error_perf(perf_alloc(PC_COUNT, MODULE_NAME": comm_err")),
	_packet_count_perf(perf_alloc(PC_COUNT, MODULE_NAME": packets"))
{
	strncpy(_port_name, serial_port, sizeof(_port_name) - 1);
	_port_name[sizeof(_port_name) - 1] = '\0';
}

ST3215Servo::~ST3215Servo()
{
	if (_uart >= 0) {
		::close(_uart);
		_uart = -1;
	}

	perf_free(_loop_perf);
	perf_free(_comms_error_perf);
	perf_free(_packet_count_perf);
}

bool ST3215Servo::init()
{
	// Configure serial port
	if (!configure_port()) {
		PX4_WARN("Failed to configure serial port, will retry later");
		// Don't fail completely, try to recover later
	}

	// Start work queue
	ScheduleOnInterval(SCHEDULE_INTERVAL);

	PX4_INFO("ST3215 servo driver started on %s", _port_name);
	return true;
}

bool ST3215Servo::configure_port()
{
	/* open fd */
	_uart = ::open(_port_name, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_uart < 0) {
		PX4_ERR("open failed (%i)", errno);
		return false;
	}

	struct termios uart_config;

	int termios_state;

	/* fill the struct for the new configuration */
	tcgetattr(_uart, &uart_config);

	/* clear ONLCR flag (which appends a CR for every LF) */
	uart_config.c_oflag &= ~ONLCR;

	//TODO: should I keep this?
	/* no parity, one stop bit */
	uart_config.c_cflag &= ~(CSTOPB | PARENB);

	// Get baudrate parameter (default to 1000000 if not set)
	unsigned speed = B1000000;  // Default for ST3215
	if (_param_baudrate.get() > 0) {
		int32_t baudrate = _param_baudrate.get();

		switch (baudrate) {
		case 9600:   speed = B9600; break;
		case 19200:  speed = B19200; break;
		case 38400:  speed = B38400; break;
		case 57600:  speed = B57600; break;
		case 115200: speed = B115200; break;
		case 1000000: speed = B1000000; break;
		default:
			PX4_WARN("Unsupported baudrate: %ld, using 1000000", baudrate);
			speed = B1000000;
			break;
		}
	}

	/* set baud rate */
	if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
		PX4_ERR("CFG: %d ISPD", termios_state);
	}

	if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
		PX4_ERR("CFG: %d OSPD", termios_state);
	}

	if ((termios_state = tcsetattr(_uart, TCSANOW, &uart_config)) < 0) {
		PX4_ERR("baud %d ATTR", termios_state);
	}

	return true;
}

void ST3215Servo::Run()
{
	if (should_exit()) {
		ScheduleClear();
		if (_uart >= 0) {
			::close(_uart);
			_uart = -1;
		}
		return;
	}

	perf_begin(_loop_perf);

	// Ensure UART is properly configured
	if (_uart < 0) {
		if (!configure_port()) {
			// Failed to configure, will try again next cycle
			perf_end(_loop_perf);
			return;
		}
	}

	// Update parameters
	updateParams();

	// Collect any pending data with no blocking
	collect_packet(0);

	// Process incoming servo commands
	process_commands();

	// Read servo status periodically (every 50ms)
	static hrt_abstime last_status_read = 0;
	if (hrt_elapsed_time(&last_status_read) > 50_ms && _uart >= 0) {
		last_status_read = hrt_absolute_time();

		uint8_t servo_id = _param_servo_id.get();
		if (servo_id > 0 && read_status(servo_id)) {
			_connection_ok = true;
			_last_update_time = hrt_absolute_time();
			publish_feedback();
		} else {
			// Mark as disconnected after 1 second of no communication
			if (hrt_elapsed_time(&_last_update_time) > 1_s) {
				_connection_ok = false;
			}
			perf_count(_comms_error_perf);
		}
	}

	perf_end(_loop_perf);
}

bool ST3215Servo::send_packet(const uint8_t *data, uint8_t length)
{
	if (_uart < 0 || !data || length == 0) {
		return false;
	}

	// Flush input buffer before sending to avoid stale data
	tcflush(_uart, TCIFLUSH);

	// Send the packet
	ssize_t bytes_written = ::write(_uart, data, length);
	if (bytes_written != length) {
		PX4_DEBUG("Write failed: expected %d, wrote %d", length, (int)bytes_written);
		return false;
	}

	// Wait for transmission to complete
	tcdrain(_uart);
	return true;
}

int ST3215Servo::collect_packet(uint32_t timeout_ms)
{
	if (_uart < 0) {
		return -1;
	}

	hrt_abstime start_time = hrt_absolute_time();

	do {
		// Read available data into temporary buffer
		uint8_t temp_buffer[64];
		ssize_t bytes_read = ::read(_uart, temp_buffer, sizeof(temp_buffer));

		if (bytes_read > 0) {
			// Add to circular buffer
			for (int i = 0; i < bytes_read; i++) {
				uint16_t next_head = (_rx_buffer_head + 1) % sizeof(_rx_buffer);
				if (next_head != _rx_buffer_tail) {
					_rx_buffer[_rx_buffer_head] = temp_buffer[i];
					_rx_buffer_head = next_head;
				}
			}
		}

		// Try to parse complete packets
		while (parse_packet()) {
			perf_count(_packet_count_perf);

			// Check if this is the response we're waiting for
			if (_waiting_for_response && _rx_packet[2] == _expected_response_id) {
				_waiting_for_response = false;
				return _rx_packet_length;
			}
		}

		// Check for timeout
		if (timeout_ms > 0 && hrt_elapsed_time(&start_time) > timeout_ms * 1000) {
			break;
		}

		// If no data and we have timeout, use poll for efficiency
		if (bytes_read <= 0 && timeout_ms > 0) {
			pollfd fds[1];
			fds[0].fd = _uart;
			fds[0].events = POLLIN;

			uint32_t remaining_ms = timeout_ms - (hrt_elapsed_time(&start_time) / 1000);
			if (remaining_ms > 0) {
				poll(fds, 1, remaining_ms);
			}
		}

	} while (timeout_ms > 0 && hrt_elapsed_time(&start_time) < timeout_ms * 1000);

	return -1;
}

bool ST3215Servo::parse_packet()
{
	// State machine parser similar to UWB SR150
	while (_rx_buffer_head != _rx_buffer_tail) {
		uint8_t byte = _rx_buffer[_rx_buffer_tail];
		_rx_buffer_tail = (_rx_buffer_tail + 1) % sizeof(_rx_buffer);

		switch (_parse_state) {
		case ParseState::WAIT_HEADER1:
			if (byte == ST3215_HEADER) {
				_rx_packet[0] = byte;
				_parse_state = ParseState::WAIT_HEADER2;
			}
			break;

		case ParseState::WAIT_HEADER2:
			if (byte == ST3215_HEADER2) {
				_rx_packet[1] = byte;
				_parse_state = ParseState::WAIT_ID;
			} else {
				_parse_state = ParseState::WAIT_HEADER1;
			}
			break;

		case ParseState::WAIT_ID:
			_rx_packet[2] = byte;
			_parse_state = ParseState::WAIT_LENGTH;
			break;

		case ParseState::WAIT_LENGTH:
			_rx_packet[3] = byte;
			_expected_data_length = byte;
			_rx_data_count = 0;

			if (_expected_data_length >= 2) {
				_parse_state = ParseState::WAIT_ERROR;
			} else {
				// Invalid length
				_parse_state = ParseState::WAIT_HEADER1;
			}
			break;

		case ParseState::WAIT_ERROR:
			_rx_packet[4] = byte;

			if (_expected_data_length > 2) {
				_parse_state = ParseState::WAIT_DATA;
			} else {
				_parse_state = ParseState::WAIT_CHECKSUM;
			}
			break;

		case ParseState::WAIT_DATA:
			_rx_packet[5 + _rx_data_count] = byte;
			_rx_data_count++;

			if (_rx_data_count >= (_expected_data_length - 2)) {
				_parse_state = ParseState::WAIT_CHECKSUM;
			}
			break;

		case ParseState::WAIT_CHECKSUM:
			_rx_packet[4 + _expected_data_length] = byte;
			_rx_packet_length = 4 + _expected_data_length + 1;

			// Verify checksum
			uint8_t calculated_checksum = calculate_checksum(&_rx_packet[2], _expected_data_length + 1);
			_parse_state = ParseState::WAIT_HEADER1;

			if (calculated_checksum == byte) {
				return true;  // Valid packet received
			}
			break;
		}
	}

	return false;
}

uint8_t ST3215Servo::calculate_checksum(const uint8_t *data, uint8_t length)
{
	uint8_t sum = 0;
	for (uint8_t i = 0; i < length; i++) {
		sum += data[i];
	}
	return ~sum;
}

bool ST3215Servo::ping_servo(uint8_t servo_id)
{
	uint8_t packet[6] = {
		ST3215_HEADER,      // 0xFF
		ST3215_HEADER2,     // 0xFF
		servo_id,           // ID
		0x02,               // Length
		ST3215_CMD_PING,    // Instruction
		0x00                // Checksum (will be calculated)
	};
	packet[5] = calculate_checksum(&packet[2], 3);

	_waiting_for_response = true;
	_expected_response_id = servo_id;
	_expected_response_cmd = ST3215_CMD_PING;

	if (!send_packet(packet, 6)) {
		_waiting_for_response = false;
		return false;
	}

	return collect_packet(PACKET_TIMEOUT_MS) > 0;
}

bool ST3215Servo::write_position(uint8_t servo_id, float position_rad, float speed_rad_s)
{
	// Convert radians to servo position units (0-4095 for full range)
	float position_deg = position_rad * 180.0f / M_PI_F;

	// Constrain to servo limits (-150 to +150 degrees)
	position_deg = math::constrain(position_deg, -150.0f, 150.0f);

	// Convert to servo units (0-4095 maps to -150 to +150 degrees)
	uint16_t position_raw = (uint16_t)((position_deg + 150.0f) * 4095.0f / 300.0f);

	// Convert speed from rad/s to servo units
	float speed_deg_s = fabsf(speed_rad_s) * 180.0f / M_PI_F;
	speed_deg_s = math::constrain(speed_deg_s, 0.0f, 300.0f);
	uint16_t speed_raw = (uint16_t)(speed_deg_s * 1023.0f / 300.0f);

	// Write position and speed in one command
	uint8_t data[4] = {
		(uint8_t)(position_raw & 0xFF),        // Position L
		(uint8_t)((position_raw >> 8) & 0xFF), // Position H
		(uint8_t)(speed_raw & 0xFF),           // Speed L
		(uint8_t)((speed_raw >> 8) & 0xFF)     // Speed H
	};

	return write_register(servo_id, ST3215_REG_GOAL_POSITION_L, data, 4);
}

bool ST3215Servo::read_status(uint8_t servo_id)
{
	// Read 8 bytes starting from present position register
	uint8_t data[8];
	if (!read_register(servo_id, ST3215_REG_PRESENT_POSITION_L, data, 8)) {
		return false;
	}

	// Parse the response data
	uint16_t position_raw = data[0] | (data[1] << 8);
	uint16_t speed_raw = data[2] | (data[3] << 8);
	uint16_t load_raw = data[4] | (data[5] << 8);
	_current_voltage = data[6];
	_current_temperature = data[7];

	// Convert position to radians
	float position_deg = (position_raw * 300.0f / 4095.0f) - 150.0f;
	_current_position = position_deg * M_PI_F / 180.0f;

	// Convert speed to rad/s (with direction from load register)
	float speed_deg_s = (speed_raw & 0x3FF) * 300.0f / 1023.0f;
	if (speed_raw & 0x0400) {  // Direction bit
		speed_deg_s = -speed_deg_s;
	}
	_current_speed = speed_deg_s * M_PI_F / 180.0f;

	// Load as percentage
	_current_load = (load_raw & 0x3FF) / 10.24f;

	return true;
}

bool ST3215Servo::set_torque_enable(uint8_t servo_id, bool enable)
{
	uint8_t value = enable ? 1 : 0;
	return write_register(servo_id, ST3215_REG_TORQUE_ENABLE, &value, 1);
}

bool ST3215Servo::read_register(uint8_t servo_id, uint8_t reg_addr, uint8_t *data, uint8_t length)
{
	uint8_t packet[8] = {
		ST3215_HEADER,      // 0xFF
		ST3215_HEADER2,     // 0xFF
		servo_id,           // ID
		0x04,               // Length
		ST3215_CMD_READ,    // Instruction
		reg_addr,           // Starting address
		length,             // Number of bytes to read
		0x00                // Checksum (will be calculated)
	};
	packet[7] = calculate_checksum(&packet[2], 5);

	_waiting_for_response = true;
	_expected_response_id = servo_id;
	_expected_response_cmd = ST3215_CMD_READ;

	if (!send_packet(packet, 8)) {
		_waiting_for_response = false;
		return false;
	}

	if (collect_packet(PACKET_TIMEOUT_MS) < (6 + length)) {
		return false;
	}

	// Copy data from response packet (skip header, ID, length, error)
	memcpy(data, &_rx_packet[5], length);
	return true;
}

bool ST3215Servo::write_register(uint8_t servo_id, uint8_t reg_addr, const uint8_t *data, uint8_t length)
{
	if (length > 20) {  // Safety check
		return false;
	}

	uint8_t packet[32] = {
		ST3215_HEADER,              // 0xFF
		ST3215_HEADER2,             // 0xFF
		servo_id,                   // ID
		(uint8_t)(3 + length),      // Length (instruction + address + data)
		ST3215_CMD_WRITE,           // Instruction
		reg_addr                    // Starting address
	};

	// Copy data payload
	memcpy(&packet[6], data, length);

	// Calculate and add checksum
	packet[6 + length] = calculate_checksum(&packet[2], 4 + length);

	return send_packet(packet, 7 + length);
}

void ST3215Servo::process_commands()
{
	robotic_servo_command_s cmd;
	if (_servo_command_sub.update(&cmd)) {
		uint8_t servo_id = _param_servo_id.get();

		if (servo_id == 0) {
			return;  // Invalid servo ID
		}

		// Handle enable/disable
		if (cmd.torque_enable != _servo_enabled) {
			if (set_torque_enable(servo_id, cmd.torque_enable)) {
				_servo_enabled = cmd.torque_enable;
				PX4_DEBUG("Servo %d %s", servo_id, cmd.torque_enable ? "enabled" : "disabled");
			}
		}

		// Send position command if servo is enabled
		if (_servo_enabled && cmd.torque_enable) {
			float target_speed = fabsf(cmd.goal_velocity);

			// Use default speed if not specified
			if (target_speed < 0.01f) {
				target_speed = _param_max_speed.get();
				if (target_speed < 0.01f) {
					target_speed = 2.0f;  // Default 2 rad/s
				}
			}

			write_position(servo_id, cmd.goal_position, target_speed);
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
	feedback.load = _current_load / 100.0f;  // Convert percentage to -1.0 to 1.0
	feedback.temperature = _current_temperature;
	feedback.voltage = _current_voltage * 0.1f;  // Convert to volts
	feedback.torque_enabled = _servo_enabled;
	feedback.goal_position = _current_position;  // For now, use current position
	feedback.current = 0.0f;  // Not available from ST3215
	feedback.error_flags = 0;
	feedback.position_error = 0.0f;
	feedback.moving = (fabsf(_current_speed) > 0.01f);
	feedback.position_reached = true;  // Simplification for now
	feedback.hardware_alarm = 0;
	feedback.shutdown_alarm = 0;

	_servo_feedback_pub.publish(feedback);
}

int ST3215Servo::print_status()
{
	PX4_INFO("ST3215 Servo Driver Status:");
	PX4_INFO("  Port: %s", _port_name);
	PX4_INFO("  UART: %s (fd: %d)", (_uart >= 0) ? "Open" : "Closed", _uart);
	PX4_INFO("  Servo ID: %ld", _param_servo_id.get());
	PX4_INFO("  Connected: %s", _connection_ok ? "Yes" : "No");
	PX4_INFO("  Enabled: %s", _servo_enabled ? "Yes" : "No");

	if (_connection_ok) {
		PX4_INFO("  Position: %.1f deg", (double)(_current_position * 180.0f / M_PI_F));
		PX4_INFO("  Speed: %.1f deg/s", (double)(_current_speed * 180.0f / M_PI_F));
		PX4_INFO("  Load: %.1f %%", (double)_current_load);
		PX4_INFO("  Temperature: %d °C", _current_temperature);
		PX4_INFO("  Voltage: %.1f V", (double)(_current_voltage * 0.1f));
	}

	perf_print_counter(_loop_perf);
	perf_print_counter(_comms_error_perf);
	perf_print_counter(_packet_count_perf);

	return 0;
}

ST3215Servo *ST3215Servo::instantiate(int argc, char *argv[])
{
	const char *serial_port = "/dev/ttyS3";

	int myoptind = 1;
	int ch;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			serial_port = myoptarg;
			break;

		default:
			return nullptr;
		}
	}

	return new ST3215Servo(serial_port);
}

int ST3215Servo::custom_command(int argc, char *argv[])
{
	if (argc < 1) {
		return print_usage("missing command");
	}

	if (!strcmp(argv[0], "ping")) {
		if (!_object.load()) {
			PX4_ERR("driver not running");
			return -1;
		}

		uint8_t servo_id = _object.load()->_param_servo_id.get();
		if (_object.load()->ping_servo(servo_id)) {
			PX4_INFO("Servo %d responded to ping", servo_id);
		} else {
			PX4_ERR("Servo %d did not respond to ping", servo_id);
		}
		return 0;
	}

	if (!strcmp(argv[0], "diag") || !strcmp(argv[0], "diagnostics")) {
		if (!_object.load()) {
			PX4_ERR("driver not running");
			return -1;
		}

		_object.load()->run_diagnostics();
		return 0;
	}

	if (!strcmp(argv[0], "raw_test")) {
		if (!_object.load()) {
			PX4_ERR("driver not running");
			return -1;
		}

		_object.load()->test_raw_communication();
		return 0;
	}

	if (!strcmp(argv[0], "position")) {
		if (argc < 2) {
			return print_usage("missing position argument");
		}

		if (!_object.load()) {
			PX4_ERR("driver not running");
			return -1;
		}

		float position = strtof(argv[1], nullptr);
		uint8_t servo_id = _object.load()->_param_servo_id.get();

		PX4_INFO("Setting servo %d to position %.3f rad", servo_id, (double)position);

		if (_object.load()->write_position(servo_id, position, 2.0f)) {
			PX4_INFO("Position command sent successfully");
		} else {
			PX4_ERR("Failed to send position command");
		}
		return 0;
	}

	return print_usage("unknown command");
}

int ST3215Servo::task_spawn(int argc, char *argv[])
{
	const char *device_name = "/dev/ttyS3";
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;

	while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
		case 'd':
			device_name = myoptarg;
			break;

		default:
			PX4_WARN("unrecognized flag");
			return PX4_ERROR;
		}
	}

	ST3215Servo *instance = new ST3215Servo(device_name);

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

int ST3215Servo::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Driver for ST3215 smart servo over serial interface

### Implementation
This driver communicates with ST3215 servo using its serial protocol.
The driver supports position control, speed setting, and status feedback.

### Examples
To start the driver on UART4 (default):
$ st3215_servo start -d /dev/ttyS3

To test servo communication:
$ st3215_servo ping

To set position:
$ st3215_servo position 0.5

To stop the driver:
$ st3215_servo stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("st3215_servo", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS3", "<file:dev>", "UART device", false);

	PRINT_MODULE_USAGE_COMMAND("ping");
	PRINT_MODULE_USAGE_COMMAND("status");
	PRINT_MODULE_USAGE_COMMAND("position");
	PRINT_MODULE_USAGE_ARG("<angle>", "Angle in radians", false);

	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

void ST3215Servo::run_diagnostics()
{
	PX4_INFO("=== ST3215 Servo Diagnostics ===");

	// 1. Check serial port configuration
	PX4_INFO("Serial port: %s", _port_name);
	PX4_INFO("Configured baudrate: %ld", (long)_param_baudrate.get());
	PX4_INFO("Servo ID: %ld", (long)_param_servo_id.get());

	// 2. Check if UART is open
	if (_uart < 0) {
		PX4_ERR("UART is not open!");

		// Try to open the port
		int test_fd = ::open(_port_name, O_RDWR | O_NOCTTY | O_NONBLOCK);
		if (test_fd < 0) {
			PX4_ERR("Failed to open %s: %s", _port_name, strerror(errno));

			// Check if device exists
			if (access(_port_name, F_OK) != 0) {
				PX4_ERR("Device %s does not exist!", _port_name);
			} else if (access(_port_name, R_OK | W_OK) != 0) {
				PX4_ERR("No read/write permission for %s", _port_name);
			}
		} else {
			PX4_INFO("Port can be opened, closing test fd");
			::close(test_fd);
		}
	} else {
		PX4_INFO("UART is open (fd=%d)", _uart);

		// Check port status
		int status;
		if (ioctl(_uart, TIOCMGET, &status) == 0) {
			PX4_INFO("Port status: DTR=%d RTS=%d CTS=%d DSR=%d",
					 (status & TIOCM_DTR) ? 1 : 0,
					 (status & TIOCM_RTS) ? 1 : 0,
					 (status & TIOCM_CTS) ? 1 : 0,
					 (status & TIOCM_DSR) ? 1 : 0);
		}
	}

	// 3. Check connection status
	PX4_INFO("Connection status: %s", _connection_ok ? "OK" : "NOT OK");
	if (_last_update_time > 0) {
		uint64_t time_since_update = hrt_elapsed_time(&_last_update_time) / 1000; // ms
		PX4_INFO("Time since last update: %llu ms", (unsigned long long)time_since_update);
	}

	// 4. Try to ping the servo
	PX4_INFO("Attempting to ping servo ID %ld...", (long)_param_servo_id.get());

	if (_uart >= 0) {
		// Clear any pending data
		tcflush(_uart, TCIOFLUSH);

		// Send ping command
		uint8_t servo_id = _param_servo_id.get();
		if (ping_servo(servo_id)) {
			PX4_INFO("PING SUCCESSFUL! Servo is responding");

			// Try to read status
			if (read_status(servo_id)) {
				PX4_INFO("Status read successful:");
				PX4_INFO("  Position: %.3f rad", (double)_current_position);
				PX4_INFO("  Speed: %.3f rad/s", (double)_current_speed);
				PX4_INFO("  Load: %.1f%%", (double)_current_load);
				PX4_INFO("  Voltage: %d (%.1fV)", _current_voltage, _current_voltage / 10.0);
				PX4_INFO("  Temperature: %d C", _current_temperature);
			} else {
				PX4_WARN("Failed to read servo status");
			}
		} else {
			PX4_ERR("PING FAILED! No response from servo");

			// Check for any data in buffer
			uint8_t buffer[256];
			int bytes = ::read(_uart, buffer, sizeof(buffer));
			if (bytes > 0) {
				PX4_INFO("Found %d bytes in buffer:", bytes);
				for (int i = 0; i < bytes && i < 32; i++) {
					printf("%02X ", buffer[i]);
				}
				printf("\n");
			}
		}
	}

	// 5. Performance counters
	PX4_INFO("Performance counters:");
	PX4_INFO("  Loop count: %llu", (unsigned long long)perf_event_count(_loop_perf));
	PX4_INFO("  Comm errors: %llu", (unsigned long long)perf_event_count(_comms_error_perf));
	PX4_INFO("  Packet count: %llu", (unsigned long long)perf_event_count(_packet_count_perf));

	PX4_INFO("=== End of diagnostics ===");
}

void ST3215Servo::test_raw_communication()
{
	PX4_INFO("=== Raw Communication Test ===");

	if (_uart < 0) {
		PX4_ERR("UART not open, cannot test");
		return;
	}

	// Test 1: Send ping and wait for raw response
	uint8_t servo_id = _param_servo_id.get();
	uint8_t ping_packet[6] = {
		0xFF, 0xFF,     // Header
		servo_id,       // ID
		0x02,           // Length
		0x01,           // PING command
		0x00            // Checksum
	};
	ping_packet[5] = calculate_checksum(&ping_packet[2], 3);

	PX4_INFO("Sending PING packet:");
	for (int i = 0; i < 6; i++) {
		printf("%02X ", ping_packet[i]);
	}
	printf("\n");

	// Clear buffers
	tcflush(_uart, TCIOFLUSH);

	// Send packet
	ssize_t written = ::write(_uart, ping_packet, 6);
	PX4_INFO("Wrote %d bytes", (int)written);

	if (written == 6) {
		// Wait for response
		usleep(10000); // 10ms

		uint8_t response[256];
		int total_bytes = 0;

		// Read with timeout
		for (int i = 0; i < 10; i++) {
			int bytes = ::read(_uart, response + total_bytes, sizeof(response) - total_bytes);
			if (bytes > 0) {
				total_bytes += bytes;
				PX4_INFO("Read %d bytes (total: %d)", bytes, total_bytes);
			}
			usleep(1000); // 1ms
		}

		if (total_bytes > 0) {
			PX4_INFO("Received %d bytes:", total_bytes);
			for (int i = 0; i < total_bytes && i < 64; i++) {
				printf("%02X ", response[i]);
				if ((i + 1) % 16 == 0) printf("\n");
			}
			printf("\n");
		} else {
			PX4_ERR("No response received");
		}
	}

	PX4_INFO("=== End of raw test ===");
}

extern "C" __EXPORT int st3215_servo_main(int argc, char *argv[])
{
	return ST3215Servo::main(argc, argv);
}
