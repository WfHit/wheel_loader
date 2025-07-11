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
	// Close existing connection if open
	if (_uart >= 0) {
		::close(_uart);
		_uart = -1;
	}

	// Open serial port
	PX4_DEBUG("Opening serial port %s...", _port_name);
	_uart = ::open(_port_name, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (_uart < 0) {
		PX4_ERR("Failed to open %s: %s", _port_name, strerror(errno));
		return false;
	}

	PX4_DEBUG("Serial port opened successfully (fd=%d)", _uart);

	// Configure port settings (similar to test_serial)
	struct termios uart_config;

	if (tcgetattr(_uart, &uart_config) != 0) {
		PX4_ERR("Error getting serial port attributes: %s", strerror(errno));
		::close(_uart);
		_uart = -1;
		return false;
	}

	// Get baudrate parameter (default to 115200)
	int32_t baudrate = _param_baudrate.get();
	if (baudrate <= 0) {
		baudrate = 115200;  // Default changed to 115200
	}

	PX4_DEBUG("Configuring baudrate: %ld", (long)baudrate);

	speed_t speed;
	switch (baudrate) {
	case 9600:    speed = B9600; break;
	case 19200:   speed = B19200; break;
	case 38400:   speed = B38400; break;
	case 57600:   speed = B57600; break;
	case 115200:  speed = B115200; break;
	case 230400:  speed = B230400; break;
	case 460800:  speed = B460800; break;
	case 921600:  speed = B921600; break;
	case 1000000: speed = B1000000; break;
	default:
		PX4_WARN("Unsupported baudrate: %ld, using 115200", (long)baudrate);
		speed = B115200;
		break;
	}

	cfsetospeed(&uart_config, speed);
	cfsetispeed(&uart_config, speed);

	// Configure port settings (8N1, no flow control)
	uart_config.c_cflag &= ~PARENB;  // No parity
	uart_config.c_cflag &= ~CSTOPB;  // One stop bit
	uart_config.c_cflag &= ~CSIZE;   // Clear size bits
	uart_config.c_cflag |= CS8;      // 8 data bits
	uart_config.c_cflag &= ~CRTSCTS; // No hardware flow control
	uart_config.c_cflag |= CREAD | CLOCAL; // Enable reading and ignore modem control lines

	uart_config.c_lflag &= ~ICANON;  // Non-canonical mode
	uart_config.c_lflag &= ~ECHO;    // No echo
	uart_config.c_lflag &= ~ECHOE;   // No echo erase
	uart_config.c_lflag &= ~ECHONL;  // No echo newline
	uart_config.c_lflag &= ~ISIG;    // No signal processing

	uart_config.c_iflag &= ~(IXON | IXOFF | IXANY); // No software flow control
	uart_config.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR | ICRNL);

	uart_config.c_oflag &= ~OPOST;   // No output processing
	uart_config.c_oflag &= ~ONLCR;   // No CR to NL translation

	// Set timeouts for blocking reads
	uart_config.c_cc[VTIME] = 1;     // Wait for up to 0.1s (1 decisecond)
	uart_config.c_cc[VMIN] = 0;      // No minimum number of characters

	if (tcsetattr(_uart, TCSANOW, &uart_config) != 0) {
		PX4_ERR("Error setting serial port attributes: %s", strerror(errno));
		::close(_uart);
		_uart = -1;
		return false;
	}

	// Clear any existing data
	tcflush(_uart, TCIOFLUSH);

	PX4_DEBUG("Serial port %s configured at %ld baud", _port_name, (long)baudrate);
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

	// Process incoming servo commands
	process_commands();

	// Read servo status periodically (every 50ms) - but skip if in test mode
	static hrt_abstime last_status_read = 0;
	if (hrt_elapsed_time(&last_status_read) > 50_ms && _uart >= 0 && !is_test_mode_active()) {
		last_status_read = hrt_absolute_time();

		uint8_t servo_id = _param_servo_id.get();

		if (servo_id > 0 && read_status(servo_id)) {
			_connection_ok = true;
			_last_update_time = hrt_absolute_time();
			_consecutive_errors = 0;
			publish_feedback();
		} else {
			_consecutive_errors++;
			perf_count(_comms_error_perf);
			PX4_WARN("Status read failed, consecutive errors: %d", _consecutive_errors);

			// Reset connection after 5 consecutive errors (moderate recovery)
			if (_consecutive_errors >= 5) {
				PX4_WARN("Multiple communication errors, resetting connection");
				if (_uart >= 0) {
					::close(_uart);
					_uart = -1;
				}
				_consecutive_errors = 0;
				_connection_ok = false;
			}

			// Mark as disconnected after 1 second of no communication
			if (hrt_elapsed_time(&_last_update_time) > 1_s) {
				_connection_ok = false;
			}
		}
	}

	perf_end(_loop_perf);
}

bool ST3215Servo::send_packet(const uint8_t *data, uint8_t length)
{
	if (_uart < 0 || !data || length == 0) {
		PX4_ERR("send_packet: Invalid parameters - uart=%d, data=%p, length=%d", _uart, data, length);
		return false;
	}

	// Log packet being sent (debug only)
	PX4_DEBUG("TX[%d bytes]", length);

	// Clear input buffer before sending to avoid stale data
	tcflush(_uart, TCIFLUSH);

	// Send the packet
	ssize_t bytes_written = ::write(_uart, data, length);
	if (bytes_written != length) {
		PX4_ERR("Write failed: expected %d, wrote %d, error: %s", length, (int)bytes_written, strerror(errno));
		return false;
	}

	// Wait for transmission to complete
	tcdrain(_uart);
	return true;
}

bool ST3215Servo::receive_packet(uint8_t *buffer, size_t buffer_size, uint32_t timeout_ms)
{
	if (_uart < 0 || !buffer || buffer_size == 0) {
		PX4_ERR("receive_packet: Invalid parameters - uart=%d, buffer=%p, size=%zu", _uart, buffer, buffer_size);
		return false;
	}

	hrt_abstime start_time = hrt_absolute_time();
	size_t bytes_received = 0;

	// First, read at least the header (4 bytes: 0xFF 0xFF ID LENGTH)
	const size_t header_size = 4;

	while (bytes_received < header_size && bytes_received < buffer_size) {
		// Check for timeout
		if (hrt_elapsed_time(&start_time) > timeout_ms * 1000) {
			PX4_WARN("Header timeout after %lu ms, got %zu bytes", timeout_ms, bytes_received);
			return false;
		}

		ssize_t bytes_read = ::read(_uart, buffer + bytes_received, buffer_size - bytes_received);
		if (bytes_read > 0) {
			bytes_received += bytes_read;
		} else if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
			PX4_ERR("Read error: %s", strerror(errno));
			return false;
		}

		// Small delay to prevent CPU spinning
		usleep(500);
	}

	// Check if we have valid header
	if (bytes_received < header_size) {
		PX4_WARN("Insufficient header bytes: got %zu, need %zu", bytes_received, header_size);
		return false;
	}

	// Validate header
	if (buffer[0] != ST3215_HEADER || buffer[1] != ST3215_HEADER2) {
		PX4_ERR("Invalid header: 0x%02X 0x%02X (expected 0xFF 0xFF)", buffer[0], buffer[1]);
		return false;
	}

	// Parse packet length from header
	uint8_t packet_length = buffer[3];

	// Calculate total expected packet size
	// Format: Header(2) + ID(1) + Length(1) + Data(Length) = 4 + Length
	// Where Data includes: Status(1) + Parameters(Length-2) + Checksum(1)
	size_t total_expected = 4 + packet_length;

	if (total_expected > buffer_size) {
		PX4_ERR("Packet too large: expected %zu bytes, buffer only %zu", total_expected, buffer_size);
		return false;
	}

	// Read remaining bytes
	while (bytes_received < total_expected && bytes_received < buffer_size) {
		if (hrt_elapsed_time(&start_time) > timeout_ms * 1000) {
			PX4_WARN("Data timeout after %lu ms, got %zu/%zu bytes", timeout_ms, bytes_received, total_expected);
			break;
		}

		ssize_t bytes_read = ::read(_uart, buffer + bytes_received, buffer_size - bytes_received);
		if (bytes_read > 0) {
			bytes_received += bytes_read;
		} else if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
			PX4_ERR("Read error: %s", strerror(errno));
			return false;
		}
		usleep(500);
	}

	// Log received packet (debug only)
	PX4_DEBUG("RX[%zu bytes]", bytes_received);

	// Check if we got complete packet
	if (bytes_received < total_expected) {
		PX4_WARN("Incomplete packet: expected %zu, got %zu", total_expected, bytes_received);
		return false;
	}

	// Verify checksum
	// Checksum = ~(ID + Length + Status + Parameters...)
	// Calculate from buffer[2] (ID) for (packet_length + 1) bytes (ID + Length + Status + Parameters)
	uint8_t calculated_checksum = calculate_checksum(&buffer[2], packet_length + 1);
	uint8_t received_checksum = buffer[total_expected - 1];

	if (calculated_checksum == received_checksum) {
		return true;
	} else {
		PX4_ERR("Checksum mismatch: calc=0x%02X, recv=0x%02X", calculated_checksum, received_checksum);
		return false;
	}
}

bool ST3215Servo::ping_servo(uint8_t servo_id)
{
	PX4_DEBUG("Pinging servo ID %d...", servo_id);

	uint8_t packet[6] = {
		ST3215_HEADER,      // 0xFF
		ST3215_HEADER2,     // 0xFF
		servo_id,           // ID
		0x02,               // Length
		ST3215_CMD_PING,    // Instruction
		0x00                // Checksum (will be calculated)
	};
	packet[5] = calculate_checksum(&packet[2], 3);

	if (!send_packet(packet, 6)) {
		PX4_ERR("Failed to send ping packet");
		return false;
	}

	// Wait for response (simple blocking read with timeout)
	uint8_t response[16];
	if (receive_packet(response, sizeof(response), PACKET_TIMEOUT_MS)) {
		// Check if response is from correct servo and command
		if (response[2] == servo_id) {
			PX4_DEBUG("Ping response received from servo %d", servo_id);
			return true;
		} else {
			PX4_ERR("Ping response from wrong servo ID: expected %d, got %d", servo_id, response[2]);
		}
	} else {
		PX4_ERR("No ping response received");
	}

	return false;
}

bool ST3215Servo::read_register(uint8_t servo_id, uint8_t reg_addr, uint8_t *data, uint8_t length)
{
	PX4_DEBUG("Reading %d bytes from servo %d, register 0x%02X", length, servo_id, reg_addr);

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

	if (!send_packet(packet, 8)) {
		PX4_ERR("Failed to send read command");
		return false;
	}

	// Wait for response
	uint8_t response[32];
	if (receive_packet(response, sizeof(response), PACKET_TIMEOUT_MS)) {
		// Check if response is from correct servo
		if (response[2] == servo_id && response[3] >= (length + 2)) {

			// Copy data from response packet (skip header, ID, length, error)
			memcpy(data, &response[5], length);

			PX4_DEBUG("Read data successful");

			return true;
		} else {
			PX4_ERR("Invalid read response: servo_id=%d (expected %d), length=%d (expected >=%d)",
				response[2], servo_id, response[3], length + 2);
		}
	} else {
		PX4_ERR("No read response received");
	}

	return false;
}

bool ST3215Servo::write_register(uint8_t servo_id, uint8_t reg_addr, const uint8_t *data, uint8_t length)
{
	if (length > 20) {  // Safety check
		PX4_ERR("Write length too large: %d bytes (max 20)", length);
		return false;
	}

	PX4_DEBUG("Writing %d bytes to servo %d, register 0x%02X", length, servo_id, reg_addr);

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

	PX4_DEBUG("Write command prepared");

	bool result = send_packet(packet, 7 + length);
	if (!result) {
		PX4_ERR("Failed to send write command");
	}

	return result;
}

uint8_t ST3215Servo::calculate_checksum(const uint8_t *data, uint8_t length)
{
	uint8_t sum = 0;
	for (uint8_t i = 0; i < length; i++) {
		sum += data[i];
	}
	return ~sum;
}

bool ST3215Servo::write_position(uint8_t servo_id, float position_rad, float speed_rad_s)
{
	// Convert radians to servo position units
	float position_deg = position_rad * 180.0f / M_PI_F;

	// Convert to servo units (0-4095 maps to 0-360 degrees)
	// Handle negative positions like SMS_STS WritePosEx
	int16_t position_value = (int16_t)(position_deg * 4095.0f / 360.0f);
	uint16_t position_raw;

	if (position_value < 0) {
		position_raw = (-position_value) | (1 << 15); // Set sign bit for negative
	} else {
		position_raw = (uint16_t)position_value;
	}

	// Convert speed from rad/s to servo units
	// Based on SMS_STS implementation, speed range is 0-1023
	float speed_deg_s = fabsf(speed_rad_s) * 180.0f / M_PI_F;
	speed_deg_s = math::constrain(speed_deg_s, 0.0f, 360.0f); // Max ~6.28 rad/s
	uint16_t speed_raw = (uint16_t)(speed_deg_s * 1023.0f / 360.0f);

	// Write position and speed using the SMS_STS style (ACC + Position + Time + Speed)
	uint8_t data[7] = {
		0,                                      // ACC (acceleration, 0 = default)
		(uint8_t)(position_raw & 0xFF),        // Position L
		(uint8_t)((position_raw >> 8) & 0xFF), // Position H
		0, 0,                                   // Time L, H (0 = move immediately)
		(uint8_t)(speed_raw & 0xFF),           // Speed L
		(uint8_t)((speed_raw >> 8) & 0xFF)     // Speed H
	};

	// Retry once on failure
	if (!write_register(servo_id, ST3215_REG_ACC, data, 7)) {
		// Single retry
		usleep(1000); // 1ms delay
		return write_register(servo_id, ST3215_REG_ACC, data, 7);
	}
	return true;
}

bool ST3215Servo::read_status(uint8_t servo_id)
{
	// Read each register individually like SMS_STS.cpp does
	uint8_t pos_data[2], speed_data[2], load_data[2];
	uint8_t voltage_data[1], temp_data[1];

	// Read position (2 bytes) - like ReadPos in SMS_STS.cpp
	if (!read_register(servo_id, ST3215_REG_PRESENT_POSITION_L, pos_data, 2)) {
		PX4_ERR("Failed to read position register");
		return false;
	}

	// Read speed (2 bytes) - like ReadSpeed in SMS_STS.cpp
	if (!read_register(servo_id, ST3215_REG_PRESENT_SPEED_L, speed_data, 2)) {
		PX4_ERR("Failed to read speed register");
		return false;
	}

	// Read load (2 bytes) - like ReadLoad in SMS_STS.cpp
	if (!read_register(servo_id, ST3215_REG_PRESENT_LOAD_L, load_data, 2)) {
		PX4_ERR("Failed to read load register");
		return false;
	}

	// Read voltage (1 byte) - like ReadVoltage in SMS_STS.cpp
	if (!read_register(servo_id, ST3215_REG_PRESENT_VOLTAGE, voltage_data, 1)) {
		PX4_ERR("Failed to read voltage register");
		return false;
	}

	// Read temperature (1 byte) - like ReadTemper in SMS_STS.cpp
	if (!read_register(servo_id, ST3215_REG_PRESENT_TEMP, temp_data, 1)) {
		PX4_ERR("Failed to read temperature register");
		return false;
	}

	// Parse the response data according to SMS_STS register layout
	uint16_t position_raw = pos_data[0] | (pos_data[1] << 8);
	uint16_t speed_raw = speed_data[0] | (speed_data[1] << 8);
	uint16_t load_raw = load_data[0] | (load_data[1] << 8);
	_current_voltage = voltage_data[0];
	_current_temperature = temp_data[0];

	// Convert position to radians (0-4095 maps to 0-360 degrees)
	// SMS_STS ReadPos handles sign bit, but for position it's typically not used in normal operation
	uint16_t pos_value = position_raw;
	if (position_raw & (1 << 15)) {  // Handle negative position like SMS_STS
		pos_value = -(position_raw & ~(1 << 15));
	}
	float position_deg = (pos_value * 360.0f / 4095.0f);
	_current_position = position_deg * M_PI_F / 180.0f;

	// Convert speed to rad/s - handle sign bit according to SMS_STS
	float speed_deg_s = (speed_raw & ~(1 << 15)) * 360.0f / 1023.0f;  // Remove sign bit before conversion
	if (speed_raw & (1 << 15)) {  // Check sign bit
		speed_deg_s = -speed_deg_s;
	}
	_current_speed = speed_deg_s * M_PI_F / 180.0f;

	// Convert load - handle sign bit according to SMS_STS
	float load_value = (load_raw & ~(1 << 10)) / 10.0f; // Remove sign bit, 0-1000 -> 0-100%
	if (load_raw & (1 << 10)) {  // Check sign bit for load (bit 10, not 15)
		load_value = -load_value;
	}
	_current_load = load_value;

	return true;
}

bool ST3215Servo::set_torque_enable(uint8_t servo_id, bool enable)
{
	uint8_t value = enable ? 1 : 0;

	// Retry once on failure
	if (!write_register(servo_id, ST3215_REG_TORQUE_ENABLE, &value, 1)) {
		// Single retry
		usleep(1000); // 1ms delay
		return write_register(servo_id, ST3215_REG_TORQUE_ENABLE, &value, 1);
	}
	return true;
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

		// Enter test mode to suspend status reading
		_object.load()->enter_test_mode();

		uint8_t servo_id = _object.load()->_param_servo_id.get();
		bool result = _object.load()->ping_servo(servo_id);

		// Exit test mode
		_object.load()->exit_test_mode();

		if (result) {
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

		// Enter test mode to suspend status reading during diagnostics
		_object.load()->enter_test_mode();
		_object.load()->run_diagnostics();
		_object.load()->exit_test_mode();
		return 0;
	}

	if (!strcmp(argv[0], "raw_test")) {
		if (!_object.load()) {
			PX4_ERR("driver not running");
			return -1;
		}

		// Enter test mode to suspend status reading during raw test
		_object.load()->enter_test_mode();
		_object.load()->test_raw_communication();
		_object.load()->exit_test_mode();
		return 0;
	}

	if (!strcmp(argv[0], "verbose")) {
		if (!_object.load()) {
			PX4_ERR("driver not running");
			return -1;
		}

		// Enable verbose logging by setting log level
		PX4_INFO("Enabling verbose logging for ST3215 driver");
		// Note: You can manually set this via: param set SYS_LOGGER_LEVEL 4
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

		// Enter test mode to suspend status reading
		_object.load()->enter_test_mode();

		bool result = _object.load()->write_position(servo_id, position, 2.0f);

		// Exit test mode
		_object.load()->exit_test_mode();

		if (result) {
			PX4_INFO("Position command sent successfully");
		} else {
			PX4_ERR("Failed to send position command");
		}
		return 0;
	}

	if (!strcmp(argv[0], "wheel_mode")) {
		if (!_object.load()) {
			PX4_ERR("driver not running");
			return -1;
		}

		// Enter test mode to suspend status reading
		_object.load()->enter_test_mode();

		uint8_t servo_id = _object.load()->_param_servo_id.get();
		bool result = _object.load()->wheel_mode(servo_id);

		// Exit test mode
		_object.load()->exit_test_mode();

		if (result) {
			PX4_INFO("Servo %d set to wheel mode", servo_id);
		} else {
			PX4_ERR("Failed to set servo %d to wheel mode", servo_id);
		}
		return 0;
	}

	if (!strcmp(argv[0], "speed")) {
		if (argc < 2) {
			return print_usage("missing speed argument");
		}

		if (!_object.load()) {
			PX4_ERR("driver not running");
			return -1;
		}

		float speed = strtof(argv[1], nullptr);
		uint8_t servo_id = _object.load()->_param_servo_id.get();

		PX4_INFO("Setting servo %d speed to %.3f rad/s", servo_id, (double)speed);

		// Enter test mode to suspend status reading
		_object.load()->enter_test_mode();

		bool result = _object.load()->write_speed(servo_id, speed);

		// Exit test mode
		_object.load()->exit_test_mode();

		if (result) {
			PX4_INFO("Speed command sent successfully");
		} else {
			PX4_ERR("Failed to send speed command");
		}
		return 0;
	}

	if (!strcmp(argv[0], "read_info")) {
		if (!_object.load()) {
			PX4_ERR("driver not running");
			return -1;
		}

		// Enter test mode to suspend status reading
		_object.load()->enter_test_mode();

		uint8_t servo_id = _object.load()->_param_servo_id.get();

		int moving = _object.load()->read_moving(servo_id);
		int mode = _object.load()->read_mode(servo_id);

		// Exit test mode
		_object.load()->exit_test_mode();

		PX4_INFO("Servo %d info: moving=%d, mode=%d", servo_id, moving, mode);
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

To run diagnostics:
$ st3215_servo diag

To set position:
$ st3215_servo position 0.5

To set wheel mode:
$ st3215_servo wheel_mode

To set speed (in wheel mode):
$ st3215_servo speed 1.5

To read servo info:
$ st3215_servo read_info

To stop the driver:
$ st3215_servo stop
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("st3215_servo", "driver");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS3", "<file:dev>", "UART device", false);

	PRINT_MODULE_USAGE_COMMAND("ping");
	PRINT_MODULE_USAGE_COMMAND("diag");
	PRINT_MODULE_USAGE_COMMAND("status");
	PRINT_MODULE_USAGE_COMMAND("position");
	PRINT_MODULE_USAGE_ARG("<angle>", "Angle in radians", false);
	PRINT_MODULE_USAGE_COMMAND("wheel_mode");
	PRINT_MODULE_USAGE_COMMAND("speed");
	PRINT_MODULE_USAGE_ARG("<speed>", "Speed in rad/s", false);
	PRINT_MODULE_USAGE_COMMAND("read_info");

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
		if (configure_port()) {
			PX4_INFO("Port opened successfully for diagnostics");
		} else {
			PX4_ERR("Failed to open port for diagnostics");
			return;
		}
	} else {
		PX4_INFO("UART is open (fd=%d)", _uart);
	}

	// 3. Check connection status
	PX4_INFO("Connection status: %s", _connection_ok ? "OK" : "NOT OK");
	PX4_INFO("Consecutive errors: %d", _consecutive_errors);
	if (_last_update_time > 0) {
		uint64_t time_since_update = hrt_elapsed_time(&_last_update_time) / 1000; // ms
		PX4_INFO("Time since last update: %llu ms", (unsigned long long)time_since_update);
	}

	// 4. Communication test
	uint8_t servo_id = _param_servo_id.get();
	PX4_INFO("Running communication tests for servo ID %d...", servo_id);

	// Test 1: Ping test
	PX4_INFO("1. Ping test...");
	if (ping_servo(servo_id)) {
		PX4_INFO("   PING: PASSED");
	} else {
		PX4_ERR("   PING: FAILED");
	}

	// Test 2: Status read test
	PX4_INFO("2. Status read test...");
	if (read_status(servo_id)) {
		PX4_INFO("   STATUS READ: PASSED");
		PX4_INFO("   Position: %.3f rad (%.1f deg)", (double)_current_position,
			(double)(_current_position * 180.0f / M_PI_F));
		PX4_INFO("   Speed: %.3f rad/s (%.1f deg/s)", (double)_current_speed,
			(double)(_current_speed * 180.0f / M_PI_F));
		PX4_INFO("   Load: %.1f%%", (double)_current_load);
		PX4_INFO("   Voltage: %.1fV", (double)(_current_voltage * 0.1f));
		PX4_INFO("   Temperature: %d°C", _current_temperature);
	} else {
		PX4_ERR("   STATUS READ: FAILED");
	}

	// Test 3: Extended servo info read test
	PX4_INFO("3. Extended servo info test...");
	int moving = read_moving(servo_id);
	int mode = read_mode(servo_id);

	if (moving >= 0 && mode >= 0) {
		PX4_INFO("   EXTENDED INFO: PASSED");
		PX4_INFO("   Moving status: %d", moving);
		PX4_INFO("   Mode: %d (0=position, 1=wheel)", mode);
	} else {
		PX4_ERR("   EXTENDED INFO: FAILED");
	}

	// Test 4: Basic communication patterns (like test_serial)
	PX4_INFO("4. Basic communication pattern test...");
	test_communication_patterns();

	// 5. Performance counters
	PX4_INFO("Performance counters:");
	PX4_INFO("  Loop count: %llu", (unsigned long long)perf_event_count(_loop_perf));
	PX4_INFO("  Comm errors: %llu", (unsigned long long)perf_event_count(_comms_error_perf));
	PX4_INFO("  Packet count: %llu", (unsigned long long)perf_event_count(_packet_count_perf));

	PX4_INFO("=== End of diagnostics ===");
}

void ST3215Servo::test_communication_patterns()
{
	if (_uart < 0) {
		PX4_ERR("UART not open, cannot test communication patterns");
		return;
	}

	uint8_t servo_id = _param_servo_id.get();
	int passed = 0;
	int total = 5;

	// Test multiple ping commands
	for (int i = 0; i < total; i++) {
		if (ping_servo(servo_id)) {
			passed++;
		}
		usleep(2000); // 2ms between tests
	}

	float success_rate = (float)passed / (float)total * 100.0f;
	PX4_INFO("   Communication pattern test: %d/%d passed (%.1f%%)",
		passed, total, (double)success_rate);

	if (success_rate >= 80.0f) {
		PX4_INFO("   COMMUNICATION PATTERNS: PASSED");
	} else {
		PX4_ERR("   COMMUNICATION PATTERNS: FAILED");
	}
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

	PX4_DEBUG("Sending PING packet");

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
			PX4_INFO("Received %d bytes", total_bytes);
		} else {
			PX4_ERR("No response received");
		}
	}

	PX4_INFO("=== End of raw test ===");
}

// Test mode control methods
void ST3215Servo::enter_test_mode()
{
	_test_mode_active = true;
	_test_mode_start = hrt_absolute_time();
	PX4_DEBUG("ST3215: Entering test mode - status reading suspended");
}

void ST3215Servo::exit_test_mode()
{
	if (_test_mode_active) {
		_test_mode_active = false;
		hrt_abstime duration = hrt_elapsed_time(&_test_mode_start) / 1000; // ms
		PX4_DEBUG("ST3215: Exiting test mode after %llu ms", (unsigned long long)duration);
	}
}

bool ST3215Servo::is_test_mode_active() const
{
	// Auto-exit test mode after 5 seconds to prevent getting stuck
	if (_test_mode_active && hrt_elapsed_time(&_test_mode_start) > 5_s) {
		// Const cast is safe here since we're just clearing a safety flag
		const_cast<ST3215Servo*>(this)->_test_mode_active = false;
		PX4_WARN("ST3215: Auto-exiting test mode after timeout");
		return false;
	}
	return _test_mode_active;
}

// SMS_STS style utility functions
bool ST3215Servo::wheel_mode(uint8_t servo_id)
{
	return write_register(servo_id, ST3215_REG_MODE, (const uint8_t[]){1}, 1);
}

bool ST3215Servo::write_speed(uint8_t servo_id, float speed_rad_s, uint8_t acc)
{
	// Convert speed to servo units with sign handling
	float speed_deg_s = speed_rad_s * 180.0f / M_PI_F;
	uint16_t speed_raw = (uint16_t)(fabsf(speed_deg_s) * 1023.0f / 360.0f);

	if (speed_deg_s < 0) {
		speed_raw |= (1 << 15); // Set sign bit for negative speed
	}

	// First write acceleration
	if (!write_register(servo_id, ST3215_REG_ACC, &acc, 1)) {
		return false;
	}

	// Then write speed
	uint8_t speed_data[2] = {
		(uint8_t)(speed_raw & 0xFF),
		(uint8_t)((speed_raw >> 8) & 0xFF)
	};

	return write_register(servo_id, ST3215_REG_GOAL_SPEED_L, speed_data, 2);
}

bool ST3215Servo::unlock_eprom(uint8_t servo_id)
{
	return write_register(servo_id, ST3215_REG_LOCK, (const uint8_t[]){0}, 1);
}

bool ST3215Servo::lock_eprom(uint8_t servo_id)
{
	return write_register(servo_id, ST3215_REG_LOCK, (const uint8_t[]){1}, 1);
}

int ST3215Servo::read_moving(uint8_t servo_id)
{
	uint8_t moving_data;
	if (read_register(servo_id, ST3215_REG_MOVING, &moving_data, 1)) {
		return moving_data;
	}
	return -1;
}

int ST3215Servo::read_mode(uint8_t servo_id)
{
	uint8_t mode_data;
	if (read_register(servo_id, ST3215_REG_MODE, &mode_data, 1)) {
		return mode_data;
	}
	return -1;
}

extern "C" __EXPORT int st3215_servo_main(int argc, char *argv[])
{
	return ST3215Servo::main(argc, argv);
}
