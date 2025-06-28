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
 * @file st3215_servo_commands.cpp
 * @author PX4 Development Team
 *
 * Command line interface for ST3215 smart servo driver
 */

#include "st3215_servo.hpp"

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

int ST3215Servo::custom_command(int argc, char *argv[])
{
	if (argc < 1) {
		return print_usage("missing command");
	}

	const char *command = argv[0];

	// Check if driver is running for commands that need it
	if (!is_running() && strcmp(command, "start") != 0 && strcmp(command, "uart_test") != 0) {
		PX4_ERR("Driver not running. Start it first with 'st3215_servo start'");
		return PX4_ERROR;
	}

	ST3215Servo *instance = static_cast<ST3215Servo*>(_object.load());
	if (!instance && strcmp(command, "start") != 0 && strcmp(command, "uart_test") != 0) {
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

	if (!strcmp(command, "wheel_mode")) {
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
			return print_usage("speed command requires speed value (rad/s)");
		}

		uint8_t servo_id = instance->_param_servo_id.get();
		float speed = atof(argv[1]);
		uint8_t acceleration = 50; // Default acceleration

		if (argc >= 3) {
			acceleration = atoi(argv[2]);
		}

		if (argc >= 4) {
			servo_id = atoi(argv[3]);
		}

		if (instance->write_speed(servo_id, speed, acceleration)) {
			PX4_INFO("Servo ID %d: Speed set to %.3f rad/s (acc: %d)", servo_id, (double)speed, acceleration);
			return PX4_OK;
		} else {
			PX4_ERR("Failed to set speed");
			return PX4_ERROR;
		}
	}

	if (!strcmp(command, "position_ex")) {
		if (argc < 2) {
			return print_usage("position_ex command requires position value (radians)");
		}

		uint8_t servo_id = instance->_param_servo_id.get();
		float position = atof(argv[1]);
		float speed = 1.0f; // Default speed
		uint8_t acceleration = 50; // Default acceleration

		if (argc >= 3) {
			speed = atof(argv[2]);
		}

		if (argc >= 4) {
			acceleration = atoi(argv[3]);
		}

		if (argc >= 5) {
			servo_id = atoi(argv[4]);
		}

		if (instance->write_position_ex(servo_id, position, speed, acceleration)) {
			PX4_INFO("Servo ID %d: Position set to %.3f rad (speed: %.3f, acc: %d)",
				servo_id, (double)position, (double)speed, acceleration);
			return PX4_OK;
		} else {
			PX4_ERR("Failed to set position");
			return PX4_ERROR;
		}
	}

	if (!strcmp(command, "feedback")) {
		uint8_t servo_id = instance->_param_servo_id.get();

		if (argc >= 2) {
			servo_id = atoi(argv[1]);
		}

		if (instance->feedback(servo_id)) {
			PX4_INFO("Servo ID %d feedback updated successfully", servo_id);
			return PX4_OK;
		} else {
			PX4_ERR("Failed to read feedback");
			return PX4_ERROR;
		}
	}

	if (!strcmp(command, "uart_test")) {
		const char *port1 = "/dev/ttyS1";
		const char *port2 = "/dev/ttyS2";
		int baud_rate = 115200;

		if (argc >= 2) {
			port1 = argv[1];
		}

		if (argc >= 3) {
			port2 = argv[2];
		}

		if (argc >= 4) {
			baud_rate = atoi(argv[3]);
		}

		return ST3215Servo::uart_test(port1, port2, baud_rate);
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

	if (!strcmp(command, "monitor_ping")) {
		if (argc < 2) {
			return print_usage("monitor_ping command requires monitor port path");
		}

		const char *monitor_port = argv[1];
		uint8_t servo_id = 1; // Default servo ID

		if (argc >= 3) {
			servo_id = atoi(argv[2]);
		}

		PX4_INFO("Monitoring ping transmission on port %s for servo ID %d", monitor_port, servo_id);

		if (instance->monitor_ping_transmission(monitor_port, servo_id)) {
			PX4_INFO("Ping transmission monitoring: SUCCESS");
			return PX4_OK;
		} else {
			PX4_ERR("Ping transmission monitoring: FAILED");
			return PX4_ERROR;
		}
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
Driver for ST3215 smart servo with full SCServo protocol support.

The driver communicates with the servo via UART and provides:
- Position and speed control with acceleration
- Comprehensive feedback (position, speed, load, temperature, voltage)
- Wheel mode for continuous rotation
- Enhanced SCServo protocol functions
- Ping functionality to test connectivity
- Direct register read/write operations
- Torque enable/disable control
- LED control
- Factory reset capability
- UART testing utilities

### Examples
Start the driver with default serial port (/dev/ttyS1):
$ st3215_servo start

Start the driver with custom serial port:
$ st3215_servo start -d /dev/ttyS2

Check status:
$ st3215_servo status

Ping servo (test connectivity):
$ st3215_servo ping [servo_id]

Enable/disable servo torque:
$ st3215_servo enable [servo_id]
$ st3215_servo disable [servo_id]

Control LED (0=off, 1=on, 2=blink):
$ st3215_servo led 1 [servo_id]

Set position with acceleration:
$ st3215_servo position_ex 1.57 2.0 100 [servo_id]

Set wheel mode and speed:
$ st3215_servo wheel_mode [servo_id]
$ st3215_servo speed 2.0 50 [servo_id]

Read/write registers:
$ st3215_servo read 0x18 [servo_id] [word]
$ st3215_servo write 0x18 1 [servo_id] [word]

Test UART communication:
$ st3215_servo uart_test /dev/ttyS1 /dev/ttyS2 [baud_rate]

Monitor ping transmission:
$ st3215_servo monitor_ping /dev/ttyS2 [servo_id]
(Monitors what is transmitted from main port by receiving on monitor port)

Run diagnostics:
$ st3215_servo diagnose [servo_id]

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
	PRINT_MODULE_USAGE_COMMAND_DESCR("position_ex", "Set position with speed and acceleration");
	PRINT_MODULE_USAGE_COMMAND_DESCR("wheel_mode", "Enable wheel mode for continuous rotation");
	PRINT_MODULE_USAGE_COMMAND_DESCR("speed", "Set wheel mode speed with acceleration");
	PRINT_MODULE_USAGE_COMMAND_DESCR("feedback", "Read all servo feedback data");
	PRINT_MODULE_USAGE_COMMAND_DESCR("read", "Read servo register (add 'word' for 16-bit read)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("write", "Write servo register (add 'word' for 16-bit write)");
	PRINT_MODULE_USAGE_COMMAND_DESCR("reset", "Factory reset servo");
	PRINT_MODULE_USAGE_COMMAND_DESCR("uart_test", "Test UART communication between two ports");
	PRINT_MODULE_USAGE_COMMAND_DESCR("monitor_ping", "Monitor ping transmission on different serial port");
	PRINT_MODULE_USAGE_COMMAND_DESCR("diagnose", "Run comprehensive connection diagnostic");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}
