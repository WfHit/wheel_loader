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
 * @file st3215_servo_core.cpp
 * @author PX4 Development Team
 *
 * Core driver functionality for ST3215 smart servo
 */

#include "st3215_servo.hpp"

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
