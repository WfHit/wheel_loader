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
 * @file st3215_servo.hpp
 * @author PX4 Development Team
 *
 * Driver for ST3215 smart servo
 */

#pragma once

#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <lib/perf/perf_counter.h>
#include <lib/parameters/param.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/robotic_servo_command.h>
#include <uORB/topics/robotic_servo_feedback.h>

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>

using namespace time_literals;

/**
 * @brief ST3215 Smart Servo Driver
 *
 * This driver controls a single ST3215 servo via UART communication.
 * It subscribes to servo commands and publishes servo feedback.
 */
class ST3215Servo : public ModuleBase<ST3215Servo>,
		    public ModuleParams,
		    public px4::ScheduledWorkItem
{
public:
	ST3215Servo();
	~ST3215Servo() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static ST3215Servo *instantiate(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase::print_status() */
	int print_status() override;

	bool init();

private:
	void Run() override;

	/**
	 * Process servo commands
	 */
	void process_commands();

	/**
	 * Publish servo feedback
	 */
	void publish_feedback();

	/**
	 * Send position command to servo
	 * @param position Target position in degrees
	 * @param speed Speed in degrees/second
	 * @return true if command sent successfully
	 */
	bool send_position_command(float position, float speed);

	/**
	 * Read servo status
	 * @return true if status read successfully
	 */
	bool read_servo_status();

	/**
	 * Calculate checksum for ST3215 protocol
	 * @param data Packet data
	 * @param length Data length
	 * @return Checksum value
	 */
	uint8_t calculate_checksum(const uint8_t *data, uint8_t length);

	/**
	 * Open serial port for communication
	 * @return true if opened successfully
	 */
	bool open_serial_port();

	/**
	 * Close serial port
	 */
	void close_serial_port();

	/**
	 * Send raw packet to servo
	 * @param data Packet data
	 * @param length Packet length
	 * @return true if sent successfully
	 */
	bool send_packet(const uint8_t *data, uint8_t length);

	/**
	 * Receive response packet from servo
	 * @param buffer Buffer to store received data
	 * @param max_length Maximum buffer length
	 * @return Number of bytes received, -1 on error
	 */
	int receive_packet(uint8_t *buffer, uint8_t max_length);

	// Serial communication
	int _serial_fd{-1};
	static constexpr size_t SERIAL_PORT_MAX_LEN = 32;
	char _serial_port[SERIAL_PORT_MAX_LEN]{"/dev/ttyS6"};

	// uORB subscriptions
	uORB::Subscription _servo_command_sub{ORB_ID(robotic_servo_command)};

	// uORB publications
	uORB::Publication<robotic_servo_feedback_s> _servo_feedback_pub{ORB_ID(robotic_servo_feedback)};

	// Performance counters
	perf_counter_t _loop_perf;
	perf_counter_t _comms_error_perf;

	// Servo state
	float _current_position{0.0f};		// Current position in degrees
	float _target_position{0.0f};		// Target position in degrees
	float _current_speed{0.0f};		// Current speed in degrees/second
	float _current_load{0.0f};		// Current load percentage
	uint8_t _current_temperature{0};	// Current temperature in Celsius
	bool _servo_enabled{false};		// Servo enabled state
	hrt_abstime _last_command_time{0};	// Last command timestamp

	// Communication buffers
	static constexpr size_t BUFFER_SIZE = 32;
	uint8_t _tx_buffer[BUFFER_SIZE];
	uint8_t _rx_buffer[BUFFER_SIZE];

	// ST3215 Protocol constants
	static constexpr uint8_t ST3215_HEADER = 0xFF;
	static constexpr uint8_t ST3215_HEADER2 = 0xFF;
	static constexpr uint8_t ST3215_ID_DEFAULT = 1;
	static constexpr uint8_t ST3215_CMD_WRITE = 0x03;
	static constexpr uint8_t ST3215_CMD_READ = 0x02;
	static constexpr uint8_t ST3215_REG_GOAL_POSITION = 0x2A;
	static constexpr uint8_t ST3215_REG_GOAL_SPEED = 0x2E;
	static constexpr uint8_t ST3215_REG_PRESENT_POSITION = 0x38;
	static constexpr uint8_t ST3215_REG_PRESENT_SPEED = 0x3A;
	static constexpr uint8_t ST3215_REG_PRESENT_LOAD = 0x3C;
	static constexpr uint8_t ST3215_REG_PRESENT_TEMP = 0x3F;
	static constexpr uint8_t ST3215_REG_TORQUE_ENABLE = 0x28;

	// Timing constants
	static constexpr unsigned SCHEDULE_INTERVAL = 20_ms;	// 50 Hz update rate
	static constexpr hrt_abstime COMMAND_TIMEOUT = 1_s;	// Command timeout

	// Parameters (following 16-character limit)
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::ST3215_ID>) _param_servo_id,
		(ParamInt<px4::params::ST3215_BAUDRATE>) _param_baudrate,
		(ParamFloat<px4::params::ST3215_MIN_POS>) _param_min_position,
		(ParamFloat<px4::params::ST3215_MAX_POS>) _param_max_position,
		(ParamFloat<px4::params::ST3215_MAX_SPD>) _param_max_speed
	)
};
