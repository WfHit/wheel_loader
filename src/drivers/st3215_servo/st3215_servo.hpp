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
 * Simplified ST3215 smart servo driver header
 * Uses SerialPort API like UWB SR150 for robust UART communication
 */

#pragma once

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/robotic_servo_command.h>
#include <uORB/topics/robotic_servo_feedback.h>

#include <poll.h>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <sys/select.h>

using namespace time_literals;

class ST3215Servo : public ModuleBase<ST3215Servo>,
		    public ModuleParams,
		    public px4::ScheduledWorkItem
{
public:
	ST3215Servo(const char *serial_port = "/dev/ttyS1");
	~ST3215Servo() override;

	static int task_spawn(int argc, char *argv[]);
	static ST3215Servo *instantiate(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	int print_status() override;
	bool init();

	// Add diagnostic methods
	void run_diagnostics();
	void test_raw_communication();

private:
	void Run() override;

	// Serial communication using device::SerialPort like UWB SR150
	bool configure_port();
	bool send_packet(const uint8_t *data, uint8_t length);
	int collect_packet(uint32_t timeout_ms = 10);
	bool parse_packet();
	uint8_t calculate_checksum(const uint8_t *data, uint8_t length);

	// Basic servo operations
	bool ping_servo(uint8_t servo_id);
	bool write_position(uint8_t servo_id, float position_rad, float speed_rad_s);
	bool read_status(uint8_t servo_id);
	bool set_torque_enable(uint8_t servo_id, bool enable);

	// Helper functions
	bool read_register(uint8_t servo_id, uint8_t reg_addr, uint8_t *data, uint8_t length);
	bool write_register(uint8_t servo_id, uint8_t reg_addr, const uint8_t *data, uint8_t length);

	void process_commands();
	void publish_feedback();

	// Serial port using file descriptor (like UWB SR150)
	int _uart{-1};
	char _port_name[32];
	fd_set _uart_set;
	struct timeval _uart_timeout{};

	// Packet parsing state machine (similar to UWB SR150)
	enum class ParseState {
		WAIT_HEADER1,
		WAIT_HEADER2,
		WAIT_ID,
		WAIT_LENGTH,
		WAIT_ERROR,
		WAIT_DATA,
		WAIT_CHECKSUM
	};

	ParseState _parse_state{ParseState::WAIT_HEADER1};
	uint8_t _rx_buffer[256];           // Circular buffer for incoming data
	uint8_t _rx_packet[256];           // Complete packet buffer
	uint16_t _rx_buffer_head{0};       // Head pointer for circular buffer
	uint16_t _rx_buffer_tail{0};       // Tail pointer for circular buffer
	uint8_t _rx_packet_length{0};      // Length of received packet
	uint8_t _expected_data_length{0};  // Expected data length in current packet
	uint8_t _rx_data_count{0};         // Current data byte count

	// Command response handling
	bool _waiting_for_response{false};
	uint8_t _expected_response_id{0};
	uint8_t _expected_response_cmd{0};

	// uORB topics
	uORB::Subscription _servo_command_sub{ORB_ID(robotic_servo_command)};
	uORB::Publication<robotic_servo_feedback_s> _servo_feedback_pub{ORB_ID(robotic_servo_feedback)};

	// Performance counters
	perf_counter_t _loop_perf;
	perf_counter_t _comms_error_perf;
	perf_counter_t _packet_count_perf;

	// Current servo state
	float _current_position{0.0f};     // Current position in radians
	float _current_speed{0.0f};        // Current speed in rad/s
	float _current_load{0.0f};         // Current load percentage
	uint8_t _current_temperature{0};   // Current temperature in degrees C
	uint8_t _current_voltage{0};       // Current voltage in 0.1V units
	bool _servo_enabled{false};        // Torque enable state
	bool _connection_ok{false};        // Communication status
	hrt_abstime _last_update_time{0};  // Last successful update time

	// ST3215 Protocol constants
	static constexpr uint8_t ST3215_HEADER = 0xFF;
	static constexpr uint8_t ST3215_HEADER2 = 0xFF;

	// Instruction set
	static constexpr uint8_t ST3215_CMD_PING = 0x01;
	static constexpr uint8_t ST3215_CMD_READ = 0x02;
	static constexpr uint8_t ST3215_CMD_WRITE = 0x03;

	// Essential register addresses
	static constexpr uint8_t ST3215_REG_ID = 0x03;
	static constexpr uint8_t ST3215_REG_TORQUE_ENABLE = 0x18;
	static constexpr uint8_t ST3215_REG_GOAL_POSITION_L = 0x1E;
	static constexpr uint8_t ST3215_REG_GOAL_POSITION_H = 0x1F;
	static constexpr uint8_t ST3215_REG_MOVING_SPEED_L = 0x20;
	static constexpr uint8_t ST3215_REG_MOVING_SPEED_H = 0x21;
	static constexpr uint8_t ST3215_REG_PRESENT_POSITION_L = 0x24;
	static constexpr uint8_t ST3215_REG_PRESENT_POSITION_H = 0x25;
	static constexpr uint8_t ST3215_REG_PRESENT_SPEED_L = 0x26;
	static constexpr uint8_t ST3215_REG_PRESENT_SPEED_H = 0x27;
	static constexpr uint8_t ST3215_REG_PRESENT_LOAD_L = 0x28;
	static constexpr uint8_t ST3215_REG_PRESENT_LOAD_H = 0x29;
	static constexpr uint8_t ST3215_REG_PRESENT_VOLTAGE = 0x2A;
	static constexpr uint8_t ST3215_REG_PRESENT_TEMP = 0x2B;

	// Timing constants
	static constexpr unsigned SCHEDULE_INTERVAL = 20_ms;
	static constexpr uint32_t PACKET_TIMEOUT_MS = 50;

	// Module parameters
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::ST3215_ID>) _param_servo_id,
		(ParamInt<px4::params::ST3215_BAUDRATE>) _param_baudrate,
		(ParamFloat<px4::params::ST3215_MIN_POS>) _param_min_position,
		(ParamFloat<px4::params::ST3215_MAX_POS>) _param_max_position,
		(ParamFloat<px4::params::ST3215_MAX_SPD>) _param_max_speed
	)
};
