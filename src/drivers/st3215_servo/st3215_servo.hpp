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
#include <px4_platform_common/getopt.h>

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
	ST3215Servo(const char *serial_port = "/dev/ttyS1");
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

	/**
	 * Set compliance parameters
	 * @param servo_id Servo ID
	 * @param cw_margin CW margin
	 * @param ccw_margin CCW margin
	 * @param cw_slope CW slope
	 * @param ccw_slope CCW slope
	 * @return true if successful
	 */
	bool set_compliance_params(uint8_t servo_id, uint8_t cw_margin, uint8_t ccw_margin, uint8_t cw_slope, uint8_t ccw_slope);

	/**
	 * Set return delay time
	 * @param servo_id Servo ID
	 * @param delay_time Delay time in microseconds
	 * @return true if successful
	 */
	bool set_return_delay(uint8_t servo_id, uint8_t delay_time);

	/**
	 * Set status return level
	 * @param servo_id Servo ID
	 * @param level Status return level (0=none, 1=read only, 2=all)
	 * @return true if successful
	 */
	bool set_status_return_level(uint8_t servo_id, uint8_t level);

	/**
	 * Set alarm LED
	 * @param servo_id Servo ID
	 * @param alarm_led Alarm LED configuration
	 * @return true if successful
	 */
	bool set_alarm_led(uint8_t servo_id, uint8_t alarm_led);

	/**
	 * Set alarm shutdown
	 * @param servo_id Servo ID
	 * @param alarm_shutdown Alarm shutdown configuration
	 * @return true if successful
	 */
	bool set_alarm_shutdown(uint8_t servo_id, uint8_t alarm_shutdown);

	/**
	 * Set angle limits
	 * @param servo_id Servo ID
	 * @param min_angle Minimum angle in radians
	 * @param max_angle Maximum angle in radians
	 * @return true if successful
	 */
	bool set_angle_limits(uint8_t servo_id, float min_angle, float max_angle);

	/**
	 * Set voltage limits
	 * @param servo_id Servo ID
	 * @param min_voltage Minimum voltage in 0.1V units
	 * @param max_voltage Maximum voltage in 0.1V units
	 * @return true if successful
	 */
	bool set_voltage_limits(uint8_t servo_id, uint8_t min_voltage, uint8_t max_voltage);

	/**
	 * Set temperature limit
	 * @param servo_id Servo ID
	 * @param max_temperature Maximum temperature in Celsius
	 * @return true if successful
	 */
	bool set_temperature_limit(uint8_t servo_id, uint8_t max_temperature);

	// UART test helpers
	static int open_uart_port(const char *port, int baud_rate);
	static void close_uart_port(int fd);
	static bool test_uart_loopback(int fd1, int fd2);
	static bool test_uart_bidirectional(int fd1, int fd2);
	static bool test_uart_performance(int fd1, int fd2);

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
	 * Ping servo to check connectivity
	 * @param servo_id Servo ID to ping
	 * @return true if servo responds
	 */
	bool ping_servo(uint8_t servo_id);

	/**
	 * Read single byte from servo register
	 * @param servo_id Servo ID
	 * @param reg_addr Register address
	 * @param value Pointer to store read value
	 * @return true if read successful
	 */
	bool read_byte(uint8_t servo_id, uint8_t reg_addr, uint8_t *value);

	/**
	 * Read word (2 bytes) from servo register
	 * @param servo_id Servo ID
	 * @param reg_addr Register address
	 * @param value Pointer to store read value
	 * @return true if read successful
	 */
	bool read_word(uint8_t servo_id, uint8_t reg_addr, uint16_t *value);

	/**
	 * Write single byte to servo register
	 * @param servo_id Servo ID
	 * @param reg_addr Register address
	 * @param value Value to write
	 * @return true if write successful
	 */
	bool write_byte(uint8_t servo_id, uint8_t reg_addr, uint8_t value);

	/**
	 * Write word (2 bytes) to servo register
	 * @param servo_id Servo ID
	 * @param reg_addr Register address
	 * @param value Value to write
	 * @return true if write successful
	 */
	bool write_word(uint8_t servo_id, uint8_t reg_addr, uint16_t value);

	/**
	 * Enable/disable servo torque
	 * @param servo_id Servo ID
	 * @param enable True to enable, false to disable
	 * @return true if successful
	 */
	bool set_torque_enable(uint8_t servo_id, bool enable);

	/**
	 * Set servo LED state
	 * @param servo_id Servo ID
	 * @param state LED state (0=off, 1=on, 2=blink)
	 * @return true if successful
	 */
	bool set_led(uint8_t servo_id, uint8_t state);

	/**
	 * Read all servo status (position, speed, load, temperature, voltage)
	 * @param servo_id Servo ID
	 * @return true if successful
	 */
	bool read_full_status(uint8_t servo_id);

	/**
	 * Set servo ID
	 * @param current_id Current servo ID
	 * @param new_id New servo ID
	 * @return true if successful
	 */
	bool set_servo_id(uint8_t current_id, uint8_t new_id);

	/**
	 * Set servo baud rate
	 * @param servo_id Servo ID
	 * @param baud_rate Baud rate (0-7, see manual)
	 * @return true if successful
	 */
	bool set_baud_rate(uint8_t servo_id, uint8_t baud_rate);

	/**
	 * Reset servo to factory settings
	 * @param servo_id Servo ID
	 * @return true if successful
	 */
	bool factory_reset(uint8_t servo_id);

	/**
	 * Enhanced position control with acceleration (like WritePosEx from SCServo)
	 * @param servo_id Servo ID
	 * @param position Target position in radians
	 * @param speed Speed in rad/s
	 * @param acceleration Acceleration value (0-254)
	 * @return true if successful
	 */
	bool write_position_ex(uint8_t servo_id, float position, float speed, uint8_t acceleration = 0);

	/**
	 * Set servo to wheel mode (continuous rotation)
	 * @param servo_id Servo ID
	 * @return true if successful
	 */
	bool set_wheel_mode(uint8_t servo_id);

	/**
	 * Control servo speed in wheel mode
	 * @param servo_id Servo ID
	 * @param speed Speed in rad/s (positive=CW, negative=CCW)
	 * @param acceleration Acceleration value
	 * @return true if successful
	 */
	bool write_speed(uint8_t servo_id, float speed, uint8_t acceleration = 0);

	/**
	 * Set torque limit
	 * @param servo_id Servo ID
	 * @param torque_limit Torque limit (0-1000, where 1000 = 100%)
	 * @return true if successful
	 */
	bool set_torque_limit(uint8_t servo_id, uint16_t torque_limit);

	/**
	 * Comprehensive feedback reading (like FeedBack from SCServo)
	 * @param servo_id Servo ID
	 * @return true if successful
	 */
	bool feedback(uint8_t servo_id);

	/**
	 * Read current consumption
	 * @param servo_id Servo ID
	 * @return Current in mA, -1 if failed
	 */
	int read_current(uint8_t servo_id);

	/**
	 * Unlock EPROM for writing
	 * @param servo_id Servo ID
	 * @return true if successful
	 */
	bool unlock_eprom(uint8_t servo_id);

	/**
	 * Lock EPROM to prevent accidental writes
	 * @param servo_id Servo ID
	 * @return true if successful
	 */
	bool lock_eprom(uint8_t servo_id);

	/**
	 * Calibrate servo center position
	 * @param servo_id Servo ID
	 * @return true if successful
	 */
	bool calibrate_center(uint8_t servo_id);

	/**
	 * Test UART communication between two serial ports
	 * @param port1 Path to first serial port (e.g., "/dev/ttyS1")
	 * @param port2 Path to second serial port (e.g., "/dev/ttyS2")
	 * @param baud_rate Baud rate for communication (default: 115200)
	 * @return true if communication test successful
	 */
	static bool uart_test(const char *port1, const char *port2, int baud_rate = 115200);

	/**
	 * Set position PID gains
	 * @param servo_id Servo ID
	 * @param p_gain Proportional gain
	 * @param i_gain Integral gain
	 * @param d_gain Derivative gain
	 * @return true if successful
	 */
	bool set_position_pid(uint8_t servo_id, uint8_t p_gain, uint8_t i_gain, uint8_t d_gain);

	/**
	 * Set velocity PID gains
	 * @param servo_id Servo ID
	 * @param p_gain Proportional gain
	 * @param i_gain Integral gain
	 * @return true if successful
	 */
	bool set_velocity_pid(uint8_t servo_id, uint8_t p_gain, uint8_t i_gain);

	/**
	 * Open and configure UART test ports
	 * @param port1 Path to first serial port
	 * @param port2 Path to second serial port
	 * @param baud_rate Baud rate for communication
	 * @param fd1 Pointer to store first file descriptor
	 * @param fd2 Pointer to store second file descriptor
	 * @return true if successful
	 */
	bool uart_test_open_ports(const char *port1, const char *port2, uint32_t baud_rate, int *fd1, int *fd2);

	/**
	 * Close UART test ports
	 * @param fd1 First file descriptor
	 * @param fd2 Second file descriptor
	 */
	void uart_test_close_ports(int fd1, int fd2);

	/**
	 * Configure a single UART port for testing
	 * @param fd File descriptor
	 * @param baud_rate Baud rate for communication
	 * @return true if successful
	 */
	bool uart_test_configure_port(int fd, uint32_t baud_rate);

	/**
	 * Perform communication test between two ports
	 * @param fd1 First port file descriptor
	 * @param fd2 Second port file descriptor
	 * @return Number of successful tests
	 */
	int uart_test_communication(int fd1, int fd2);

	/**
	 * Send message on one port and receive on another
	 * @param tx_fd Transmit file descriptor
	 * @param rx_fd Receive file descriptor
	 * @param message Message to send
	 * @param test_num Test number (0 for reverse test)
	 * @param total_tests Total number of tests
	 * @return true if successful
	 */
	bool uart_test_send_receive(int tx_fd, int rx_fd, const char *message, int test_num, int total_tests);

	/**
	 * Print UART test results summary
	 * @param success_count Number of successful tests
	 */
	void uart_test_print_results(int success_count);

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
	 * Verify serial port configuration and status
	 * @return true if port is properly configured and accessible
	 */
	bool verify_serial_port();

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

	/**
	 * Monitor ping transmission by receiving on a different serial port
	 * @param monitor_port Path to monitor serial port (e.g., "/dev/ttyS2")
	 * @param servo_id Servo ID to ping on main port
	 * @return true if correct ping packet is detected on monitor port
	 */
	bool monitor_ping_transmission(const char *monitor_port, uint8_t servo_id);

	// Serial communication
	int _serial_fd{-1};
	static constexpr size_t SERIAL_PORT_MAX_LEN = 32;
	char _serial_port[SERIAL_PORT_MAX_LEN]{"/dev/ttyS1"};

	// uORB subscriptions
	uORB::Subscription _servo_command_sub{ORB_ID(robotic_servo_command)};

	// uORB publications
	uORB::Publication<robotic_servo_feedback_s> _servo_feedback_pub{ORB_ID(robotic_servo_feedback)};

	// Performance counters
	perf_counter_t _loop_perf;
	perf_counter_t _comms_error_perf;

	// Servo state
	float _current_position{0.0f};		// Current position in radians
	float _target_position{0.0f};		// Target position in radians
	float _current_speed{0.0f};		// Current speed in rad/s
	float _current_load{0.0f};		// Current load percentage
	uint8_t _current_temperature{0};	// Current temperature in Celsius
	uint8_t _current_voltage{0};		// Current voltage in 0.1V units
	uint8_t _error_status{0};		// Error status byte
	bool _servo_enabled{false};		// Servo enabled state
	bool _is_moving{false};			// Servo is moving
	hrt_abstime _last_command_time{0};	// Last command timestamp
	hrt_abstime _last_ping_time{0};		// Last successful ping time
	bool _connection_ok{false};		// Connection status

	// Communication buffers
	static constexpr size_t BUFFER_SIZE = 32;
	uint8_t _tx_buffer[BUFFER_SIZE];
	uint8_t _rx_buffer[BUFFER_SIZE];

	// ST3215 Protocol constants
	static constexpr uint8_t ST3215_HEADER = 0xFF;
	static constexpr uint8_t ST3215_HEADER2 = 0xFF;
	static constexpr uint8_t ST3215_ID_DEFAULT = 1;
	static constexpr uint8_t ST3215_ID_BROADCAST = 0xFE;

	// Command instructions
	static constexpr uint8_t ST3215_CMD_PING = 0x01;
	static constexpr uint8_t ST3215_CMD_READ = 0x02;
	static constexpr uint8_t ST3215_CMD_WRITE = 0x03;
	static constexpr uint8_t ST3215_CMD_REG_WRITE = 0x04;
	static constexpr uint8_t ST3215_CMD_ACTION = 0x05;
	static constexpr uint8_t ST3215_CMD_RESET = 0x06;
	static constexpr uint8_t ST3215_CMD_SYNC_WRITE = 0x83;

	// EEPROM registers (non-volatile)
	static constexpr uint8_t ST3215_REG_MODEL_NUMBER_L = 0x00;
	static constexpr uint8_t ST3215_REG_MODEL_NUMBER_H = 0x01;
	static constexpr uint8_t ST3215_REG_VERSION = 0x02;
	static constexpr uint8_t ST3215_REG_ID = 0x03;
	static constexpr uint8_t ST3215_REG_BAUD_RATE = 0x04;
	static constexpr uint8_t ST3215_REG_RETURN_DELAY = 0x05;
	static constexpr uint8_t ST3215_REG_CW_ANGLE_LIMIT_L = 0x06;
	static constexpr uint8_t ST3215_REG_CW_ANGLE_LIMIT_H = 0x07;
	static constexpr uint8_t ST3215_REG_CCW_ANGLE_LIMIT_L = 0x08;
	static constexpr uint8_t ST3215_REG_CCW_ANGLE_LIMIT_H = 0x09;
	static constexpr uint8_t ST3215_REG_TEMP_LIMIT = 0x0B;
	static constexpr uint8_t ST3215_REG_MIN_VOLTAGE = 0x0C;
	static constexpr uint8_t ST3215_REG_MAX_VOLTAGE = 0x0D;
	static constexpr uint8_t ST3215_REG_MAX_TORQUE_L = 0x0E;
	static constexpr uint8_t ST3215_REG_MAX_TORQUE_H = 0x0F;
	static constexpr uint8_t ST3215_REG_STATUS_RETURN_LEVEL = 0x10;
	static constexpr uint8_t ST3215_REG_ALARM_LED = 0x11;
	static constexpr uint8_t ST3215_REG_ALARM_SHUTDOWN = 0x12;
	static constexpr uint8_t ST3215_REG_MAX_TEMP_LIMIT = 0x0B;
	static constexpr uint8_t ST3215_REG_MIN_VOLTAGE_LIMIT = 0x0C;
	static constexpr uint8_t ST3215_REG_MAX_VOLTAGE_LIMIT = 0x0D;

	// RAM registers (volatile)
	static constexpr uint8_t ST3215_REG_TORQUE_ENABLE = 0x18;
	static constexpr uint8_t ST3215_REG_LED = 0x19;
	static constexpr uint8_t ST3215_REG_D_GAIN = 0x1A;
	static constexpr uint8_t ST3215_REG_I_GAIN = 0x1B;
	static constexpr uint8_t ST3215_REG_P_GAIN = 0x1C;
	static constexpr uint8_t ST3215_REG_GOAL_POSITION_L = 0x1E;
	static constexpr uint8_t ST3215_REG_GOAL_POSITION_H = 0x1F;
	static constexpr uint8_t ST3215_REG_MOVING_SPEED_L = 0x20;
	static constexpr uint8_t ST3215_REG_MOVING_SPEED_H = 0x21;
	static constexpr uint8_t ST3215_REG_TORQUE_LIMIT_L = 0x22;
	static constexpr uint8_t ST3215_REG_TORQUE_LIMIT_H = 0x23;
	static constexpr uint8_t ST3215_REG_PRESENT_POSITION_L = 0x24;
	static constexpr uint8_t ST3215_REG_PRESENT_POSITION_H = 0x25;
	static constexpr uint8_t ST3215_REG_PRESENT_SPEED_L = 0x26;
	static constexpr uint8_t ST3215_REG_PRESENT_SPEED_H = 0x27;
	static constexpr uint8_t ST3215_REG_PRESENT_LOAD_L = 0x28;
	static constexpr uint8_t ST3215_REG_PRESENT_LOAD_H = 0x29;
	static constexpr uint8_t ST3215_REG_PRESENT_VOLTAGE = 0x2A;
	static constexpr uint8_t ST3215_REG_PRESENT_TEMP = 0x2B;
	static constexpr uint8_t ST3215_REG_REGISTERED = 0x2C;
	static constexpr uint8_t ST3215_REG_MOVING = 0x2E;
	static constexpr uint8_t ST3215_REG_LOCK = 0x2F;
	static constexpr uint8_t ST3215_REG_PUNCH_L = 0x30;
	static constexpr uint8_t ST3215_REG_PUNCH_H = 0x31;

	// Additional SCServo-compatible registers
	static constexpr uint8_t ST3215_REG_MIN_ANGLE_LIMIT_L = 0x09;
	static constexpr uint8_t ST3215_REG_MIN_ANGLE_LIMIT_H = 0x0A;
	static constexpr uint8_t ST3215_REG_MAX_ANGLE_LIMIT_L = 0x0B;
	static constexpr uint8_t ST3215_REG_MAX_ANGLE_LIMIT_H = 0x0C;
	static constexpr uint8_t ST3215_REG_CW_DEAD = 0x1A;
	static constexpr uint8_t ST3215_REG_CCW_DEAD = 0x1B;
	static constexpr uint8_t ST3215_REG_OFS_L = 0x1F;
	static constexpr uint8_t ST3215_REG_OFS_H = 0x20;
	static constexpr uint8_t ST3215_REG_MODE = 0x21;

	// SRAM registers (SCServo-compatible)
	static constexpr uint8_t ST3215_REG_ACC = 0x29;
	static constexpr uint8_t ST3215_REG_GOAL_POSITION_L_NEW = 0x2A;
	static constexpr uint8_t ST3215_REG_GOAL_POSITION_H_NEW = 0x2B;
	static constexpr uint8_t ST3215_REG_GOAL_TIME_L = 0x2C;
	static constexpr uint8_t ST3215_REG_GOAL_TIME_H = 0x2D;
	static constexpr uint8_t ST3215_REG_GOAL_SPEED_L = 0x2E;
	static constexpr uint8_t ST3215_REG_GOAL_SPEED_H = 0x2F;
	static constexpr uint8_t ST3215_REG_TORQUE_LIMIT_L_NEW = 0x30;
	static constexpr uint8_t ST3215_REG_TORQUE_LIMIT_H_NEW = 0x31;

	// Feedback registers (SCServo-compatible)
	static constexpr uint8_t ST3215_REG_PRESENT_POSITION_L_NEW = 0x38;
	static constexpr uint8_t ST3215_REG_PRESENT_POSITION_H_NEW = 0x39;
	static constexpr uint8_t ST3215_REG_PRESENT_SPEED_L_NEW = 0x3A;
	static constexpr uint8_t ST3215_REG_PRESENT_SPEED_H_NEW = 0x3B;
	static constexpr uint8_t ST3215_REG_PRESENT_LOAD_L_NEW = 0x3C;
	static constexpr uint8_t ST3215_REG_PRESENT_LOAD_H_NEW = 0x3D;
	static constexpr uint8_t ST3215_REG_PRESENT_VOLTAGE_NEW = 0x3E;
	static constexpr uint8_t ST3215_REG_PRESENT_TEMP_NEW = 0x3F;
	static constexpr uint8_t ST3215_REG_MOVING_NEW = 0x42;
	static constexpr uint8_t ST3215_REG_PRESENT_CURRENT_L = 0x45;
	static constexpr uint8_t ST3215_REG_PRESENT_CURRENT_H = 0x46;

	// Legacy register names for compatibility
	static constexpr uint8_t ST3215_REG_GOAL_POSITION = ST3215_REG_GOAL_POSITION_L;
	static constexpr uint8_t ST3215_REG_GOAL_SPEED = ST3215_REG_MOVING_SPEED_L;
	static constexpr uint8_t ST3215_REG_PRESENT_POSITION = ST3215_REG_PRESENT_POSITION_L;
	static constexpr uint8_t ST3215_REG_PRESENT_SPEED = ST3215_REG_PRESENT_SPEED_L;
	static constexpr uint8_t ST3215_REG_PRESENT_LOAD = ST3215_REG_PRESENT_LOAD_L;

	// Error codes
	static constexpr uint8_t ST3215_ERROR_NONE = 0x00;
	static constexpr uint8_t ST3215_ERROR_INPUT_VOLTAGE = 0x01;
	static constexpr uint8_t ST3215_ERROR_ANGLE_LIMIT = 0x02;
	static constexpr uint8_t ST3215_ERROR_OVERHEATING = 0x04;
	static constexpr uint8_t ST3215_ERROR_RANGE = 0x08;
	static constexpr uint8_t ST3215_ERROR_CHECKSUM = 0x10;
	static constexpr uint8_t ST3215_ERROR_OVERLOAD = 0x20;
	static constexpr uint8_t ST3215_ERROR_INSTRUCTION = 0x40;

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
