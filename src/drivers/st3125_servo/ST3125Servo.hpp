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

#pragma once

#include <termios.h>

#include <drivers/drv_hrt.h>
#include <lib/drivers/device/device.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/PublicationMulti.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/robotic_servo_command.h>
#include <uORB/topics/servo_feedback.h>
#include <fcntl.h>
#include <unistd.h>

using namespace time_literals;

/**
 * @brief ST3125 Smart Servo Driver
 *
 * This driver interfaces with ST3125 smart servos via serial communication.
 * It supports position control, velocity control, and provides feedback
 * including position, velocity, current, voltage, and temperature.
 */
class ST3125Servo : public ModuleBase<ST3125Servo>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
    ST3125Servo();
    ~ST3125Servo() override;

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    /** @see ModuleBase::print_status() */
    int print_status() override;

    bool init();
    void Run() override;

private:
    // ST3125 Protocol definitions
    static constexpr uint8_t HEADER = 0xFF;
    static constexpr uint8_t HEADER2 = 0xFF;

    // Instruction set
    enum Instruction : uint8_t {
        INST_PING = 0x01,
        INST_READ = 0x02,
        INST_WRITE = 0x03,
        INST_REG_WRITE = 0x04,
        INST_ACTION = 0x05,
        INST_RESET = 0x06,
        INST_SYNC_WRITE = 0x83,
    };

    // Memory addresses
    enum MemoryAddress : uint8_t {
        ADDR_ID = 0x05,
        ADDR_BAUD_RATE = 0x06,
        ADDR_MIN_POSITION_LIMIT = 0x09,
        ADDR_MAX_POSITION_LIMIT = 0x0B,
        ADDR_MAX_TEMPERATURE = 0x0D,
        ADDR_MAX_VOLTAGE = 0x0E,
        ADDR_MIN_VOLTAGE = 0x0F,
        ADDR_MAX_TORQUE = 0x10,
        ADDR_STATUS_RETURN = 0x11,
        ADDR_ALARM_LED = 0x12,
        ADDR_ALARM_SHUTDOWN = 0x13,
        ADDR_TORQUE_ENABLE = 0x28,
        ADDR_LED = 0x19,
        ADDR_GOAL_POSITION = 0x2A,
        ADDR_GOAL_TIME = 0x2C,
        ADDR_GOAL_SPEED = 0x2E,
        ADDR_LOCK = 0x30,
        ADDR_PRESENT_POSITION = 0x38,
        ADDR_PRESENT_SPEED = 0x3A,
        ADDR_PRESENT_LOAD = 0x3C,
        ADDR_PRESENT_VOLTAGE = 0x3E,
        ADDR_PRESENT_TEMPERATURE = 0x3F,
        ADDR_PRESENT_CURRENT = 0x45,
    };

    // Servo configuration
    struct ServoInfo {
        uint8_t id;
        bool connected;
        float min_position;
        float max_position;
        float position_offset;
        float position_scale;
        uint16_t last_position_raw;
        float last_position;
        float last_velocity;
        uint32_t last_update_time;
        uint8_t error_count;
    };

    // Maximum number of servos
    static constexpr uint8_t MAX_SERVOS = 12;

    // Serial communication
    int _serial_fd{-1};
    char _serial_port[32];
    uint32_t _baudrate{1000000};
    uint8_t _rx_buffer[256];
    uint8_t _tx_buffer[256];

    // Servo management
    ServoInfo _servos[MAX_SERVOS];
    uint8_t _num_servos{0};
    uint8_t _active_servo_ids[MAX_SERVOS];

    // Publications
    uORB::PublicationMulti<servo_feedback_s> _servo_feedback_pub{ORB_ID(servo_feedback)};

    // Subscriptions
    uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
    uORB::Subscription _robotic_servo_command_sub{ORB_ID(robotic_servo_command)};

    // Performance counters
    perf_counter_t _cycle_perf;
    perf_counter_t _interval_perf;
    perf_counter_t _comms_errors;
    perf_counter_t _checksum_errors;

    // Parameters
    DEFINE_PARAMETERS(
        (ParamInt<px4::params::ST3125_BAUDRATE>) _param_baudrate,
        (ParamInt<px4::params::ST3125_UPD_RAT>) _param_update_rate,
        (ParamInt<px4::params::ST3125_POS_MIN>) _param_pos_min,
        (ParamInt<px4::params::ST3125_POS_MAX>) _param_pos_max,
        (ParamInt<px4::params::ST3125_TORQUE_EN>) _param_torque_enable
    )

    // Internal methods
    bool open_serial();
    void close_serial();
    bool configure_servo(uint8_t id);
    bool ping_servo(uint8_t id);
    bool scan_servos();
    void update_robotic_servo_commands();

    // Protocol methods
    uint8_t calculate_checksum(const uint8_t *packet, uint8_t length);
    int send_packet(uint8_t id, uint8_t instruction, const uint8_t *params, uint8_t param_length);
    int receive_packet(uint8_t *buffer, uint8_t &id, uint8_t &error, uint8_t &param_length);

    // Read operations
    bool read_byte(uint8_t id, uint8_t address, uint8_t &value);
    bool read_word(uint8_t id, uint8_t address, uint16_t &value);
    bool read_dword(uint8_t id, uint8_t address, uint32_t &value);

    // Write operations
    bool write_byte(uint8_t id, uint8_t address, uint8_t value);
    bool write_word(uint8_t id, uint8_t address, uint16_t value);
    bool write_dword(uint8_t id, uint8_t address, uint32_t value);

    // Servo control
    bool set_position(uint8_t id, float position);
    bool set_velocity(uint8_t id, float velocity);
    bool set_torque_enable(uint8_t id, bool enable);
    bool sync_write_positions(const float *positions);

    // Feedback reading
    bool read_position(uint8_t id, float &position);
    bool read_velocity(uint8_t id, float &velocity);
    bool read_current(uint8_t id, float &current);
    bool read_voltage(uint8_t id, float &voltage);
    bool read_temperature(uint8_t id, float &temperature);
    bool read_all_feedback();

    // Update methods
    void publish_servo_feedback();
};
