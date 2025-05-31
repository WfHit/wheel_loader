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

#include "ST3125Servo.hpp"
#include <px4_platform_common/log.h>
#include <lib/mathlib/mathlib.h>
#include <matrix/matrix/math.hpp>

ST3125Servo::ST3125Servo() :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default),
    _cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")),
    _interval_perf(perf_alloc(PC_INTERVAL, MODULE_NAME": interval")),
    _comms_errors(perf_alloc(PC_COUNT, MODULE_NAME": comms errors")),
    _checksum_errors(perf_alloc(PC_COUNT, MODULE_NAME": checksum errors"))
{
    strcpy(_serial_port, "/dev/ttyS3"); // Default serial port
}

ST3125Servo::~ST3125Servo()
{
    ScheduleClear();
    close_serial();

    perf_free(_cycle_perf);
    perf_free(_interval_perf);
    perf_free(_comms_errors);
    perf_free(_checksum_errors);
}

bool ST3125Servo::init()
{
    // Update parameters
    updateParams();

    // Open serial port
    if (!open_serial()) {
        PX4_ERR("Failed to open serial port");
        return false;
    }

    // Scan for servos
    if (!scan_servos()) {
        PX4_ERR("No servos found");
        close_serial();
        return false;
    }

    PX4_INFO("Found %d servos", _num_servos);

    // Configure each servo
    for (uint8_t i = 0; i < _num_servos; i++) {
        if (!configure_servo(_active_servo_ids[i])) {
            PX4_WARN("Failed to configure servo ID %d", _active_servo_ids[i]);
        }
    }

    // Set torque enable based on parameter
    for (uint8_t i = 0; i < _num_servos; i++) {
        set_torque_enable(_active_servo_ids[i], _param_torque_enable.get());
    }

    // Schedule updates
    ScheduleOnInterval(_param_update_rate.get() > 0 ? (1_s / _param_update_rate.get()) : 20_ms);

    return true;
}

bool ST3125Servo::open_serial()
{
    // Close if already open
    if (_serial_fd >= 0) {
        close(_serial_fd);
    }

    // Open serial port
    _serial_fd = open(_serial_port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (_serial_fd < 0) {
        PX4_ERR("Failed to open serial port %s", _serial_port);
        return false;
    }

    // Configure serial port
    struct termios uart_config;
    tcgetattr(_serial_fd, &uart_config);

    // Set baudrate
    speed_t speed;
    switch (_param_baudrate.get()) {
        case 9600:    speed = B9600; break;
        case 19200:   speed = B19200; break;
        case 38400:   speed = B38400; break;
        case 57600:   speed = B57600; break;
        case 115200:  speed = B115200; break;
        case 230400:  speed = B230400; break;
        case 460800:  speed = B460800; break;
        case 921600:  speed = B921600; break;
        default:
            speed = B1000000;
            break;
    }

    cfsetispeed(&uart_config, speed);
    cfsetospeed(&uart_config, speed);

    // 8N1
    uart_config.c_cflag &= ~PARENB;
    uart_config.c_cflag &= ~CSTOPB;
    uart_config.c_cflag &= ~CSIZE;
    uart_config.c_cflag |= CS8;
    uart_config.c_cflag |= (CLOCAL | CREAD);

    // Raw mode
    uart_config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    uart_config.c_iflag &= ~(IXON | IXOFF | IXANY);
    uart_config.c_oflag &= ~OPOST;

    // Set timeout
    uart_config.c_cc[VMIN] = 0;
    uart_config.c_cc[VTIME] = 1; // 100ms timeout

    if (tcsetattr(_serial_fd, TCSANOW, &uart_config) < 0) {
        PX4_ERR("Failed to configure serial port");
        close(_serial_fd);
        _serial_fd = -1;
        return false;
    }

    // Flush buffers
    tcflush(_serial_fd, TCIOFLUSH);

    return true;
}

void ST3125Servo::close_serial()
{
    if (_serial_fd >= 0) {
        close(_serial_fd);
        _serial_fd = -1;
    }
}

uint8_t ST3125Servo::calculate_checksum(const uint8_t *packet, uint8_t length)
{
    uint8_t checksum = 0;
    for (uint8_t i = 2; i < length - 1; i++) {
        checksum += packet[i];
    }
    return ~checksum;
}

int ST3125Servo::send_packet(uint8_t id, uint8_t instruction, const uint8_t *params, uint8_t param_length)
{
    if (_serial_fd < 0) {
        return -1;
    }

    // Build packet
    _tx_buffer[0] = HEADER;
    _tx_buffer[1] = HEADER;
    _tx_buffer[2] = id;
    _tx_buffer[3] = param_length + 2; // Length = params + instruction + checksum
    _tx_buffer[4] = instruction;

    // Copy parameters
    if (params && param_length > 0) {
        memcpy(&_tx_buffer[5], params, param_length);
    }

    // Calculate checksum
    uint8_t packet_length = 5 + param_length + 1;
    _tx_buffer[packet_length - 1] = calculate_checksum(_tx_buffer, packet_length);

    // Send packet
    int written = write(_serial_fd, _tx_buffer, packet_length);
    if (written != packet_length) {
        perf_count(_comms_errors);
        return -1;
    }

    return 0;
}

int ST3125Servo::receive_packet(uint8_t *buffer, uint8_t &id, uint8_t &error, uint8_t &param_length)
{
    if (_serial_fd < 0) {
        return -1;
    }

    // Read header
    uint8_t header_count = 0;
    uint64_t start_time = hrt_absolute_time();

    while (header_count < 2) {
        uint8_t byte;
        int ret = read(_serial_fd, &byte, 1);

        if (ret == 1) {
            if (byte == HEADER) {
                buffer[header_count++] = byte;
            } else {
                header_count = 0;
            }
        }

        // Timeout check
        if (hrt_elapsed_time(&start_time) > 10000) { // 10ms timeout
            return -1;
        }
    }

    // Read ID and length
    int ret = read(_serial_fd, &buffer[2], 2);
    if (ret != 2) {
        perf_count(_comms_errors);
        return -1;
    }

    id = buffer[2];
    uint8_t length = buffer[3];

    if (length < 2 || length > 250) {
        perf_count(_comms_errors);
        return -1;
    }

    // Read remaining data
    ret = read(_serial_fd, &buffer[4], length);
    if (ret != length) {
        perf_count(_comms_errors);
        return -1;
    }

    // Verify checksum
    uint8_t calculated_checksum = calculate_checksum(buffer, 4 + length);
    if (calculated_checksum != buffer[4 + length - 1]) {
        perf_count(_checksum_errors);
        return -1;
    }

    error = buffer[4];
    param_length = length - 2;

    return 0;
}

bool ST3125Servo::ping_servo(uint8_t id)
{
    if (send_packet(id, INST_PING, nullptr, 0) != 0) {
        return false;
    }

    uint8_t rx_id, error, param_length;
    if (receive_packet(_rx_buffer, rx_id, error, param_length) != 0) {
        return false;
    }

    return (rx_id == id && error == 0);
}

bool ST3125Servo::scan_servos()
{
    _num_servos = 0;

    PX4_INFO("Scanning for servos...");

    // Scan ID range 0-253 (254 is broadcast)
    for (uint8_t id = 0; id < 254 && _num_servos < MAX_SERVOS; id++) {
        if (ping_servo(id)) {
            _active_servo_ids[_num_servos] = id;
            _servos[id].id = id;
            _servos[id].connected = true;
            _num_servos++;
            PX4_INFO("Found servo ID %d", id);
        }
    }

    return (_num_servos > 0);
}

bool ST3125Servo::configure_servo(uint8_t id)
{
    if (id >= MAX_SERVOS) {
        return false;
    }

    ServoInfo &servo = _servos[id];

    // Read position limits
    uint16_t min_pos_raw, max_pos_raw;
    if (!read_word(id, ADDR_MIN_POSITION_LIMIT, min_pos_raw) ||
        !read_word(id, ADDR_MAX_POSITION_LIMIT, max_pos_raw)) {
        return false;
    }

    // Convert to radians (assuming 0-4095 = 0-360 degrees)
    servo.min_position = (min_pos_raw / 4095.0f) * 2.0f * M_PI_F;
    servo.max_position = (max_pos_raw / 4095.0f) * 2.0f * M_PI_F;

    // Set position scale and offset
    servo.position_scale = 2.0f * M_PI_F / 4095.0f;
    servo.position_offset = 0.0f;

    return true;
}

bool ST3125Servo::read_byte(uint8_t id, uint8_t address, uint8_t &value)
{
    uint8_t params[2] = {address, 1};
    if (send_packet(id, INST_READ, params, 2) != 0) {
        return false;
    }

    uint8_t rx_id, error, param_length;
    if (receive_packet(_rx_buffer, rx_id, error, param_length) != 0) {
        return false;
    }

    if (rx_id == id && error == 0 && param_length == 1) {
        value = _rx_buffer[5];
        return true;
    }

    return false;
}

bool ST3125Servo::read_word(uint8_t id, uint8_t address, uint16_t &value)
{
    uint8_t params[2] = {address, 2};
    if (send_packet(id, INST_READ, params, 2) != 0) {
        return false;
    }

    uint8_t rx_id, error, param_length;
    if (receive_packet(_rx_buffer, rx_id, error, param_length) != 0) {
        return false;
    }

    if (rx_id == id && error == 0 && param_length == 2) {
        value = _rx_buffer[5] | (_rx_buffer[6] << 8);
        return true;
    }

    return false;
}

bool ST3125Servo::write_byte(uint8_t id, uint8_t address, uint8_t value)
{
    uint8_t params[2] = {address, value};
    return (send_packet(id, INST_WRITE, params, 2) == 0);
}

bool ST3125Servo::write_word(uint8_t id, uint8_t address, uint16_t value)
{
    uint8_t params[3] = {address, (uint8_t)(value & 0xFF), (uint8_t)(value >> 8)};
    return (send_packet(id, INST_WRITE, params, 3) == 0);
}

bool ST3125Servo::set_position(uint8_t id, float position)
{
    if (id >= MAX_SERVOS || !_servos[id].connected) {
        return false;
    }

    // Clamp position
    position = math::constrain(position, _servos[id].min_position, _servos[id].max_position);

    // Convert to raw value
    uint16_t raw_position = (uint16_t)((position / (2.0f * M_PI_F)) * 4095.0f);

    return write_word(id, ADDR_GOAL_POSITION, raw_position);
}

bool ST3125Servo::set_torque_enable(uint8_t id, bool enable)
{
    return write_byte(id, ADDR_TORQUE_ENABLE, enable ? 1 : 0);
}

bool ST3125Servo::set_velocity(uint8_t id, float velocity)
{
    if (id >= MAX_SERVOS || !_servos[id].connected) {
        return false;
    }

    // Convert velocity from rad/s to servo units
    // ST3125 velocity is typically in units of 0.229 RPM
    // 1 rad/s = 9.549 RPM, so velocity_raw = velocity * 9.549 / 0.229
    float velocity_rpm = velocity * 9.549f; // Convert rad/s to RPM
    uint16_t raw_velocity = (uint16_t)math::constrain(velocity_rpm / 0.229f, 0.0f, 1023.0f);

    return write_word(id, ADDR_GOAL_SPEED, raw_velocity);
}

bool ST3125Servo::sync_write_positions(const float *positions)
{
    if (!positions || _num_servos == 0) {
        return false;
    }

    // Build sync write packet
    uint8_t params[4 + MAX_SERVOS * 3]; // Start addr, length, then ID + 2 bytes per servo
    params[0] = ADDR_GOAL_POSITION; // Start address
    params[1] = 2; // Data length per servo

    uint8_t param_index = 2;
    for (uint8_t i = 0; i < _num_servos; i++) {
        uint8_t id = _active_servo_ids[i];

        // Convert position to raw value
        float position = math::constrain(positions[i], _servos[id].min_position, _servos[id].max_position);
        uint16_t raw_position = (uint16_t)((position / (2.0f * M_PI_F)) * 4095.0f);

        params[param_index++] = id;
        params[param_index++] = raw_position & 0xFF;
        params[param_index++] = raw_position >> 8;
    }

    return (send_packet(0xFE, INST_SYNC_WRITE, params, param_index) == 0);
}

bool ST3125Servo::read_position(uint8_t id, float &position)
{
    uint16_t raw_position;
    if (!read_word(id, ADDR_PRESENT_POSITION, raw_position)) {
        return false;
    }

    position = (raw_position / 4095.0f) * 2.0f * M_PI_F;
    _servos[id].last_position_raw = raw_position;
    _servos[id].last_position = position;

    return true;
}

bool ST3125Servo::read_velocity(uint8_t id, float &velocity)
{
    uint16_t raw_speed;
    if (!read_word(id, ADDR_PRESENT_SPEED, raw_speed)) {
        return false;
    }

    // Convert to rad/s (assuming unit is 0.111 RPM)
    float rpm = (raw_speed & 0x3FF) * 0.111f;
    if (raw_speed & 0x400) {
        rpm = -rpm; // Negative direction
    }

    velocity = rpm * 2.0f * M_PI_F / 60.0f;
    _servos[id].last_velocity = velocity;

    return true;
}

bool ST3125Servo::read_current(uint8_t id, float &current)
{
    uint16_t raw_current;
    if (!read_word(id, ADDR_PRESENT_CURRENT, raw_current)) {
        return false;
    }

    // Convert to Amperes (assuming 6.5mA per unit)
    current = raw_current * 0.0065f;

    return true;
}

bool ST3125Servo::read_voltage(uint8_t id, float &voltage)
{
    uint8_t raw_voltage;
    if (!read_byte(id, ADDR_PRESENT_VOLTAGE, raw_voltage)) {
        return false;
    }

    // Convert to Volts (0.1V per unit)
    voltage = raw_voltage * 0.1f;

    return true;
}

bool ST3125Servo::read_temperature(uint8_t id, float &temperature)
{
    uint8_t raw_temp;
    if (!read_byte(id, ADDR_PRESENT_TEMPERATURE, raw_temp)) {
        return false;
    }

    // Direct Celsius reading
    temperature = (float)raw_temp;

    return true;
}

bool ST3125Servo::read_all_feedback()
{
    bool all_success = true;

    for (uint8_t i = 0; i < _num_servos; i++) {
        uint8_t id = _active_servo_ids[i];
        float position, velocity, current, voltage, temperature;

        if (!read_position(id, position) ||
            !read_velocity(id, velocity) ||
            !read_current(id, current) ||
            !read_voltage(id, voltage) ||
            !read_temperature(id, temperature)) {
            _servos[id].error_count++;
            all_success = false;
            continue;
        }

        // Reset error count on successful read
        _servos[id].error_count = 0;
        _servos[id].last_update_time = hrt_absolute_time();
    }

    return all_success;
}

void ST3125Servo::Run()
{
    if (should_exit()) {
        ScheduleClear();
        exit_and_cleanup();
        return;
    }

    perf_begin(_cycle_perf);
    perf_count(_interval_perf);

    // Update parameters
    if (_parameter_update_sub.updated()) {
        parameter_update_s param_update;
        _parameter_update_sub.copy(&param_update);
        updateParams();
    }

    // Handle robotic servo commands
    update_robotic_servo_commands();

    // Read feedback from all servos
    read_all_feedback();

    // Publish feedback
    publish_servo_feedback();

    perf_end(_cycle_perf);
}

void ST3125Servo::publish_servo_feedback()
{
    for (uint8_t i = 0; i < _num_servos; i++) {
        uint8_t id = _active_servo_ids[i];
        ServoInfo &servo = _servos[id];

        if (servo.error_count < 10) { // Only publish if servo is responsive
            servo_feedback_s feedback{};
            feedback.timestamp = hrt_absolute_time();
            feedback.id = id;
            feedback.position = servo.last_position;
            feedback.velocity = servo.last_velocity;
            feedback.torque_enabled = _param_torque_enable.get();

            // Read additional data
            read_current(id, feedback.current);
            read_voltage(id, feedback.voltage);
            read_temperature(id, feedback.temperature);

            // Read load
            uint16_t raw_load;
            if (read_word(id, ADDR_PRESENT_LOAD, raw_load)) {
                feedback.load = (raw_load & 0x3FF) / 1023.0f;
                if (raw_load & 0x400) {
                    feedback.load = -feedback.load;
                }
            }

            _servo_feedback_pub.publish(feedback);
        }
    }
}

int ST3125Servo::print_status()
{
    PX4_INFO("ST3125 Servo Driver Status");
    PX4_INFO("=========================");
    PX4_INFO("Serial port: %s", _serial_port);
    PX4_INFO("Baudrate: %d", (int)_param_baudrate.get());
    PX4_INFO("Update rate: %d Hz",(int) _param_update_rate.get());
    PX4_INFO("Number of servos: %d", _num_servos);

    for (uint8_t i = 0; i < _num_servos; i++) {
        uint8_t id = _active_servo_ids[i];
        ServoInfo &servo = _servos[id];

        PX4_INFO("\nServo ID %d:", id);
        PX4_INFO("  Position: %.3f rad (%.1f deg)",
                 (double)servo.last_position,
                 (double)(servo.last_position * 180.0f / M_PI_F));
        PX4_INFO("  Velocity: %.3f rad/s", (double)servo.last_velocity);
        PX4_INFO("  Error count: %d", servo.error_count);

        if (servo.error_count < 10) {
            float current, voltage, temperature;
            if (read_current(id, current)) {
                PX4_INFO("  Current: %.3f A", (double)current);
            }
            if (read_voltage(id, voltage)) {
                PX4_INFO("  Voltage: %.1f V", (double)voltage);
            }
            if (read_temperature(id, temperature)) {
                PX4_INFO("  Temperature: %.1f C", (double)temperature);
            }
        }
    }

    perf_print_counter(_cycle_perf);
    perf_print_counter(_interval_perf);
    perf_print_counter(_comms_errors);
    perf_print_counter(_checksum_errors);

    return 0;
}

int ST3125Servo::task_spawn(int argc, char *argv[])
{
    ST3125Servo *instance = new ST3125Servo();

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

void ST3125Servo::update_robotic_servo_commands()
{
    robotic_servo_command_s command;

    while (_robotic_servo_command_sub.update(&command)) {
        // Check if this command is for one of our servos
        bool servo_found = false;
        for (uint8_t i = 0; i < _num_servos; i++) {
            if (_active_servo_ids[i] == command.id) {
                servo_found = true;
                break;
            }
        }

        if (!servo_found) {
            continue; // Skip commands for servos not managed by this driver
        }

        // Handle different command types
        switch (command.command_type) {
            case 0: // Position control
                if (command.torque_enable) {
                    set_torque_enable(command.id, true);
                }
                set_position(command.id, command.goal_position);
                break;

            case 1: // Velocity control
                if (command.torque_enable) {
                    set_torque_enable(command.id, true);
                }
                set_velocity(command.id, command.goal_velocity);
                break;

            case 2: // Current control
                // ST3125 doesn't support direct current control,
                // but we can set current limit and use position mode
                if (command.torque_enable) {
                    set_torque_enable(command.id, true);
                }
                // For current control, we might need to implement a wrapper
                // that uses position control with current limiting
                break;

            case 3: // Extended position control
                if (command.torque_enable) {
                    set_torque_enable(command.id, true);
                }
                set_position(command.id, command.goal_position);
                break;

            default:
                PX4_WARN("Unknown command type: %d for servo ID: %d", command.command_type, command.id);
                break;
        }

        // Handle torque enable/disable
        if (!command.torque_enable) {
            set_torque_enable(command.id, false);
        }

        // Handle LED control if supported
        if (command.led_enable) {
            write_byte(command.id, ADDR_LED, 1);
        } else {
            write_byte(command.id, ADDR_LED, 0);
        }
    }
}

int ST3125Servo::custom_command(int argc, char *argv[])
{
    return print_usage("unknown command");
}

int ST3125Servo::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
ST3125 smart servo driver that communicates with ST3125 servos via serial interface.

The driver supports:
- Position control
- Velocity feedback
- Current, voltage, and temperature monitoring
- Multi-servo synchronization
- Error detection and recovery

### Implementation
The driver uses a serial protocol similar to Dynamixel servos to communicate
with ST3125 servos. It publishes servo feedback data and accepts actuator
control commands.

### Examples
Start the driver:
$ st3125_servo start

Start with custom serial port:
$ st3125_servo start -d /dev/ttyS4

Display status:
$ st3125_servo status
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("st3125_servo", "driver");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS3", nullptr, "Serial device", false);
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

extern "C" __EXPORT int st3125_servo_main(int argc, char *argv[])
{
    return ST3125Servo::main(argc, argv);
}
