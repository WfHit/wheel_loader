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

#include "nooploop_linktrack.hpp"

#include <errno.h>
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#include <cstdio>
#include <cstring>

#include <drivers/drv_hrt.h>
#include <lib/parameters/param.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>

#define MODULE_NAME "nooploop_linktrack"

NoopLoopLinkTrack::NoopLoopLinkTrack(const char *port) :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::serial_port_to_wq(port))
{
    strncpy(_port, port, sizeof(_port) - 1);
    _port[sizeof(_port) - 1] = '\0';

    _sample_perf = perf_alloc(PC_ELAPSED, MODULE_NAME);
    _comms_errors = perf_alloc(PC_COUNT, MODULE_NAME": comm_err");
    memset(_anchors, 0, sizeof(_anchors));
}

NoopLoopLinkTrack::~NoopLoopLinkTrack()
{
    ScheduleClear();
    if (_fd >= 0) {
        close(_fd);
        _fd = -1;
    }
    perf_free(_sample_perf);
    perf_free(_comms_errors);
}

int NoopLoopLinkTrack::init()
{
    // Load anchors
    load_anchors("/fs/microsd/uwb_anchors.conf");

    // Schedule at 100Hz (like SR150)
    ScheduleOnInterval(10_ms);

    PX4_INFO("LinkTrack started on %s", _port);
    return PX4_OK;
}

bool NoopLoopLinkTrack::configure_device()
{
    if (_fd < 0) {
        PX4_ERR("Cannot configure device: serial port not open");
        return false;
    }

    // Configure LP_MODE2
    uint8_t cmd[] = {NLINK_HEADER, 0x08, 0x00, 0x10, 0x02, (uint8_t)_param_tag_id.get(), 0x01, 0x00, 0x00, NLINK_FRAME_END};
    uint8_t sum = 0;
    for (int i = 1; i < 8; i++) sum += cmd[i];
    cmd[8] = sum;

    ssize_t written = write(_fd, cmd, sizeof(cmd));
    if (written != sizeof(cmd)) {
        PX4_ERR("Write failed: %zd/%zu bytes (errno: %d)", written, sizeof(cmd), errno);
        perf_count(_comms_errors);
        return false;
    }
    usleep(100000);

    // Enable Node_Frame3
    uint8_t out[] = {NLINK_HEADER, 0x06, 0x00, 0x13, NLINK_NODE_FRAME3, 0x01, 0x00, NLINK_FRAME_END};
    sum = 0;
    for (int i = 1; i < 6; i++) sum += out[i];
    out[6] = sum;

    written = write(_fd, out, sizeof(out));
    if (written != sizeof(out)) {
        PX4_ERR("Write failed: %zd/%zu bytes (errno: %d)", written, sizeof(out), errno);
        perf_count(_comms_errors);
        return false;
    }
    usleep(100000);

    PX4_INFO("Device configuration completed successfully");
    return true;
}

void NoopLoopLinkTrack::Run()
{
    if (should_exit()) {
        ScheduleClear();
        if (_fd >= 0) {
            close(_fd);
            _fd = -1;
        }
        return;
    }

    // Open and configure UART if not already open (similar to UWB SR150)
    if (_fd < 0) {
        /* open fd */
        _fd = ::open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);

        if (_fd < 0) {
            PX4_ERR("open failed (%i): %s", errno, strerror(errno));
            perf_count(_comms_errors);
            return;
        }

        struct termios uart_config;
        int termios_state;

        /* fill the struct for the new configuration */
        if (tcgetattr(_fd, &uart_config) < 0) {
            PX4_ERR("tcgetattr failed: %s", strerror(errno));
            close(_fd);
            _fd = -1;
            return;
        }

        /* clear ONLCR flag (which appends a CR for every LF) */
        uart_config.c_oflag &= ~ONLCR;

        /* no parity, one stop bit, 8 data bits */
        uart_config.c_cflag &= ~(CSTOPB | PARENB);
        uart_config.c_cflag |= CS8;

        /* disable flow control */
        uart_config.c_cflag &= ~CRTSCTS;

        /* set raw input/output mode */
        uart_config.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
        uart_config.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | ICRNL);

        /* set baud rate based on parameter */
        speed_t speed = B921600; // default
        int32_t baud_param = _param_baud_rate.get();

        switch (baud_param) {
            case 9600: speed = B9600; break;
            case 19200: speed = B19200; break;
            case 38400: speed = B38400; break;
            case 57600: speed = B57600; break;
            case 115200: speed = B115200; break;
            case 230400: speed = B230400; break;
            case 460800: speed = B460800; break;
            case 921600: speed = B921600; break;
            default:
                PX4_WARN("Unsupported baud rate %ld, using 921600", (long)baud_param);
                speed = B921600;
                break;
        }

        /* set baud rate */
        if ((termios_state = cfsetispeed(&uart_config, speed)) < 0) {
            PX4_ERR("CFG: %d ISPD", termios_state);
            close(_fd);
            _fd = -1;
            return;
        }

        if ((termios_state = cfsetospeed(&uart_config, speed)) < 0) {
            PX4_ERR("CFG: %d OSPD", termios_state);
            close(_fd);
            _fd = -1;
            return;
        }

        if ((termios_state = tcsetattr(_fd, TCSANOW, &uart_config)) < 0) {
            PX4_ERR("baud %d ATTR: %s", termios_state, strerror(errno));
            close(_fd);
            _fd = -1;
            return;
        }

        // Flush any existing data
        tcflush(_fd, TCIOFLUSH);

        PX4_INFO("Serial port %s configured at %ld baud", _port, (long)baud_param);

        // Setup device configuration
        if (!configure_device()) {
            PX4_ERR("Device configuration failed");
            close(_fd);
            _fd = -1;
            return;
        }
    }

    if (!_param_enable.get()) {
        ScheduleDelayed(1_s);
        return;
    }

    perf_begin(_sample_perf);

    uint8_t data[128];
    ssize_t bytes = read(_fd, data, sizeof(data));

    if (bytes > 0) {
        for (ssize_t i = 0; i < bytes; i++) {
            _rx_buffer[_rx_buffer_pos++] = data[i];

            if (_rx_buffer_pos >= sizeof(_rx_buffer)) _rx_buffer_pos = 0;

            // Look for complete frame
            if (_rx_buffer_pos >= 25 && _rx_buffer[_rx_buffer_pos-1] == NLINK_FRAME_END) {
                for (size_t j = 0; j < _rx_buffer_pos - 4; j++) {
                    if (_rx_buffer[j] == NLINK_HEADER && _rx_buffer[j+3] == NLINK_NODE_FRAME3) {
                        parse_frame(_rx_buffer + j, _rx_buffer_pos - j);
                        break;
                    }
                }
                _rx_buffer_pos = 0;
            }
        }
    }

    perf_end(_sample_perf);
}


void NoopLoopLinkTrack::publish_range(uint8_t anchor_id, float distance, float rssi)
{
    if (anchor_id >= MAX_ANCHORS || !_anchors[anchor_id].active) return;

    sensor_uwb_s msg = {};
    msg.timestamp = hrt_absolute_time();
    msg.sessionid = 0;
    msg.time_offset = 0;
    msg.anchor_id = anchor_id;
    msg.tag_id = (uint8_t)_param_tag_id.get();
    msg.range = distance;
    msg.rssi = (int8_t)rssi;
    msg.los_confidence = 50; // Default value
    msg.first_path_power = 0;
    msg.total_path_power = 0;
    msg.anchor_x = _anchors[anchor_id].x;
    msg.anchor_y = _anchors[anchor_id].y;
    msg.anchor_z = _anchors[anchor_id].z;
    msg.anchor_pos_valid = _anchors[anchor_id].active;
    msg.multipath_count = 0;
    msg.range_bias = 0.0f;
    msg.aoa_azimuth_fom = 0;
    msg.aoa_elevation_fom = 0;
    msg.aoa_dest_azimuth_fom = 0;
    msg.aoa_dest_elevation_fom = 0;
    msg.orientation = 0;
    msg.offset_x = 0.0f;
    msg.offset_y = 0.0f;
    msg.offset_z = 0.0f;

    _sensor_uwb_pub.publish(msg);
}

int NoopLoopLinkTrack::task_spawn(int argc, char *argv[])
{
    const char *port = "/dev/ttyS2";
    int ch;
    const char *myoptarg = nullptr;
    int myoptind = 1;

    while ((ch = px4_getopt(argc, argv, "d:", &myoptind, &myoptarg)) != EOF) {
        if (ch == 'd') port = myoptarg;
    }

    NoopLoopLinkTrack *instance = new NoopLoopLinkTrack(port);
    if (!instance) return PX4_ERROR;

    _object.store(instance);
    _task_id = task_id_is_work_queue;

    if (instance->init() != PX4_OK) {
        delete instance;
        _object.store(nullptr);
        return PX4_ERROR;
    }
    return PX4_OK;
}

int NoopLoopLinkTrack::custom_command(int argc, char *argv[])
{
    if (argc > 0 && strcmp(argv[0], "stop") == 0) {
        // Handle stop command even if not running
        return PX4_OK;
    }

    if (argc > 0 && strcmp(argv[0], "debug") == 0) {
        NoopLoopLinkTrack *instance = get_instance();
        if (instance) {
            instance->debug_uart();
        }
        return PX4_OK;
    }

    if (!is_running()) {
        PX4_ERR("not running");
        return PX4_ERROR;
    }

    if (argc > 0 && strcmp(argv[0], "status") == 0) {
        NoopLoopLinkTrack *instance = get_instance();
        if (instance) {
            PX4_INFO("Port: %s, Anchors: %d", instance->_port, instance->_num_anchors);
            PX4_INFO("Performance:");
            perf_print_counter(instance->_sample_perf);
            perf_print_counter(instance->_comms_errors);
        }
    }
    return PX4_OK;
}

int NoopLoopLinkTrack::print_usage(const char *reason)
{
    if (reason) PX4_WARN("%s", reason);

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
NoopLoop LinkTrack UWB driver for positioning using Ultra-Wideband ranging.

### Usage
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("nooploop_linktrack", "driver");
    PRINT_MODULE_USAGE_COMMAND_DESCR("start", "Start the driver");
    PRINT_MODULE_USAGE_PARAM_STRING('d', "/dev/ttyS6", "<device>", "Serial device", true);
    PRINT_MODULE_USAGE_COMMAND_DESCR("stop", "Stop the driver");
    PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Show driver status");
    PRINT_MODULE_USAGE_COMMAND_DESCR("debug", "Debug UART communication (send/receive raw data)");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return PX4_OK;
}

bool NoopLoopLinkTrack::parse_frame(const uint8_t *data, size_t length)
{
    if (length < 25) return false;

    uint8_t tag_id = data[4];
    if (tag_id != (uint8_t)_param_tag_id.get()) return false;

    uint8_t num_ranges = data[23];
    if (num_ranges > MAX_RANGES_PER_FRAME) return false;

    const RangeData *ranges = (const RangeData *)(data + 24);

    for (int i = 0; i < num_ranges; i++) {
        const RangeData &r = ranges[i];

        if (r.distance_mm > 0 && r.distance_mm < 100000 && r.rssi > -100) {
            float distance = r.distance_mm / 1000.0f;
            publish_range(r.anchor_id, distance, r.rssi);
        }
    }

    return true;
}

void NoopLoopLinkTrack::process_ranges(uint8_t tag_id, uint8_t num_ranges, const RangeData *ranges)
{
    for (int i = 0; i < num_ranges; i++) {
        const RangeData &r = ranges[i];
        if (r.distance_mm > 0 && r.distance_mm < 100000 && r.rssi > -100) {
            float distance = r.distance_mm / 1000.0f;
            publish_range(r.anchor_id, distance, r.rssi);
        }
    }
}

bool NoopLoopLinkTrack::load_anchors(const char *filename)
{
    FILE *file = fopen(filename, "r");
    if (!file) return false;

    _num_anchors = 0;
    char line[128];

    while (fgets(line, sizeof(line), file) && _num_anchors < MAX_ANCHORS) {
        if (line[0] == '#') continue;

        int id, active;
        float x, y, z;
        char name[32];

        if (sscanf(line, "%d,%31[^,],%f,%f,%f,%d", &id, name, &x, &y, &z, &active) >= 5) {
            if (id < MAX_ANCHORS) {
                _anchors[id] = {(uint8_t)id, x, y, z, active != 0};
                _num_anchors++;
            }
        }
    }

    fclose(file);
    return true;
}

uint8_t NoopLoopLinkTrack::calculate_checksum(const uint8_t *data, size_t length)
{
    uint8_t sum = 0;
    for (size_t i = 0; i < length; i++) {
        sum += data[i];
    }
    return sum;
}

void NoopLoopLinkTrack::debug_uart()
{
    PX4_INFO("Starting UART debug mode on %s (Press Ctrl+C to exit)", _port);
    PX4_INFO("Will show raw TX/RX data with timestamps");

    // Open UART if not already open
    int debug_fd = _fd;
    bool should_close = false;

    if (debug_fd < 0) {
        debug_fd = ::open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (debug_fd < 0) {
            PX4_ERR("Failed to open %s for debug: %d", _port, errno);
            return;
        }
        should_close = true;

        // Configure UART
        struct termios uart_config;
        tcgetattr(debug_fd, &uart_config);
        uart_config.c_oflag &= ~ONLCR;
        uart_config.c_cflag &= ~(CSTOPB | PARENB);
        cfsetispeed(&uart_config, B921600);
        cfsetospeed(&uart_config, B921600);
        tcsetattr(debug_fd, TCSANOW, &uart_config);
    }

    // Send a test command to see TX data
    PX4_INFO("=== Sending test configuration command ===");
    uint8_t test_cmd[] = {NLINK_HEADER, 0x08, 0x00, 0x10, 0x02, 0x00, 0x01, 0x00, 0x00, NLINK_FRAME_END};
    uint8_t sum = 0;
    for (int i = 1; i < 8; i++) sum += test_cmd[i];
    test_cmd[8] = sum;

    printf("TX (%zu bytes): ", sizeof(test_cmd));
    for (size_t i = 0; i < sizeof(test_cmd); i++) {
        printf("%02X ", test_cmd[i]);
    }
    printf("\n");

    ssize_t written = write(debug_fd, test_cmd, sizeof(test_cmd));
    if (written != sizeof(test_cmd)) {
        PX4_WARN("Write failed: %zd/%zu bytes (errno: %d - %s)", written, sizeof(test_cmd), errno, strerror(errno));
    } else {
        PX4_INFO("Test command sent successfully");
    }

    PX4_INFO("=== Listening for RX data (10 seconds) ===");

    uint8_t rx_buffer[256];
    uint64_t start_time = hrt_absolute_time();
    uint64_t timeout = 10000000; // 10 seconds in microseconds

    while ((hrt_absolute_time() - start_time) < timeout) {
        ssize_t bytes = read(debug_fd, rx_buffer, sizeof(rx_buffer));

        if (bytes > 0) {
            uint64_t timestamp = hrt_absolute_time();
            printf("RX (%zd bytes) [%llu us]: ", bytes, timestamp);

            for (ssize_t i = 0; i < bytes; i++) {
                printf("%02X ", rx_buffer[i]);

                // Print ASCII if printable
                if (i == bytes - 1) {
                    printf(" | ");
                    for (ssize_t j = 0; j < bytes; j++) {
                        char c = rx_buffer[j];
                        printf("%c", (c >= 32 && c <= 126) ? c : '.');
                    }
                }
            }
            printf("\n");

            // Try to decode known frame types
            decode_debug_frame(rx_buffer, bytes);
        }

        usleep(1000); // 1ms delay to prevent busy waiting
    }

    PX4_INFO("=== Debug session completed ===");

    if (should_close && debug_fd >= 0) {
        close(debug_fd);
    }
}

void NoopLoopLinkTrack::decode_debug_frame(const uint8_t *data, size_t length)
{
    if (length < 4) return;

    // Look for LinkTrack header
    for (size_t i = 0; i <= length - 4; i++) {
        if (data[i] == NLINK_HEADER && (i + 3) < length) {
            uint8_t frame_type = data[i + 3];
            printf("  -> Frame detected: Header=0x%02X, Type=0x%02X", data[i], frame_type);

            switch (frame_type) {
                case NLINK_NODE_FRAME3:
                    printf(" (NODE_FRAME3 - Position Data)");
                    if (length >= i + 25) {
                        uint8_t tag_id = data[i + 4];
                        uint8_t num_ranges = (i + 23 < length) ? data[i + 23] : 0;
                        printf(" Tag_ID=%d, Ranges=%d", tag_id, num_ranges);
                    }
                    break;
                case 0x10:
                    printf(" (Configuration Response)");
                    break;
                case 0x13:
                    printf(" (Frame Enable Response)");
                    break;
                default:
                    printf(" (Unknown Type)");
                    break;
            }
            printf("\n");
        }
    }
}


