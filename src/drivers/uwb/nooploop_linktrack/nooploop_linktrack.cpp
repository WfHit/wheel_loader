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

    // Configure LP_MODE2 - Set Tag Mode with ID
    uint8_t cmd[] = {NLINK_HEADER, 0x08, 0x10, 0x02, (uint8_t)_param_tag_id.get(), 0x01, 0x00};
    uint8_t sum = 0;
    for (int i = 1; i < 6; i++) sum += cmd[i];
    cmd[6] = sum; // Checksum

    ssize_t written = write(_fd, cmd, sizeof(cmd));
    if (written != sizeof(cmd)) {
        PX4_ERR("Write failed: %zd/%zu bytes (errno: %d)", written, sizeof(cmd), errno);
        perf_count(_comms_errors);
        return false;
    }
    usleep(100000);

    // Enable Node_Frame3 output
    uint8_t out[] = {NLINK_HEADER, 0x06, 0x13, NLINK_NODE_FRAME3, 0x01};
    sum = 0;
    for (int i = 1; i < 4; i++) sum += out[i];
    out[4] = sum; // Checksum

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

            if (_rx_buffer_pos >= sizeof(_rx_buffer)) {
                _rx_buffer_pos = 0; // Buffer overflow protection
            }

            // Look for complete Node_Frame3 packets
            if (_rx_buffer_pos >= 4) { // Minimum to read header + function + length(2 bytes)
                // Scan for frame start
                for (size_t j = 0; j <= _rx_buffer_pos - 4; j++) {
                    if (_rx_buffer[j] == NLINK_HEADER && _rx_buffer[j+1] == NLINK_NODE_FRAME3) {
                        // Frame length is 2 bytes, little endian
                        uint16_t frame_length = _rx_buffer[j+2] | (_rx_buffer[j+3] << 8);
                        size_t total_frame_size = 4 + frame_length; // header + function + length(2) + payload

                        // Check if we have received the complete frame
                        if (j + total_frame_size <= _rx_buffer_pos) {
                            // Verify checksum
                            uint8_t calculated_checksum = 0;
                            for (size_t k = j + 1; k < j + total_frame_size - 1; k++) {
                                calculated_checksum += _rx_buffer[k];
                            }
                            uint8_t received_checksum = _rx_buffer[j + total_frame_size - 1];

                            if (calculated_checksum == received_checksum) {
                                parse_frame(_rx_buffer + j, total_frame_size);
                            } else {
                                PX4_WARN("Checksum mismatch: calc=0x%02X, recv=0x%02X",
                                        calculated_checksum, received_checksum);
                                perf_count(_comms_errors);
                            }

                            // Remove processed frame from buffer
                            size_t remaining = _rx_buffer_pos - (j + total_frame_size);
                            if (remaining > 0) {
                                memmove(_rx_buffer, _rx_buffer + j + total_frame_size, remaining);
                            }
                            _rx_buffer_pos = remaining;
                            break;
                        }
                    }
                }
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
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return PX4_OK;
}

bool NoopLoopLinkTrack::parse_frame(const uint8_t *data, size_t length)
{
    if (length < sizeof(NodeFrame3Header)) {
        PX4_DEBUG("Frame too short: %zu bytes", length);
        return false;
    }

    const NodeFrame3Header *header = (const NodeFrame3Header *)data;

    // Verify frame header and function mark
    if (header->frame_header != NLINK_HEADER || header->function_mark != NLINK_NODE_FRAME3) {
        PX4_DEBUG("Invalid frame header or function mark");
        return false;
    }

    // Check if this is from our tag (we can process any tag, but filter if needed)
    uint8_t node_id = header->id;
    if (header->role == NLINK_ROLE_TAG && node_id != (uint8_t)_param_tag_id.get()) {
        // This frame is from a different tag, ignore it
        return false;
    }

    // Only process frames from tags (role 0x02) that contain anchor measurements
    if (header->role != NLINK_ROLE_TAG) {
        return false;
    }

    uint8_t valid_quantity = header->valid_quantity;
    if (valid_quantity > MAX_RANGES_PER_FRAME) {
        PX4_WARN("Too many ranges: %d", valid_quantity);
        return false;
    }

    // Calculate expected frame size
    size_t expected_size = sizeof(NodeFrame3Header) +
                          (valid_quantity * sizeof(AnchorData)) +
                          1; // checksum

    if (length < expected_size) {
        PX4_DEBUG("Frame incomplete: got %zu, expected %zu", length, expected_size);
        return false;
    }

    // Parse anchor data
    const AnchorData *anchor_data = (const AnchorData *)(data + sizeof(NodeFrame3Header));

    for (int i = 0; i < valid_quantity; i++) {
        const AnchorData &anchor = anchor_data[i];

        // Verify this is anchor data (role should be 0x01)
        if (anchor.role == NLINK_ROLE_ANCHOR) {
            // Decode 3-byte distance field (int24, little endian) from mm to meters
            uint32_t distance_mm = anchor.distance[0] |
                                  (anchor.distance[1] << 8) |
                                  (anchor.distance[2] << 16);

            float distance_m = distance_mm / 1000.0f;

            // Use rx_rssi as received power level (negative dBm)
            float rssi = -(float)anchor.rx_rssi;

            // Validate range
            if (distance_m > 0.1f && distance_m < 100.0f && rssi > -120.0f) {
                publish_range(anchor.id, distance_m, rssi);
            }
        }
    }

    return true;
}

void NoopLoopLinkTrack::process_ranges(uint8_t tag_id, uint8_t num_ranges, const AnchorData *ranges)
{
    for (int i = 0; i < num_ranges; i++) {
        const AnchorData &r = ranges[i];
        uint32_t distance_mm = r.distance[0] | (r.distance[1] << 8) | (r.distance[2] << 16);
        if (r.role == NLINK_ROLE_ANCHOR && distance_mm > 100 && distance_mm < 100000) {
            float distance = distance_mm / 1000.0f; // Convert mm to meters
            float rssi = -(float)r.rx_rssi; // Use rx_rssi as in parse_frame
            publish_range(r.id, distance, rssi);
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


