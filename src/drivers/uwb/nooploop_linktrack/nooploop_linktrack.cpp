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

// =============================================================================
// Constructor/Destructor
// =============================================================================

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

// =============================================================================
// Core Lifecycle Functions
// =============================================================================

int NoopLoopLinkTrack::init()
{
    // Load anchors
    load_anchors("/fs/microsd/uwb_anchors.conf");

    // Schedule at 100Hz (like SR150)
    ScheduleOnInterval(10_ms);

    PX4_INFO("LinkTrack started on %s", _port);
    return PX4_OK;
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
    }

    if (!_param_enable.get()) {
        ScheduleDelayed(1_s);
        return;
    }

    perf_begin(_sample_perf);

    // Read data from UART
    uint8_t data[256];
    ssize_t bytes = read(_fd, data, sizeof(data));

    if (bytes > 0) {
        // Parse each byte through the state machine
        for (ssize_t i = 0; i < bytes; i++) {
            if (parse_char(data[i])) {
                // Complete frame received, process it
                _frames_received++;
            }
        }
    }

    perf_end(_sample_perf);
}

// =============================================================================
// Device Configuration Functions
// =============================================================================

bool NoopLoopLinkTrack::load_anchors(const char *filename)
{
    FILE *file = fopen(filename, "r");
    if (!file) {
        PX4_WARN("Could not open anchor config file: %s", filename);
        return false;
    }

    _num_anchors = 0;
    char line[128];

    while (fgets(line, sizeof(line), file) && _num_anchors < MAX_ANCHORS) {
        if (line[0] == '#') continue;

        int id, forbid;
        float x, y, z;
        char name[32];

        if (sscanf(line, "%d,%31[^,],%f,%f,%f,%d", &id, name, &x, &y, &z, &forbid) >= 5) {
            if (id < MAX_ANCHORS) {
                _anchors[id] = {(uint8_t)id, x, y, z, forbid != 0}; // forbid!=0 means forbidden, forbid==0 means active
                _num_anchors++;
                PX4_INFO("Loaded anchor %d: %s at (%.2f, %.2f, %.2f) %s",
                         id, name, (double)x, (double)y, (double)z,
                         (forbid == 0) ? "ACTIVE" : "FORBIDDEN");
            }
        } else if (sscanf(line, "%d,%31[^,],%f,%f,%f", &id, name, &x, &y, &z) >= 5) {
            // If no forbid flag specified, default to active (forbid=false)
            if (id < MAX_ANCHORS) {
                _anchors[id] = {(uint8_t)id, x, y, z, false}; // Default to active (forbid=false)
                _num_anchors++;
                PX4_INFO("Loaded anchor %d: %s at (%.2f, %.2f, %.2f) ACTIVE (default)",
                         id, name, (double)x, (double)y, (double)z);
            }
        }
    }

    fclose(file);
    PX4_INFO("Loaded %d anchors from %s", _num_anchors, filename);
    return true;
}

// =============================================================================
// Data Parsing Functions
// =============================================================================

bool NoopLoopLinkTrack::parse_char(uint8_t c)
{
    switch (_parser_state) {
    case ParserState::UNINIT:
    case ParserState::IDLE:
        if (c == NLINK_HEADER) {
            _calculated_checksum = 0; // Start fresh checksum calculation
            _parser_state = ParserState::GOT_HEADER;
            _frame_buffer[0] = c;
            _packet_idx = 1;
            _calculated_checksum += c; // Include header in checksum
        } else {
            // Show only occasionally to avoid spam
            static uint32_t idle_count = 0;
            if (++idle_count % 1000 == 0) {
                PX4_DEBUG("IDLE: discarded %lu bytes", (unsigned long)idle_count);
            }
        }
        break;

    case ParserState::GOT_HEADER:
        if (c == NLINK_NODE_FRAME3) {
            _parser_state = ParserState::GOT_FUNCTION;
            _frame_buffer[_packet_idx++] = c;
            _calculated_checksum += c; // Include function mark in checksum
        } else {
            // Invalid function mark, reset
            _parser_state = ParserState::IDLE;
            _parse_errors++;
            perf_count(_comms_errors);
        }
        break;

    case ParserState::GOT_FUNCTION:
        // Frame length low byte (little endian)
        _frame_length = c;
        _frame_buffer[_packet_idx++] = c;
        _calculated_checksum += c;
        _parser_state = ParserState::GOT_LENGTH_LOW;
        break;

    case ParserState::GOT_LENGTH_LOW:
        {
            // Frame length high byte (little endian)
            _frame_length |= ((uint16_t)c << 8);
            _frame_buffer[_packet_idx++] = c;
            _calculated_checksum += c;

            // Validate frame length to prevent buffer overflow
            // According to protocol: frame_length is the total frame size INCLUDING all bytes
            // Minimum frame: header(1) + function(1) + length(2) + role(1) + id(1) + payload(15) + at_least_one_block(7) + checksum(1) = 29 bytes
            if (_frame_length < 29 || _frame_length > RX_BUFFER_SIZE) {
                PX4_WARN("Invalid frame length: %u bytes (min=29, max=%zu)", (unsigned int)_frame_length, RX_BUFFER_SIZE);
                _parser_state = ParserState::IDLE;
                _parse_errors++;
                _buffer_overruns++;
                perf_count(_comms_errors);
                break;
            }

            // Calculate remaining payload size: total_frame - already_received_bytes(4: header+function+length)
            uint16_t remaining_bytes = _frame_length - 4;
            if (remaining_bytes == 0) {
                // Invalid - frame must have at least role+id+payload+checksum
                PX4_WARN("Invalid frame: no payload (total frame=%u)", (unsigned int)_frame_length);
                _parser_state = ParserState::IDLE;
                _parse_errors++;
                break;
            } else {
                _parser_state = ParserState::GOT_PAYLOAD;
            }
        }
        break;

    case ParserState::GOT_PAYLOAD:
        {
            _frame_buffer[_packet_idx++] = c;
            _calculated_checksum += c;

            // Check if we've received all bytes except the final checksum
            // Total frame includes: header(1) + function(1) + length(2) + payload + checksum(1)
            // So payload ends when we have received: _frame_length - 1 bytes (all except final checksum)
            if (_packet_idx >= (size_t)(_frame_length - 1)) {
                _parser_state = ParserState::GOT_CHECKSUM;
            }

            // Prevent buffer overflow
            if (_packet_idx >= RX_BUFFER_SIZE) {
                PX4_ERR("Buffer overflow during payload parsing at byte %zu", _packet_idx);
                _parser_state = ParserState::IDLE;
                _parse_errors++;
                _buffer_overruns++;
                perf_count(_comms_errors);
            }
        }
        break;

    case ParserState::GOT_CHECKSUM:
        _frame_buffer[_packet_idx++] = c;

        // Verify checksum
        if (c == _calculated_checksum) {
            // Frame complete and valid - verify total frame size matches expected
            if (_packet_idx == _frame_length) {
                _parser_state = ParserState::IDLE;
                // Process the complete frame
                parse_frame(_frame_buffer, _packet_idx);
                return true; // Frame successfully received
            } else {
                _parser_state = ParserState::IDLE;
                _parse_errors++;
                perf_count(_comms_errors);
            }
        } else {
            _parser_state = ParserState::IDLE;
            _parse_errors++;
            perf_count(_comms_errors);
        }
        break;
    }

    return false; // Frame not yet complete;
}

bool NoopLoopLinkTrack::parse_frame(const uint8_t *data, size_t length)
{
    // Validate minimum frame size: header(6) + payload(15) + at least 1 anchor block(7) + checksum(1) = 29 bytes minimum
    if (length < 29) {
        PX4_DEBUG("Frame too short: %zu bytes (minimum 29)", length);
        return false;
    }

    // Parse header (first 6 bytes)
    const NodeFrame3Header *header = (const NodeFrame3Header *)data;

    // Validate frame header
    if (header->frame_header != NLINK_HEADER || header->function_mark != NLINK_NODE_FRAME3) {
        PX4_WARN("Invalid frame header: 0x%02X 0x%02X", header->frame_header, header->function_mark);
        return false;
    }

    // Check if this is from our tag (we can process any tag, but filter if needed)
    uint8_t node_id = header->id;
    if (header->role == NLINK_ROLE_TAG && node_id != (uint8_t)_param_tag_id.get()) {
        return false; // Silently ignore frames from other tags
    }

    // Only process frames from tags (role 0x02) that contain anchor measurements
    if (header->role != NLINK_ROLE_TAG) {
        return false; // Silently ignore non-tag frames
    }

    // Parse payload (starts at byte 6)
    if (length < sizeof(NodeFrame3Header) + sizeof(NodeFrame3Payload)) {
        PX4_DEBUG("Frame too short for payload: %zu bytes", length);
        return false;
    }

    const NodeFrame3Payload *payload = (const NodeFrame3Payload *)(data + sizeof(NodeFrame3Header));
    uint8_t valid_quantity = payload->valid_quantity;

    if (valid_quantity > MAX_RANGES_PER_FRAME) {
        PX4_WARN("Too many ranges: %d, limiting to %d", valid_quantity, MAX_RANGES_PER_FRAME);
        valid_quantity = MAX_RANGES_PER_FRAME;
    }

    // Calculate expected total frame size
    size_t expected_size = sizeof(NodeFrame3Header) + sizeof(NodeFrame3Payload) +
                          (valid_quantity * sizeof(AnchorData)) + 1; // +1 for checksum

    if (length < expected_size) {
        PX4_DEBUG("Frame incomplete: got %zu, expected %zu", length, expected_size);
        return false;
    }

    // Parse anchor data blocks (start after header + payload)
    const AnchorData *anchor_data = (const AnchorData *)(data + sizeof(NodeFrame3Header) + sizeof(NodeFrame3Payload));

    for (int i = 0; i < valid_quantity; i++) {
        const AnchorData &anchor = anchor_data[i];

        // Verify this is anchor data (role should be 0x01)
        if (anchor.role == NLINK_ROLE_ANCHOR) {
            // Decode 3-byte distance field (int24, little endian) from mm to meters
            // According to protocol: unit is m * 1000 (so distance is in mm)
            uint32_t distance_mm = anchor.distance[0] |
                                  (anchor.distance[1] << 8) |
                                  (anchor.distance[2] << 16);

            float distance_m = distance_mm / 1000.0f; // Convert mm to meters

            // Convert RSSI according to protocol: stored_value / (-2) = actual_dBm
            float fp_rssi_dbm = anchor.fp_rssi / (-2.0f);  // First path RSSI
            float rx_rssi_dbm = anchor.rx_rssi / (-2.0f);  // Receive RSSI

            // Validate range (reasonable distance and RSSI values)
            if (distance_m > 0.1f && distance_m < 100.0f && rx_rssi_dbm > -200.0f && rx_rssi_dbm < 0.0f) {
                publish_range(anchor.id, distance_m, rx_rssi_dbm, fp_rssi_dbm);
            }
        } else {
            // Skip non-anchor data silently
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
            float rx_rssi = r.rx_rssi / (-2.0f); // Decode RSSI: stored_value / (-2) = actual_dBm
            float fp_rssi = r.fp_rssi / (-2.0f); // Decode first path RSSI
            publish_range(r.id, distance, rx_rssi, fp_rssi);
        }
    }
}

// =============================================================================
// Publishing Functions
// =============================================================================

void NoopLoopLinkTrack::publish_range(uint8_t anchor_id, float distance, float rssi, float fp_rssi)
{
    if (anchor_id >= MAX_ANCHORS) {
        PX4_WARN("Anchor ID %d exceeds MAX_ANCHORS (%d)", anchor_id, MAX_ANCHORS);
        return;
    }

    if (_anchors[anchor_id].forbid) {
        return; // Silently ignore forbidden anchors
    }

    sensor_uwb_s msg = {};
    msg.timestamp = hrt_absolute_time();
    msg.sessionid = 0;
    msg.time_offset = 0;
    msg.anchor_id = anchor_id;
    msg.tag_id = (uint8_t)_param_tag_id.get();
    msg.range = distance;
    msg.rssi = (int8_t)rssi;
    msg.los_confidence = 50; // Default value
    msg.first_path_power = (int8_t)fp_rssi;  // Use first path RSSI
    msg.total_path_power = (int8_t)rssi;     // Use receive RSSI as total power
    msg.anchor_x = _anchors[anchor_id].x;
    msg.anchor_y = _anchors[anchor_id].y;
    msg.anchor_z = _anchors[anchor_id].z;
    msg.anchor_pos_valid = !_anchors[anchor_id].forbid;
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

// =============================================================================
// Static Functions (Task Management, Commands, Help)
// =============================================================================

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
            PX4_INFO("Frames received: %lu", (unsigned long)instance->_frames_received);
            PX4_INFO("Parse errors: %lu", (unsigned long)instance->_parse_errors);
            PX4_INFO("Buffer overruns: %lu", (unsigned long)instance->_buffer_overruns);
            PX4_INFO("Parser state: %d", (int)instance->_parser_state);
            PX4_INFO("Parameters: TAG_ID=%d, ENABLE=%d, BAUD=%d",
                     (int)instance->_param_tag_id.get(),
                     (int)instance->_param_enable.get(),
                     (int)instance->_param_baud_rate.get());
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
