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

#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <lib/parameters/param.h>

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>

#define MODULE_NAME "nooploop_linktrack"

NoopLoopLinkTrack::NoopLoopLinkTrack(const char *port) :
    ModuleParams(nullptr),
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
    strncpy(_port, port, sizeof(_port) - 1);
    _port[sizeof(_port) - 1] = '\0';

    // Default anchor file path
    strncpy(_anchor_file, "/fs/microsd/uwb_anchors.conf", sizeof(_anchor_file) - 1);
    _anchor_file[sizeof(_anchor_file) - 1] = '\0';

    _sample_perf = perf_alloc(PC_ELAPSED, MODULE_NAME"_sample");
    _comms_errors = perf_alloc(PC_COUNT, MODULE_NAME"_comms_errors");
    _buffer_overflows = perf_alloc(PC_COUNT, MODULE_NAME"_buffer_overflows");
    _range_batch_perf = perf_alloc(PC_ELAPSED, MODULE_NAME"_range_batch");
}

NoopLoopLinkTrack::~NoopLoopLinkTrack()
{
    ScheduleClear();

    if (_fd >= 0)
    {
        close(_fd);
    }

    perf_free(_sample_perf);
    perf_free(_comms_errors);
    perf_free(_buffer_overflows);
    perf_free(_range_batch_perf);
}

bool NoopLoopLinkTrack::load_anchor_positions(const char *filename)
{
    FILE *file = fopen(filename, "r");

    if (file == nullptr) {
        PX4_WARN("Failed to open anchor file: %s", filename);
        return false;
    }

    _num_anchors = 0;
    char line[256];
    int line_num = 0;

    while (fgets(line, sizeof(line), file) != nullptr) {
        line_num++;

        // Remove newline
        char *newline = strchr(line, '\n');
        if (newline) {
            *newline = '\0';
        }

        // Skip comments and empty lines
        if (line[0] == '\0' || line[0] == '#') {
            continue;
        }

        AnchorInfo anchor;

        // Format: ID,Name,X,Y,Z
        char *token = strtok(line, ",");
        if (token == nullptr) {
            PX4_WARN("Invalid format at line %d", line_num);
            continue;
        }
        anchor.id = (uint8_t)atoi(token);

        // Parse Name
        token = strtok(nullptr, ",");
        if (token == nullptr) {
            PX4_WARN("Failed to read name at line %d", line_num);
            continue;
        }
        strncpy(anchor.name, token, sizeof(anchor.name) - 1);
        anchor.name[sizeof(anchor.name) - 1] = '\0';

        // Parse X
        token = strtok(nullptr, ",");
        if (token == nullptr) {
            PX4_WARN("Failed to read X at line %d", line_num);
            continue;
        }
        anchor.x = atof(token);

        // Parse Y
        token = strtok(nullptr, ",");
        if (token == nullptr) {
            PX4_WARN("Failed to read Y at line %d", line_num);
            continue;
        }
        anchor.y = atof(token);

        // Parse Z
        token = strtok(nullptr, ",");
        if (token == nullptr) {
            PX4_WARN("Failed to read Z at line %d", line_num);
            continue;
        }
        anchor.z = atof(token);

        anchor.valid = true;

        // Store anchor if we have space and ID is valid
        if (_num_anchors < MAX_ANCHORS && anchor.id < MAX_ANCHORS) {
            _anchors[_num_anchors] = anchor;
            _num_anchors++;
        }

        PX4_INFO("Loaded anchor %d (%s): [%.2f, %.2f, %.2f]",
                 anchor.id, anchor.name,
                 (double)anchor.x, (double)anchor.y, (double)anchor.z);
    }

    fclose(file);

    PX4_INFO("Loaded %d anchors from %s", _num_anchors, filename);

    // Save to parameters for EKF2
    save_anchor_positions_to_params();

    return _num_anchors > 0;
}

void NoopLoopLinkTrack::save_anchor_positions_to_params()
{
    // Save anchor positions to EKF2 parameters
    for (uint8_t i = 0; i < _num_anchors; i++)
    {
        const AnchorInfo &anchor = _anchors[i];

        if (anchor.id >= MAX_ANCHORS)
        {
            continue;
        }

        char param_name[32];

        // Set X coordinate
        snprintf(param_name, sizeof(param_name), "EKF2_UWB_A%d_X", anchor.id);
        param_t param_x = param_find(param_name);
        if (param_x != PARAM_INVALID) {
            param_set(param_x, &anchor.x);
        }

        // Set Y coordinate
        snprintf(param_name, sizeof(param_name), "EKF2_UWB_A%d_Y", anchor.id);
        param_t param_y = param_find(param_name);
        if (param_y != PARAM_INVALID) {
            param_set(param_y, &anchor.y);
        }

        // Set Z coordinate
        snprintf(param_name, sizeof(param_name), "EKF2_UWB_A%d_Z", anchor.id);
        param_t param_z = param_find(param_name);
        if (param_z != PARAM_INVALID) {
            param_set(param_z, &anchor.z);
        }
    }
}

int NoopLoopLinkTrack::init()
{
    // Load anchor positions
    if (!load_anchor_positions(_anchor_file)) {
        PX4_WARN("No anchors loaded, continuing without anchor info");
    }

    // Open serial port
    _fd = open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);

    if (_fd < 0) {
        PX4_ERR("Failed to open port %s", _port);
        return -1;
    }

    // Configure serial port
    struct termios uart_config;
    if (tcgetattr(_fd, &uart_config) < 0) {
        PX4_ERR("Failed to get port attributes");
        return -1;
    }

    // Set baudrate
    int baudrate = _param_baud_rate.get();
    speed_t speed;

	switch (baudrate) {
	case 115200:
		speed = B115200;
		break;
	case 230400:
		speed = B230400;
		break;
	case 460800:
		speed = B460800;
		break;
	case 921600:
		speed = B921600;
		break;
	default:
		PX4_ERR("Unsupported baudrate: %d", baudrate);
		return -1;
	}

    cfsetispeed(&uart_config, speed);
    cfsetospeed(&uart_config, speed);

    // 8N1
    uart_config.c_cflag &= ~(CSTOPB | PARENB | CSIZE);
    uart_config.c_cflag |= CS8;

    // Raw mode
    uart_config.c_lflag &= ~(ECHO | ECHONL | ICANON | IEXTEN | ISIG);
    uart_config.c_iflag &= ~(BRKINT | ICRNL | IGNBRK | IGNCR | INLCR | INPCK | ISTRIP | IXON | IXOFF | IXANY);
    uart_config.c_oflag &= ~(OPOST | ONLCR | OCRNL);

    if (tcsetattr(_fd, TCSANOW, &uart_config) < 0) {
        PX4_ERR("Failed to set port attributes");
        return -1;
    }

    // Configure LinkTrack device
    if (!configure_device()) {
        PX4_ERR("Failed to configure LinkTrack device");
        return -1;
    }

    ScheduleOnInterval(5_ms); // 200Hz to handle burst data

    return OK;
}

bool NoopLoopLinkTrack::configure_device()
{
    // Configure for multi-range output with diagnostics
    uint8_t config_cmd[] = {
        HEADER,
        0x05,  // Length
        0x10,  // Config command
        0x00,  // Disable position output
        0x01,  // Enable range output
        0x01,  // Enable batch mode
        0x00,  // Checksum (calculated)
        FRAME_END
    };

    config_cmd[6] = calculate_checksum(config_cmd + 1, 5);

	if (write(_fd, config_cmd, sizeof(config_cmd)) != sizeof(config_cmd))
	{
		return false;
	}

    usleep(100000); // 100ms delay

    return true;
}

void NoopLoopLinkTrack::Run()
{
	// Check if UWB is enabled
	if (!_param_enable.get())
	{
		// Schedule for later check (1Hz)
		ScheduleDelayed(1_s);
		return;
	}

	if (_fd < 0)
	{
		// Try to reinitialize
		if (init() != OK)
		{
			ScheduleDelayed(1_s);
		}
		return;
	}

    perf_begin(_sample_perf);

    // Read available data
    uint8_t read_buffer[512];
    ssize_t bytes_read = read(_fd, read_buffer, sizeof(read_buffer));

	if (bytes_read > 0)
	{
		// Process each byte through the parser
		for (ssize_t i = 0; i < bytes_read; i++)
		{
			uint8_t byte = read_buffer[i];

			switch (_parser_state) {
			case ParserState::WAIT_HEADER:
				if (byte == HEADER)
				{
					_rx_buffer[0] = byte;
					_rx_buffer_pos = 1;
					_parser_state = ParserState::WAIT_LENGTH_LOW;
				}
				break;

            case ParserState::WAIT_LENGTH_LOW:
                _rx_buffer[_rx_buffer_pos++] = byte;
                _frame_length = byte;
                _parser_state = ParserState::WAIT_LENGTH_HIGH;
                break;

			case ParserState::WAIT_LENGTH_HIGH:
				_rx_buffer[_rx_buffer_pos++] = byte;
				_frame_length |= (byte << 8);

				// Sanity check frame length
				if (_frame_length > 1000)
				{
					perf_count(_comms_errors);
					_parser_state = ParserState::WAIT_HEADER;
					_rx_buffer_pos = 0;
					break;
				}

				_parser_state = ParserState::WAIT_TYPE;
				break;

            case ParserState::WAIT_TYPE:
                _rx_buffer[_rx_buffer_pos++] = byte;
                _frame_type = byte;
                _parser_state = ParserState::WAIT_DATA;
                break;

			case ParserState::WAIT_DATA:
				_rx_buffer[_rx_buffer_pos++] = byte;

				if (_rx_buffer_pos >= static_cast<size_t>(_frame_length + 3U))
				{
					_parser_state = ParserState::WAIT_CHECKSUM;
				}
                break;

            case ParserState::WAIT_CHECKSUM:
                _rx_buffer[_rx_buffer_pos++] = byte;
                _parser_state = ParserState::WAIT_END;
                break;

            case ParserState::WAIT_END:
                if (byte == FRAME_END) {
                    // Verify checksum
                    uint8_t calc_checksum = calculate_checksum(_rx_buffer + 1, _frame_length + 2);
                    uint8_t recv_checksum = _rx_buffer[_frame_length + 3];

                    if (calc_checksum == recv_checksum) {
                        parse_frame(_rx_buffer, _rx_buffer_pos);
                    } else {
                        perf_count(_comms_errors);
                        PX4_DEBUG("Checksum mismatch: calc=0x%02x, recv=0x%02x", calc_checksum, recv_checksum);
                    }
                }

                _parser_state = ParserState::WAIT_HEADER;
                _rx_buffer_pos = 0;
                break;
            }

            if (_rx_buffer_pos >= sizeof(_rx_buffer)) {
                perf_count(_buffer_overflows);
                PX4_DEBUG("Buffer overflow, resetting parser");
                _parser_state = ParserState::WAIT_HEADER;
                _rx_buffer_pos = 0;
            }
        }
    } else if (bytes_read < 0 && errno != EAGAIN && errno != EWOULDBLOCK) {
        // Error reading from port
        perf_count(_comms_errors);
        PX4_DEBUG("Read error: %s", strerror(errno));
    }

    perf_end(_sample_perf);
}

bool NoopLoopLinkTrack::parse_frame(uint8_t *buffer, size_t len)
{
    FrameType type = static_cast<FrameType>(_frame_type);

    switch (type) {
    case FrameType::MULTI_RANGE_WITH_DIAGNOSTICS:
        process_multi_range(buffer + 4);
        break;

    case FrameType::RANGE_BATCH:
        process_multi_range(buffer + 4);
        break;

    default:
        return false;
    }

    return true;
}

void NoopLoopLinkTrack::process_multi_range(const uint8_t *data)
{
    perf_begin(_range_batch_perf);

    MultiRangePacket *packet = (MultiRangePacket *)data;

    // Check if we should publish all ranges or only for our tag
    bool publish_all = _param_publish_all_ranges.get();
    uint8_t our_tag_id = _param_tag_id.get();

    if (!publish_all && packet->tag_id != our_tag_id) {
        perf_end(_range_batch_perf);
        return;
    }

    // Set timestamp if not provided
    if (packet->timestamp_us == 0) {
        packet->timestamp_us = hrt_absolute_time();
    }

    publish_uwb_batch(*packet);

    perf_end(_range_batch_perf);
}

float NoopLoopLinkTrack::estimate_range_bias(int8_t rssi, uint8_t los_confidence, uint8_t multipath_count)
{
    // Estimate range bias based on signal quality
    float bias = 0.0f;

    // RSSI-based bias estimation
    if (rssi < -85) {
        bias += 0.2f; // 20cm bias for weak signals
    } else if (rssi < -75) {
        bias += 0.1f; // 10cm bias for moderate signals
    }

    // LOS confidence bias
    if (los_confidence < 50) {
        bias += 0.3f; // 30cm bias for likely NLOS
    } else if (los_confidence < 80) {
        bias += 0.1f; // 10cm bias for uncertain LOS
    }

    // Multipath bias
    bias += multipath_count * 0.05f; // 5cm per detected multipath

    return bias;
}

void NoopLoopLinkTrack::publish_uwb_batch(const MultiRangePacket &packet)
{
    const float min_rssi = _param_min_rssi.get();

    // Validate packet
    if (packet.num_measurements == 0 || packet.num_measurements > MAX_MEASUREMENTS_PER_MSG) {
        PX4_DEBUG("Invalid measurement count: %d", packet.num_measurements);
        return;
    }

    for (uint8_t i = 0; i < packet.num_measurements; i++) {
        const RangeData &range = packet.measurements[i];

        _total_measurements++;

        // Filter by signal strength
        if (range.rssi < min_rssi) {
            _filtered_measurements++;
            continue;
        }

        // Validate range (0.1m to 100m)
        float range_m = range.distance_mm / 1000.0f;
        if (range_m < 0.1f || range_m > 100.0f) {
            _filtered_measurements++;
            PX4_DEBUG("Range out of bounds: %.2f m", (double)range_m);
            continue;
        }

        sensor_uwb_s uwb_msg{};
        uwb_msg.timestamp = packet.timestamp_us;
        uwb_msg.sessionid = 0;
        uwb_msg.time_offset = i * 1000; // 1ms between measurements
        uwb_msg.anchor_id = range.anchor_id;
        uwb_msg.tag_id = range.tag_id;

        // Range in meters
        uwb_msg.range = range_m;

        // Signal quality metrics
        uwb_msg.rssi = range.rssi;
        uwb_msg.los_confidence = range.los_confidence;
        uwb_msg.first_path_power = range.first_path_amp;
        uwb_msg.total_path_power = range.rx_power;
        uwb_msg.multipath_count = range.multipath_count;

        // Estimate range bias based on signal quality
        uwb_msg.range_bias = estimate_range_bias(range.rssi, range.los_confidence, range.multipath_count);

        // Add anchor position if available
        AnchorInfo *anchor = find_anchor(range.anchor_id);
        if (anchor != nullptr) {
            uwb_msg.anchor_x = anchor->x;
            uwb_msg.anchor_y = anchor->y;
            uwb_msg.anchor_z = anchor->z;
            uwb_msg.anchor_pos_valid = true;
        } else {
            uwb_msg.anchor_pos_valid = false;
            PX4_DEBUG("No position data for anchor %d", range.anchor_id);
        }

        // Add sensor offset
        uwb_msg.offset_x = _param_offset_x.get();
        uwb_msg.offset_y = _param_offset_y.get();
        uwb_msg.offset_z = _param_offset_z.get();

        _sensor_uwb_pub.publish(uwb_msg);

        PX4_DEBUG("UWB: tag=%d, anchor=%d, range=%.2f m, rssi=%d dBm",
                 range.tag_id, range.anchor_id, (double)range_m, range.rssi);
    }
}

uint8_t NoopLoopLinkTrack::calculate_checksum(const uint8_t *data, size_t len)
{
    uint8_t checksum = 0;
    for (size_t i = 0; i < len; i++) {
        checksum += data[i];
    }
    return checksum;
}

NoopLoopLinkTrack::AnchorInfo *NoopLoopLinkTrack::find_anchor(uint8_t id)
{
    for (uint8_t i = 0; i < _num_anchors; i++) {
        if (_anchors[i].id == id && _anchors[i].valid) {
            return &_anchors[i];
        }
    }
    return nullptr;
}
