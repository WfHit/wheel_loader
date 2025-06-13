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

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <drivers/device/device.h>
#include <drivers/drv_hrt.h>
#include <lib/parameters/param.h>
#include <lib/perf/perf_counter.h>

#include <uORB/Publication.hpp>
#include <uORB/topics/sensor_uwb.h>

#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <cstring>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <vector>
#include <map>

using namespace time_literals;

class NoopLoopLinkTrack : public ModuleBase<NoopLoopLinkTrack>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
    NoopLoopLinkTrack(const char *port);
    ~NoopLoopLinkTrack() override;

    /** @see ModuleBase */
    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    int init();

private:
    // LinkTrack protocol definitions
    static constexpr uint8_t HEADER = 0x55;
    static constexpr uint8_t FRAME_END = 0x77;
    static constexpr int MAX_ANCHORS = 16;
    static constexpr int MAX_MEASUREMENTS_PER_MSG = 8;

    enum class FrameType : uint8_t {
        POSITION_2D = 0x00,
        POSITION_3D = 0x01,
        RANGE_BATCH = 0x02,
        ANCHOR_POSITION = 0x03,
        TAG_POSITION = 0x04,
        SYSTEM_STATUS = 0x05,
        MULTI_RANGE_WITH_DIAGNOSTICS = 0x06
    };

    struct AnchorInfo {
        uint8_t id;
        float x;
        float y;
        float z;
        bool valid;
        char name[32];
    };

    struct RangeData {
        uint8_t anchor_id;
        uint8_t tag_id;
        uint32_t distance_mm;
        int8_t rssi;
        uint8_t los_confidence;
        uint16_t first_path_amp;
        uint16_t rx_power;
        uint8_t multipath_count;
    } __attribute__((packed));

    struct MultiRangePacket {
        uint8_t tag_id;
        uint8_t num_measurements;
        uint64_t timestamp_us;
        RangeData measurements[MAX_MEASUREMENTS_PER_MSG];
    } __attribute__((packed));

    void Run() override;
    bool parse_frame(uint8_t *buffer, size_t len);
    void process_multi_range(const uint8_t *data);
    void publish_uwb_batch(const MultiRangePacket &packet);
    bool configure_device();
    uint8_t calculate_checksum(const uint8_t *data, size_t len);
    bool load_anchor_positions(const char *filename);
    void save_anchor_positions_to_params();
    float estimate_range_bias(int8_t rssi, uint8_t los_confidence, uint8_t multipath_count);

    // Serial port
    char _port[32];
    int _fd{-1};

    // Anchor configuration
    std::map<uint8_t, AnchorInfo> _anchors;
    char _anchor_file[256];

    // Parser state
    enum class ParserState {
        WAIT_HEADER,
        WAIT_LENGTH_LOW,
        WAIT_LENGTH_HIGH,
        WAIT_TYPE,
        WAIT_DATA,
        WAIT_CHECKSUM,
        WAIT_END
    };

    ParserState _parser_state{ParserState::WAIT_HEADER};
    uint8_t _rx_buffer[1024];
    size_t _rx_buffer_pos{0};
    uint16_t _frame_length{0};
    uint8_t _frame_type{0};

    // Publications
    uORB::Publication<sensor_uwb_s> _sensor_uwb_pub{ORB_ID(sensor_uwb)};

    // Performance counters
    perf_counter_t _sample_perf;
    perf_counter_t _comms_errors;
    perf_counter_t _buffer_overflows;
    perf_counter_t _range_batch_perf;

    // Statistics
    uint32_t _total_measurements{0};
    uint32_t _filtered_measurements{0};

    // Parameters
    DEFINE_PARAMETERS(
        (ParamInt<px4::params::SENS_UWB_BAUD>) _param_baud_rate,
        (ParamInt<px4::params::SENS_UWB_TAG_ID>) _param_tag_id,
        (ParamFloat<px4::params::SENS_UWB_MIN_RSSI>) _param_min_rssi,
        (ParamInt<px4::params::SENS_UWB_PUB_ALL>) _param_publish_all_ranges,
        (ParamFloat<px4::params::SENS_UWB_OFFSET_X>) _param_offset_x,
        (ParamFloat<px4::params::SENS_UWB_OFFSET_Y>) _param_offset_y,
        (ParamFloat<px4::params::SENS_UWB_OFFSET_Z>) _param_offset_z
    )
};

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

    if (_fd >= 0) {
        close(_fd);
    }

    perf_free(_sample_perf);
    perf_free(_comms_errors);
    perf_free(_buffer_overflows);
    perf_free(_range_batch_perf);
}

bool NoopLoopLinkTrack::load_anchor_positions(const char *filename)
{
    std::ifstream file(filename);
    if (!file.is_open()) {
        PX4_WARN("Failed to open anchor file: %s", filename);
        return false;
    }

    _anchors.clear();
    std::string line;
    int line_num = 0;

    while (std::getline(file, line)) {
        line_num++;

        // Skip comments and empty lines
        if (line.empty() || line[0] == '#') {
            continue;
        }

        std::istringstream iss(line);
        AnchorInfo anchor;
        std::string token;

        // Format: ID,Name,X,Y,Z
        // Parse ID
        if (!std::getline(iss, token, ',')) {
            PX4_WARN("Invalid format at line %d", line_num);
            continue;
        }
        anchor.id = static_cast<uint8_t>(std::stoi(token));

        // Parse Name
        if (!std::getline(iss, token, ',')) {
            PX4_WARN("Failed to read name at line %d", line_num);
            continue;
        }
        strncpy(anchor.name, token.c_str(), sizeof(anchor.name) - 1);
        anchor.name[sizeof(anchor.name) - 1] = '\0';

        // Parse X
        if (!std::getline(iss, token, ',')) {
            PX4_WARN("Failed to read X at line %d", line_num);
            continue;
        }
        anchor.x = std::stof(token);

        // Parse Y
        if (!std::getline(iss, token, ',')) {
            PX4_WARN("Failed to read Y at line %d", line_num);
            continue;
        }
        anchor.y = std::stof(token);

        // Parse Z
        if (!std::getline(iss, token)) {
            PX4_WARN("Failed to read Z at line %d", line_num);
            continue;
        }
        anchor.z = std::stof(token);

        anchor.valid = true;
        _anchors[anchor.id] = anchor;

        PX4_INFO("Loaded anchor %d (%s): [%.2f, %.2f, %.2f]",
                 anchor.id, anchor.name,
                 (double)anchor.x, (double)anchor.y, (double)anchor.z);
    }

    file.close();

    PX4_INFO("Loaded %zu anchors from %s", _anchors.size(), filename);

    // Save to parameters for EKF2
    save_anchor_positions_to_params();

    return !_anchors.empty();
}

void NoopLoopLinkTrack::save_anchor_positions_to_params()
{
    // Save anchor positions to EKF2 parameters
    for (const auto &pair : _anchors) {
        const AnchorInfo &anchor = pair.second;

        if (anchor.id >= MAX_ANCHORS) {
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
        param_t param_z = param_find(param_z);
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
    case 115200: speed = B115200; break;
    case 230400: speed = B230400; break;
    case 460800: speed = B460800; break;
    case 921600: speed = B921600; break;
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

    if (write(_fd, config_cmd, sizeof(config_cmd)) != sizeof(config_cmd)) {
        return false;
    }

    usleep(100000); // 100ms delay

    return true;
}

void NoopLoopLinkTrack::Run()
{
    if (_fd < 0) {
        return;
    }

    perf_begin(_sample_perf);

    // Read available data
    uint8_t read_buffer[512];
    ssize_t bytes_read = read(_fd, read_buffer, sizeof(read_buffer));

    if (bytes_read > 0) {
        // Process each byte through the parser
        for (ssize_t i = 0; i < bytes_read; i++) {
            uint8_t byte = read_buffer[i];

            switch (_parser_state) {
            case ParserState::WAIT_HEADER:
                if (byte == HEADER) {
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
                _parser_state = ParserState::WAIT_TYPE;
                break;

            case ParserState::WAIT_TYPE:
                _rx_buffer[_rx_buffer_pos++] = byte;
                _frame_type = byte;
                _parser_state = ParserState::WAIT_DATA;
                break;

            case ParserState::WAIT_DATA:
                _rx_buffer[_rx_buffer_pos++] = byte;

                if (_rx_buffer_pos >= _frame_length + 3) {
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
                    }
                }

                _parser_state = ParserState::WAIT_HEADER;
                break;
            }

            if (_rx_buffer_pos >= sizeof(_rx_buffer)) {
                perf_count(_buffer_overflows);
                _parser_state = ParserState::WAIT_HEADER;
                _rx_buffer_pos = 0;
            }
        }
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

    for (int i = 0; i < packet.num_measurements && i < MAX_MEASUREMENTS_PER_MSG; i++) {
        const RangeData &range = packet.measurements[i];

        _total_measurements++;

        // Filter by signal strength
        if (range.rssi < min_rssi) {
            _filtered_measurements++;
            continue;
        }

        sensor_uwb_s uwb_msg{};
        uwb_msg.timestamp = packet.timestamp_us;
        uwb_msg.sessionid = 0;
        uwb_msg.time_offset = i * 1000;
        uwb_msg.anchor_id = range.anchor_id;
        uwb_msg.tag_id = range.tag_id;

        // Range in meters
        uwb_msg.range = range.distance_mm / 1000.0f;

        // Signal quality metrics
        uwb_msg.rssi = range.rssi;
        uwb_msg.los_confidence = range.los_confidence;
        uwb_msg.first_path_power = range.first_path_amp;
        uwb_msg.total_path_power = range.rx_power;
        uwb_msg.multipath_count = range.multipath_count;

        // Estimate range bias
        uwb_msg.range_bias = estimate_range_bias(range.rssi, range.los_confidence, range.multipath_count);

        // Add anchor position if available
        auto it = _anchors.find(range.anchor_id);
        if (it != _anchors.end() && it->second.valid) {
            uwb_msg.anchor_x = it->second.x;
            uwb_msg.anchor_y = it->second.y;
            uwb_msg.anchor_z = it->second.z;
            uwb_msg.anchor_pos_valid = true;
        } else {
            uwb_msg.anchor_pos_valid = false;
        }

        // Add sensor offset
        uwb_msg.offset_x = _param_offset_x.get();
        uwb_msg.offset_y = _param_offset_y.get();
        uwb_msg.offset_z = _param_offset_z.get();

        _sensor_uwb_pub.publish(uwb_msg);
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

int NoopLoopLinkTrack::task_spawn(int argc, char *argv[])
{
    const char *port = nullptr;
    const char *anchor_file = nullptr;

    int ch;
    int myoptind = 1;
    const char *myoptarg = nullptr;

    while ((ch = px4_getopt(argc, argv, "d:a:", &myoptind, &myoptarg)) != EOF) {
        switch (ch) {
        case 'd':
            port = myoptarg;
            break;
        case 'a':
            anchor_file = myoptarg;
            break;
        default:
            print_usage("unrecognized flag");
            return -1;
        }
    }

    if (!port) {
        print_usage("port required");
        return -1;
    }

    NoopLoopLinkTrack *instance = new NoopLoopLinkTrack(port);

    if (!instance) {
        PX4_ERR("alloc failed");
        return -1;
    }

    // Override anchor file if specified
    if (anchor_file) {
        strncpy(instance->_anchor_file, anchor_file, sizeof(instance->_anchor_file) - 1);
        instance->_anchor_file[sizeof(instance->_anchor_file) - 1] = '\0';
    }

    _object.store(instance);
    _task_id = task_id_is_work_queue;

    if (instance->init() != OK) {
        delete instance;
        _object.store(nullptr);
        _task_id = -1;
        return -1;
    }

    return OK;
}

int NoopLoopLinkTrack::custom_command(int argc, char *argv[])
{
    if (!is_running()) {
        print_usage("driver not running");
        return 1;
    }

    NoopLoopLinkTrack *instance = get_instance();

    if (!strcmp(argv[0], "status")) {
        PX4_INFO("Total measurements: %u", instance->_total_measurements);
        PX4_INFO("Filtered measurements: %u", instance->_filtered_measurements);
        PX4_INFO("Loaded anchors: %zu", instance->_anchors.size());
        return 0;
    }

    return print_usage("unknown command");
}

int NoopLoopLinkTrack::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Driver for Nooploop LinkTrack UWB positioning system.

### Implementation
The driver communicates with the LinkTrack device via UART and publishes range
data to the PX4 system. It supports batch processing of multiple range measurements
and reads anchor positions from a configuration file.

### Examples
Start the driver on UART port with custom anchor file:
$ nooploop_linktrack start -d /dev/ttyS1 -a /fs/microsd/my_anchors.conf

Anchor file format (CSV):
# ID,Name,X,Y,Z
0,Anchor_A,0.0,0.0,2.5
1,Anchor_B,10.0,0.0,2.5
2,Anchor_C,10.0,10.0,2.5
3,Anchor_D,0.0,10.0,2.5
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("nooploop_linktrack", "driver");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_PARAM_STRING('d', nullptr, "<device>", "UART device", false);
    PRINT_MODULE_USAGE_PARAM_STRING('a', "/fs/microsd/uwb_anchors.conf", "<file>", "Anchor position file", true);
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

extern "C" __EXPORT int nooploop_linktrack_main(int argc, char *argv[])
{
    return NoopLoopLinkTrack::main(argc, argv);
}
