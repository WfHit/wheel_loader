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
    ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
    strncpy(_port, port, sizeof(_port) - 1);
    _port[sizeof(_port) - 1] = '\0';

    _perf = perf_alloc(PC_ELAPSED, MODULE_NAME);
    memset(_anchors, 0, sizeof(_anchors));
}

NoopLoopLinkTrack::~NoopLoopLinkTrack()
{
    ScheduleClear();
    if (_fd >= 0) close(_fd);
    perf_free(_perf);
}

int NoopLoopLinkTrack::init()
{
    // Load anchors
    load_anchors();

    // Open serial port
    _fd = open(_port, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (_fd < 0) {
        PX4_ERR("Failed to open %s", _port);
        return PX4_ERROR;
    }

    // Configure port (921600, 8N1)
    struct termios config;
    tcgetattr(_fd, &config);
    cfsetispeed(&config, B921600);
    cfsetospeed(&config, B921600);
    config.c_cflag = CS8 | CREAD | CLOCAL;
    config.c_iflag = config.c_lflag = config.c_oflag = 0;
    tcsetattr(_fd, TCSANOW, &config);

    // Setup device
    setup_device();

    // Schedule at 100Hz
    ScheduleOnInterval(10_ms);

    PX4_INFO("LinkTrack started on %s", _port);
    return PX4_OK;
}

bool NoopLoopLinkTrack::setup_device()
{
    // Configure LP_MODE2
    uint8_t cmd[] = {HEADER, 0x08, 0x00, 0x10, 0x02, _param_tag_id.get(), 0x01, 0x00, 0x00, FRAME_END};
    uint8_t sum = 0;
    for (int i = 1; i < 8; i++) sum += cmd[i];
    cmd[8] = sum;
    write(_fd, cmd, sizeof(cmd));
    usleep(100000);

    // Enable Node_Frame3
    uint8_t out[] = {HEADER, 0x06, 0x00, 0x13, NODE_FRAME3, 0x01, 0x00, FRAME_END};
    sum = 0;
    for (int i = 1; i < 6; i++) sum += out[i];
    out[6] = sum;
    write(_fd, out, sizeof(out));
    usleep(100000);

    return true;
}

void NoopLoopLinkTrack::load_anchors()
{
    FILE *file = fopen("/fs/microsd/uwb_anchors.conf", "r");
    if (!file) return;

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
    PX4_INFO("Loaded %d anchors", _num_anchors);
}

void NoopLoopLinkTrack::Run()
{
    if (!_param_enable.get()) {
        ScheduleDelayed(1_s);
        return;
    }

    perf_begin(_perf);

    uint8_t data[128];
    ssize_t bytes = read(_fd, data, sizeof(data));

    if (bytes > 0) {
        for (ssize_t i = 0; i < bytes; i++) {
            _buffer[_pos++] = data[i];

            if (_pos >= sizeof(_buffer)) _pos = 0;

            // Look for complete frame
            if (_pos >= 25 && _buffer[_pos-1] == FRAME_END) {
                for (size_t j = 0; j < _pos - 4; j++) {
                    if (_buffer[j] == HEADER && _buffer[j+3] == NODE_FRAME3) {
                        parse_data(_buffer + j, _pos - j);
                        break;
                    }
                }
                _pos = 0;
            }
        }
    }

    perf_end(_perf);
}

bool NoopLoopLinkTrack::parse_data(const uint8_t *data, size_t len)
{
    if (len < 25) return false;

    uint8_t tag_id = data[4];
    if (tag_id != _param_tag_id.get()) return false;

    uint8_t num_ranges = data[23];
    if (num_ranges > 10) return false;

    const Range *ranges = (const Range *)(data + 24);

    for (int i = 0; i < num_ranges; i++) {
        const Range &r = ranges[i];

        if (r.distance_mm > 0 && r.distance_mm < 100000 && r.rssi > -100) {
            float distance = r.distance_mm / 1000.0f;
            publish_uwb(r.anchor_id, distance);
        }
    }

    return true;
}

void NoopLoopLinkTrack::publish_uwb(uint8_t anchor_id, float distance)
{
    if (anchor_id >= MAX_ANCHORS || !_anchors[anchor_id].active) return;

    sensor_uwb_s msg = {};
    msg.timestamp = hrt_absolute_time();
    msg.distance = distance;
    msg.accuracy = 0.1f;
    msg.anchor_id = anchor_id;
    msg.anchor_x = _anchors[anchor_id].x;
    msg.anchor_y = _anchors[anchor_id].y;
    msg.anchor_z = _anchors[anchor_id].z;

    _uwb_pub.publish(msg);
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
    if (!is_running()) return PX4_ERROR;

    if (argc > 0 && strcmp(argv[0], "status") == 0) {
        NoopLoopLinkTrack *instance = get_instance();
        if (instance) {
            PX4_INFO("Port: %s, Anchors: %d", instance->_port, instance->_num_anchors);
        }
    }
    return PX4_OK;
}

int NoopLoopLinkTrack::print_usage(const char *reason)
{
    if (reason) PX4_WARN("%s", reason);
    PX4_INFO("Usage: nooploop_linktrack {start|status|stop} [-d device]");
    return PX4_OK;
}


