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

#include <px4_platform_common/getopt.h>
#include <px4_platform_common/log.h>
#include <lib/mathlib/mathlib.h>

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
        PX4_INFO("=== NoopLoop LinkTrack Status ===");
        PX4_INFO("Port: %s", instance->_port);
        PX4_INFO("Anchor file: %s", instance->_anchor_file);
        PX4_INFO("Total measurements: %lu", instance->_total_measurements);
        PX4_INFO("Filtered measurements: %lu (%.1f%%)",
                 instance->_filtered_measurements,
                 (double)(instance->_filtered_measurements * 100.0f / math::max(1u, (unsigned int)instance->_total_measurements)));
        PX4_INFO("Loaded anchors: %d", instance->_num_anchors);

        // Print anchor info
        for (uint8_t i = 0; i < instance->_num_anchors; i++) {
            const AnchorInfo &anchor = instance->_anchors[i];
            PX4_INFO("  Anchor %d (%s): [%.2f, %.2f, %.2f]",
                     anchor.id, anchor.name,
                     (double)anchor.x, (double)anchor.y, (double)anchor.z);
        }

        // Print performance counters
        perf_print_counter(instance->_sample_perf);
        perf_print_counter(instance->_comms_errors);
        perf_print_counter(instance->_buffer_overflows);
        perf_print_counter(instance->_range_batch_perf);

        return 0;
    }

    if (!strcmp(argv[0], "reload")) {
        if (instance->load_anchor_positions(instance->_anchor_file)) {
            PX4_INFO("Anchor positions reloaded successfully");
            return 0;
        } else {
            PX4_ERR("Failed to reload anchor positions");
            return 1;
        }
    }

    if (!strcmp(argv[0], "reset")) {
        perf_reset(instance->_sample_perf);
        perf_reset(instance->_comms_errors);
        perf_reset(instance->_buffer_overflows);
        perf_reset(instance->_range_batch_perf);
        instance->_total_measurements = 0;
        instance->_filtered_measurements = 0;
        PX4_INFO("Statistics reset");
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
    PRINT_MODULE_USAGE_COMMAND_DESCR("status", "Print driver status");
    PRINT_MODULE_USAGE_COMMAND_DESCR("reload", "Reload anchor positions from file");
    PRINT_MODULE_USAGE_COMMAND_DESCR("reset", "Reset performance counters");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

extern "C" __EXPORT int nooploop_linktrack_main(int argc, char *argv[])
{
    return NoopLoopLinkTrack::main(argc, argv);
}
