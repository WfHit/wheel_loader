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
 * @file bucket_control_main.cpp.cpp
 * Bucket control module system interface and command handling.
 */

#include "bucket_control.hpp"
#include <px4_platform_common/log.h>
#include <lib/mathlib/mathlib.h>

// Module interface implementation
int BucketControl::task_spawn(int argc, char *argv[])
{
    BucketControl *instance = new BucketControl();

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

int BucketControl::custom_command(int argc, char *argv[])
{
    if (!is_running()) {
        PX4_ERR("Module not running");
        return PX4_ERROR;
    }

    BucketControl *instance = get_instance();
    if (!instance) {
        PX4_ERR("Instance not available");
        return PX4_ERROR;
    }

    if (argc < 2) {
        return print_usage("missing command");
    }

    if (!strcmp(argv[1], "test")) {
        if (argc < 3) {
            PX4_INFO("Available test commands:");
            PX4_INFO("  angle <deg>    - Set target bucket angle in degrees");
            PX4_INFO("  status         - Show current status");
            return PX4_OK;
        }

        if (!strcmp(argv[2], "angle")) {
            if (argc < 4) {
                PX4_ERR("Usage: bucket_control test angle <degrees>");
                return PX4_ERROR;
            }

            float angle_deg = atof(argv[3]);
            float angle_rad = math::radians(angle_deg);

            // Validate angle range (typical bucket range: -90 to +90 degrees)
            if (angle_deg < -90.0f || angle_deg > 90.0f) {
                PX4_WARN("Angle %f° is outside typical range [-90, +90]", static_cast<double>(angle_deg));
            }

            // Create and publish bucket command to set target angle
            bucket_command_s cmd{};
            cmd.timestamp = hrt_absolute_time();
            cmd.target_angle = angle_rad; // Set the target angle
            cmd.control_mode = 0; // Fixed to boom compensation mode
            cmd.command_mode = 0; // Position mode
            cmd.coordinate_frame = 0; // Ground reference
            cmd.max_velocity = instance->_param_max_velocity.get() / 1000.0f; // Convert mm/s to m/s for angle rate
            cmd.enable_stability_limit = false; // No AHRS integration
            cmd.enable_anti_spill = false; // Anti-spill not used in boom compensation mode
            cmd.grading_angle = nanf(""); // Use parameter defaults
            cmd.transport_angle = nanf(""); // Use parameter defaults
            cmd.stability_threshold = nanf(""); // Use parameter defaults

            // Publish the command
            static uORB::Publication<bucket_command_s> test_bucket_cmd_pub{ORB_ID(bucket_command)};
            test_bucket_cmd_pub.publish(cmd);

            PX4_INFO("Bucket command published: target angle %.1f° (%.3f rad)", static_cast<double>(angle_deg), static_cast<double>(angle_rad));
            return PX4_OK;
        }

        if (!strcmp(argv[2], "status")) {
            const char* state_names[] = {"UNINITIALIZED", "ZEROING", "READY", "MOVING", "ERROR"};

            PX4_INFO("=== Bucket Control Status ===");
            PX4_INFO("State: %s", state_names[static_cast<int>(instance->_state)]);
            PX4_INFO("Control Mode: Boom Compensation (fixed)");
            PX4_INFO("Target Bucket Angle: %.1f°", static_cast<double>(math::degrees(instance->_target_absolute_bucket_angle)));
            PX4_INFO("Current Bucket Angle: %.1f°", static_cast<double>(math::degrees(instance->_current_bucket_angle)));
            PX4_INFO("Current Boom Angle: %.1f°", static_cast<double>(math::degrees(instance->_current_boom_angle)));
            PX4_INFO("Actuator Length: %.1f mm (target: %.1f mm)",
                     static_cast<double>(instance->_current_actuator_length),
                     static_cast<double>(instance->_target_actuator_length));
            PX4_INFO("Limit Switches - Load: %s, Dump: %s",
                     instance->_limit_switch_load ? "ACTIVE" : "inactive",
                     instance->_limit_switch_dump ? "ACTIVE" : "inactive");
            PX4_INFO("Zeroing Complete: %s", instance->_zeroing_complete ? "YES" : "NO");
            PX4_INFO("Boom Angle Changed: %s", instance->_boom_angle_changed ? "YES" : "NO");

            return PX4_OK;
        }

        return print_usage("unknown test command");
    }

    return print_usage("unknown command");
}

int BucketControl::print_usage(const char *reason)
{
    if (reason) {
        PX4_WARN("%s\n", reason);
    }

    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Bucket control module for wheel loader.

Manages bucket angle control through linear actuator with boom angle compensation.
Supports zeroing procedure with load limit (bucket down) and dump limit (bucket up) switches.

)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("bucket_control", "controller");
    PRINT_MODULE_USAGE_COMMAND("start");
    PRINT_MODULE_USAGE_COMMAND_DESCR("test angle <deg>", "Set target bucket angle in degrees");
    PRINT_MODULE_USAGE_COMMAND_DESCR("test status", "Show current module status");
    PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

    return 0;
}

extern "C" __EXPORT int bucket_control_main(int argc, char *argv[])
{
    return BucketControl::main(argc, argv);
}
