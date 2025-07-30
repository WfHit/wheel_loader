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
 * @file quad_encoder_reset_test.cpp
 *
 * Test utility for sending quadrature encoder reset events via uORB
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/getopt.h>
#include <uORB/topics/quad_encoder_reset.h>
#include <uORB/uORB.h>
#include <drivers/drv_hrt.h>

extern "C" __EXPORT int quad_encoder_reset_test_main(int argc, char *argv[]);

static void usage()
{
    PRINT_MODULE_DESCRIPTION(
        R"DESCR_STR(
### Description
Test utility for sending quadrature encoder reset events via uORB.

This utility publishes quad_encoder_reset messages that the quadrature_encoder
driver will listen for and use to reset encoder positions.

### Examples
Reset encoder instance 0:
$ quad_encoder_reset_test 0

Reset encoder instance 1:
$ quad_encoder_reset_test 1
)DESCR_STR");

    PRINT_MODULE_USAGE_NAME("quad_encoder_reset_test", "command");
    PRINT_MODULE_USAGE_ARG("instance", "Encoder instance to reset (0-7)", false);
}

int quad_encoder_reset_test_main(int argc, char *argv[])
{
    if (argc < 2) {
        usage();
        return 1;
    }

    // Parse instance number
    int instance = atoi(argv[1]);
    if (instance < 0 || instance > 7) {
        PX4_ERR("Invalid instance %d, must be 0-7", instance);
        return 1;
    }

    // Create and publish reset message
    quad_encoder_reset_s reset_msg{};
    reset_msg.timestamp = hrt_absolute_time();
    reset_msg.instance = instance;

    orb_advert_t reset_pub = orb_advertise(ORB_ID(quad_encoder_reset), &reset_msg);
    if (reset_pub == nullptr) {
        PX4_ERR("Failed to advertise quad_encoder_reset topic");
        return 1;
    }

    if (orb_publish(ORB_ID(quad_encoder_reset), reset_pub, &reset_msg) != OK) {
        PX4_ERR("Failed to publish reset message");
        orb_unadvertise(reset_pub);
        return 1;
    }

    orb_unadvertise(reset_pub);
    PX4_INFO("Sent reset event for encoder instance %d", instance);
    return 0;
}
