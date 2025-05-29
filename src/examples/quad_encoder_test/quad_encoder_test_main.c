/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file quad_encoder_test_main.c
 *
 * Test application for quadrature encoder driver
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/getopt.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_quad_encoder.h>

#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>

#include "../../lib/quad_encoder/quadencoder_ioctl.h"

/* External example functions */
extern int motor_control_example(void);
extern int linear_position_example(void);

static void usage(const char *command)
{
	PX4_INFO("usage: %s {test|motor|linear|subscribe}", command);
	PX4_INFO("  test      - Basic encoder reading test");
	PX4_INFO("  motor     - Motor control example");
	PX4_INFO("  linear    - Linear position example");
	PX4_INFO("  subscribe - Subscribe to uORB topic");
}

/****************************************************************************
 * Name: basic_encoder_test
 *
 * Description:
 *   Basic test of encoder reading functionality
 *
 ****************************************************************************/
static int basic_encoder_test(void)
{
	int fd;
	int32_t position;
	int32_t velocity;
	uint32_t resolution;
	float angle_deg;

	PX4_INFO("Starting basic encoder test...");

	/* Open encoder device */
	fd = open("/dev/qe0", O_RDONLY);

	if (fd < 0) {
		PX4_ERR("Failed to open encoder device /dev/qe0");
		return -1;
	}

	/* Set encoder parameters */
	ioctl(fd, QEIOC_SETRESOLUTION, 1024);
	ioctl(fd, QEIOC_SETMODE, 4); /* X4 mode */

	/* Reset position */
	ioctl(fd, QEIOC_POSITION, 0);

	PX4_INFO("Encoder test running. Rotate encoder to see changes.");
	PX4_INFO("Press Ctrl+C to stop.");

	/* Test loop */
	for (int i = 0; i < 100; i++) {
		/* Read position */
		if (read(fd, &position, sizeof(position)) == sizeof(position)) {
			/* Get velocity */
			ioctl(fd, QEIOC_GETVELOCITY, &velocity);

			/* Get resolution */
			ioctl(fd, QEIOC_GETRESOLUTION, &resolution);

			/* Calculate angle */
			angle_deg = (float)position * 360.0f / (float)resolution;

			PX4_INFO("Position: %ld counts, Angle: %.2f deg, Velocity: %ld counts/s",
				 (long)position, (double)angle_deg, (long)velocity);
		} else {
			PX4_ERR("Failed to read encoder position");
			break;
		}

		/* Wait 100ms */
		px4_usleep(100000);
	}

	close(fd);
	PX4_INFO("Basic encoder test completed");
	return 0;
}

/****************************************************************************
 * Name: subscribe_test
 *
 * Description:
 *   Test subscribing to encoder uORB topic
 *
 ****************************************************************************/
static int subscribe_test(void)
{
	int sensor_sub = orb_subscribe(ORB_ID(sensor_quad_encoder));

	if (sensor_sub < 0) {
		PX4_ERR("Failed to subscribe to sensor_quad_encoder");
		return -1;
	}

	PX4_INFO("Subscribed to sensor_quad_encoder topic");
	PX4_INFO("Waiting for encoder data... (Press Ctrl+C to stop)");

	struct sensor_quad_encoder_s encoder_data;
	bool updated = false;

	for (int i = 0; i < 100; i++) {
		/* Check for updates */
		orb_check(sensor_sub, &updated);

		if (updated) {
			/* Copy data */
			orb_copy(ORB_ID(sensor_quad_encoder), sensor_sub, &encoder_data);

			PX4_INFO("Device %lu: pos=%ld, vel=%ld, angle=%.2f rad, speed=%.2f m/s",
				 (unsigned long)encoder_data.device_id,
				 (long)encoder_data.position,
				 (long)encoder_data.velocity,
				 (double)encoder_data.angle,
				 (double)encoder_data.speed);
		}

		px4_usleep(50000); /* 20Hz */
	}

	orb_unsubscribe(sensor_sub);
	PX4_INFO("Subscribe test completed");
	return 0;
}

/****************************************************************************
 * Name: quad_encoder_test_main
 *
 * Description:
 *   Main entry point for quad encoder test application
 *
 ****************************************************************************/
__EXPORT int quad_encoder_test_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage(argv[0]);
		return 1;
	}

	if (strcmp(argv[1], "test") == 0) {
		return basic_encoder_test();

	} else if (strcmp(argv[1], "motor") == 0) {
		return motor_control_example();

	} else if (strcmp(argv[1], "linear") == 0) {
		return linear_position_example();

	} else if (strcmp(argv[1], "subscribe") == 0) {
		return subscribe_test();

	} else {
		PX4_ERR("Unknown command: %s", argv[1]);
		usage(argv[0]);
		return 1;
	}
}
