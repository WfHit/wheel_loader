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
 * @file linear_position_example.c
 *
 * Example showing how to use quadrature encoders for linear actuator
 * position control and monitoring. This example demonstrates using
 * encoders with lead screws, rack and pinion systems, or cable drives.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/log.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_quad_encoder.h>
#include <uORB/topics/actuator_outputs.h>
#include <uORB/topics/vehicle_command.h>

#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>

#include "../../lib/quad_encoder/quadencoder_ioctl.h"

/* Linear actuator parameters */
#define ENCODER_PPR         1000      /* Encoder pulses per revolution */
#define LEAD_SCREW_PITCH    0.002f    /* Lead screw pitch in meters (2mm) */
#define ACTUATOR_LENGTH     0.200f    /* Maximum actuator travel in meters (200mm) */
#define MAX_VELOCITY        0.050f    /* Maximum velocity in m/s (50mm/s) */
#define HOMING_VELOCITY     0.010f    /* Homing velocity in m/s (10mm/s) */
#define CONTROL_FREQ        100       /* Control loop frequency (Hz) */

/* Safety limits */
#define MIN_POSITION        0.000f    /* Minimum position in meters */
#define MAX_POSITION        0.180f    /* Maximum position in meters (with safety margin) */
#define SOFT_LIMIT_MARGIN   0.010f    /* Soft limit margin in meters */

/* Linear actuator state */
struct linear_actuator_s {
	int encoder_fd;          /* Encoder file descriptor */
	int encoder_sub;         /* Encoder subscription */
	int command_sub;         /* Command subscription */
	orb_advert_t output_pub; /* Actuator output publisher */
	orb_advert_t status_pub; /* Status publisher */

	float position;          /* Current position in meters */
	float velocity;          /* Current velocity in m/s */
	float target_position;   /* Target position in meters */
	float target_velocity;   /* Target velocity in m/s */

	float counts_per_meter;  /* Encoder counts per meter */
	float zero_position;     /* Zero position offset in counts */

	bool homed;              /* Home position found */
	bool enabled;            /* Actuator enabled */
	bool limit_switch_min;   /* Minimum limit switch */
	bool limit_switch_max;   /* Maximum limit switch */

	/* Control parameters */
	float kp;                /* Position proportional gain */
	float ki;                /* Position integral gain */
	float kd;                /* Position derivative gain */
	float integral;          /* Integral accumulator */
	float previous_error;    /* Previous error for derivative */

	/* Status */
	uint32_t fault_flags;    /* Fault condition flags */
	bool running;            /* System running */
};

/* Fault flag definitions */
#define FAULT_OVER_TRAVEL    (1 << 0)
#define FAULT_UNDER_TRAVEL   (1 << 1)
#define FAULT_VELOCITY_LIMIT (1 << 2)
#define FAULT_ENCODER_ERROR  (1 << 3)
#define FAULT_NOT_HOMED      (1 << 4)

/****************************************************************************
 * Name: linear_actuator_init
 *
 * Description:
 *   Initialize linear actuator system
 *
 ****************************************************************************/
static int linear_actuator_init(struct linear_actuator_s *actuator)
{
	/* Calculate encoder scaling */
	actuator->counts_per_meter = (float)(ENCODER_PPR * 4) / LEAD_SCREW_PITCH;

	/* Open encoder device */
	actuator->encoder_fd = open("/dev/qe1", O_RDONLY);
	if (actuator->encoder_fd < 0) {
		PX4_ERR("Failed to open encoder device");
		return -1;
	}

	/* Configure encoder */
	ioctl(actuator->encoder_fd, QEIOC_SETRESOLUTION, ENCODER_PPR * 4);
	ioctl(actuator->encoder_fd, QEIOC_POSITION, 0);

	/* Subscribe to encoder topic */
	actuator->encoder_sub = orb_subscribe(ORB_ID(sensor_quad_encoder));
	if (actuator->encoder_sub < 0) {
		PX4_ERR("Failed to subscribe to encoder topic");
		close(actuator->encoder_fd);
		return -1;
	}

	/* Subscribe to command topic */
	actuator->command_sub = orb_subscribe(ORB_ID(vehicle_command));
	if (actuator->command_sub < 0) {
		PX4_ERR("Failed to subscribe to command topic");
		orb_unsubscribe(actuator->encoder_sub);
		close(actuator->encoder_fd);
		return -1;
	}

	/* Advertise output topic */
	struct actuator_outputs_s output = {};
	actuator->output_pub = orb_advertise(ORB_ID(actuator_outputs), &output);
	if (actuator->output_pub == nullptr) {
		PX4_ERR("Failed to advertise output topic");
		orb_unsubscribe(actuator->command_sub);
		orb_unsubscribe(actuator->encoder_sub);
		close(actuator->encoder_fd);
		return -1;
	}

	/* Initialize state */
	actuator->position = 0.0f;
	actuator->velocity = 0.0f;
	actuator->target_position = 0.0f;
	actuator->target_velocity = 0.0f;
	actuator->zero_position = 0.0f;

	actuator->homed = false;
	actuator->enabled = false;
	actuator->limit_switch_min = false;
	actuator->limit_switch_max = false;

	/* Control parameters */
	actuator->kp = 50.0f;
	actuator->ki = 5.0f;
	actuator->kd = 2.0f;
	actuator->integral = 0.0f;
	actuator->previous_error = 0.0f;

	actuator->fault_flags = 0;
	actuator->running = false;

	return 0;
}

/****************************************************************************
 * Name: update_position_feedback
 *
 * Description:
 *   Update position feedback from encoder
 *
 ****************************************************************************/
static int update_position_feedback(struct linear_actuator_s *actuator)
{
	struct sensor_quad_encoder_s encoder_data;
	bool updated = false;

	orb_check(actuator->encoder_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(sensor_quad_encoder), actuator->encoder_sub, &encoder_data);

		/* Convert encoder counts to position */
		float position_counts = (float)encoder_data.position - actuator->zero_position;
		actuator->position = position_counts / actuator->counts_per_meter;

		/* Calculate velocity from encoder velocity */
		actuator->velocity = (float)encoder_data.velocity / actuator->counts_per_meter;

		return 1; /* Updated */
	}

	return 0; /* No update */
}

/****************************************************************************
 * Name: check_safety_limits
 *
 * Description:
 *   Check position and velocity safety limits
 *
 ****************************************************************************/
static void check_safety_limits(struct linear_actuator_s *actuator)
{
	/* Clear previous fault flags */
	actuator->fault_flags &= ~(FAULT_OVER_TRAVEL | FAULT_UNDER_TRAVEL | FAULT_VELOCITY_LIMIT);

	/* Check position limits */
	if (actuator->position < MIN_POSITION) {
		actuator->fault_flags |= FAULT_UNDER_TRAVEL;
		actuator->limit_switch_min = true;
	} else {
		actuator->limit_switch_min = false;
	}

	if (actuator->position > MAX_POSITION) {
		actuator->fault_flags |= FAULT_OVER_TRAVEL;
		actuator->limit_switch_max = true;
	} else {
		actuator->limit_switch_max = false;
	}

	/* Check velocity limits */
	if (fabsf(actuator->velocity) > MAX_VELOCITY) {
		actuator->fault_flags |= FAULT_VELOCITY_LIMIT;
	}

	/* Check homing status */
	if (!actuator->homed) {
		actuator->fault_flags |= FAULT_NOT_HOMED;
	}
}

/****************************************************************************
 * Name: home_actuator
 *
 * Description:
 *   Home the linear actuator to find zero position
 *
 ****************************************************************************/
static int home_actuator(struct linear_actuator_s *actuator)
{
	struct actuator_outputs_s output = {};
	int32_t start_position, current_position;
	int timeout = 0;

	PX4_INFO("Starting homing sequence...");

	/* Get starting position */
	read(actuator->encoder_fd, &start_position, sizeof(start_position));

	/* Move toward minimum position at homing velocity */
	output.timestamp = hrt_absolute_time();
	output.noutputs = 1;
	output.output[0] = -HOMING_VELOCITY / MAX_VELOCITY; /* Negative for minimum direction */

	/* Move until position stops changing (hit limit) */
	while (timeout < 500) { /* 5 second timeout */
		orb_publish(ORB_ID(actuator_outputs), actuator->output_pub, &output);

		px4_usleep(10000); /* 10ms */

		read(actuator->encoder_fd, &current_position, sizeof(current_position));

		/* Check if position has stopped changing */
		if (abs(current_position - start_position) < 2) {
			timeout++;
		} else {
			timeout = 0;
			start_position = current_position;
		}
	}

	/* Stop movement */
	output.output[0] = 0.0f;
	orb_publish(ORB_ID(actuator_outputs), actuator->output_pub, &output);

	/* Set zero position */
	actuator->zero_position = (float)current_position;
	actuator->position = 0.0f;
	actuator->homed = true;

	PX4_INFO("Homing completed. Zero position set to %ld counts", (long)current_position);

	/* Move to safe position (10mm from minimum) */
	actuator->target_position = 0.010f;

	return 0;
}

/****************************************************************************
 * Name: position_control_step
 *
 * Description:
 *   Execute position control step
 *
 ****************************************************************************/
static void position_control_step(struct linear_actuator_s *actuator, float dt)
{
	float error, output;
	float derivative;
	struct actuator_outputs_s actuator_output = {};

	/* Calculate position error */
	error = actuator->target_position - actuator->position;

	/* PID control */
	float p_term = actuator->kp * error;

	/* Integral term with windup protection */
	actuator->integral += error * dt;
	if (actuator->integral > 1.0f) actuator->integral = 1.0f;
	if (actuator->integral < -1.0f) actuator->integral = -1.0f;
	float i_term = actuator->ki * actuator->integral;

	/* Derivative term */
	derivative = (error - actuator->previous_error) / dt;
	float d_term = actuator->kd * derivative;

	/* Calculate output */
	output = p_term + i_term + d_term;

	/* Clamp output */
	if (output > 1.0f) output = 1.0f;
	if (output < -1.0f) output = -1.0f;

	/* Apply safety limits */
	if ((actuator->limit_switch_min && output < 0.0f) ||
	    (actuator->limit_switch_max && output > 0.0f)) {
		output = 0.0f;
	}

	/* Disable output if not homed or faults present */
	if (!actuator->homed || (actuator->fault_flags & ~FAULT_NOT_HOMED)) {
		output = 0.0f;
	}

	/* Publish actuator command */
	actuator_output.timestamp = hrt_absolute_time();
	actuator_output.noutputs = 1;
	actuator_output.output[0] = output;

	orb_publish(ORB_ID(actuator_outputs), actuator->output_pub, &actuator_output);

	/* Save error for next iteration */
	actuator->previous_error = error;

	PX4_DEBUG("Pos: %.4f m, Target: %.4f m, Error: %.4f m, Output: %.3f",
		  (double)actuator->position, (double)actuator->target_position,
		  (double)error, (double)output);
}

/****************************************************************************
 * Name: linear_position_example
 *
 * Description:
 *   Main linear position control example
 *
 ****************************************************************************/
int linear_position_example(void)
{
	struct linear_actuator_s actuator;
	hrt_abstime last_time, current_time;
	float dt;
	int step = 0;

	PX4_INFO("Starting linear position control example...");

	/* Initialize actuator system */
	if (linear_actuator_init(&actuator) < 0) {
		PX4_ERR("Failed to initialize linear actuator");
		return -1;
	}

	actuator.running = true;
	actuator.enabled = true;
	last_time = hrt_absolute_time();

	/* Home the actuator first */
	if (home_actuator(&actuator) < 0) {
		PX4_ERR("Failed to home actuator");
		goto cleanup;
	}

	PX4_INFO("Linear actuator control running. Executing position sequence...");

	/* Control loop */
	while (actuator.running && step < 2000) {
		current_time = hrt_absolute_time();
		dt = (current_time - last_time) / 1e6f;

		/* Update feedback */
		update_position_feedback(&actuator);

		/* Check safety limits */
		check_safety_limits(&actuator);

		/* Set target positions for different test phases */
		if (step < 200) {
			/* Phase 1: Move to 50mm */
			actuator.target_position = 0.050f;

		} else if (step < 400) {
			/* Phase 2: Move to 100mm */
			actuator.target_position = 0.100f;

		} else if (step < 600) {
			/* Phase 3: Move to 150mm */
			actuator.target_position = 0.150f;

		} else if (step < 800) {
			/* Phase 4: Move to 75mm */
			actuator.target_position = 0.075f;

		} else if (step < 1000) {
			/* Phase 5: Move to 25mm */
			actuator.target_position = 0.025f;

		} else {
			/* Phase 6: Return to home position */
			actuator.target_position = 0.010f;
		}

		/* Execute control step */
		position_control_step(&actuator, dt);

		/* Status output every 50 steps */
		if (step % 50 == 0) {
			PX4_INFO("Step %d: Pos=%.1f mm, Target=%.1f mm, Vel=%.1f mm/s, Faults=0x%lx",
				 step,
				 (double)(actuator.position * 1000.0f),
				 (double)(actuator.target_position * 1000.0f),
				 (double)(actuator.velocity * 1000.0f),
				 (unsigned long)actuator.fault_flags);
		}

		/* Check for faults */
		if (actuator.fault_flags & (FAULT_OVER_TRAVEL | FAULT_UNDER_TRAVEL)) {
			PX4_WARN("Safety limit reached, stopping");
			break;
		}

		last_time = current_time;
		step++;

		/* Control loop timing */
		px4_usleep(1000000 / CONTROL_FREQ);
	}

cleanup:
	/* Stop actuator */
	struct actuator_outputs_s output = {};
	output.timestamp = hrt_absolute_time();
	output.noutputs = 1;
	output.output[0] = 0.0f;
	orb_publish(ORB_ID(actuator_outputs), actuator.output_pub, &output);

	/* Cleanup */
	orb_unadvertise(actuator.output_pub);
	orb_unsubscribe(actuator.command_sub);
	orb_unsubscribe(actuator.encoder_sub);
	close(actuator.encoder_fd);

	PX4_INFO("Linear position control example completed");
	return 0;
}
