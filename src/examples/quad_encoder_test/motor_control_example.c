/****************************************************#include <lib/quad_encoder/quad_encoder_ioctl.h>***********************
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
 * @file motor_control_example.c
 *
 * Example showing how to use quadrature encoders for motor control
 * with closed-loop position and velocity control.
 */

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/log.h>

#include <uORB/uORB.h>
#include <uORB/topics/sensor_quad_encoder.h>
#include <uORB/topics/actuator_outputs.h>

#include <sys/ioctl.h>
#include <fcntl.h>
#include <unistd.h>
#include <math.h>

#include "../../lib/quad_encoder/quad_encoder_ioctl.h"

/* Motor control parameters */
#define MOTOR_PPR           1024      /* Pulses per revolution */
#define MOTOR_GEAR_RATIO    10.0f     /* Motor gear ratio */
#define WHEEL_DIAMETER      0.15f     /* Wheel diameter in meters */
#define MAX_RPM             100.0f    /* Maximum motor RPM */
#define CONTROL_FREQ        50        /* Control loop frequency (Hz) */

/* PID controller parameters */
struct pid_controller_s {
	float kp;                /* Proportional gain */
	float ki;                /* Integral gain */
	float kd;                /* Derivative gain */
	float integral;          /* Integral accumulator */
	float previous_error;    /* Previous error for derivative */
	float output_min;        /* Minimum output */
	float output_max;        /* Maximum output */
};

/* Motor control state */
struct motor_state_s {
	int encoder_fd;          /* Encoder file descriptor */
	int motor_sub;           /* Motor subscription */
	int encoder_sub;         /* Encoder subscription */
	orb_advert_t motor_pub;  /* Motor publisher */

	float target_position;   /* Target position in radians */
	float target_velocity;   /* Target velocity in rad/s */
	float current_position;  /* Current position in radians */
	float current_velocity;  /* Current velocity in rad/s */

	struct pid_controller_s pos_pid;  /* Position PID controller */
	struct pid_controller_s vel_pid;  /* Velocity PID controller */

	bool position_mode;      /* Position control mode */
	bool running;            /* Control loop running */
};

/****************************************************************************
 * Name: pid_init
 *
 * Description:
 *   Initialize PID controller
 *
 ****************************************************************************/
static void pid_init(struct pid_controller_s *pid, float kp, float ki, float kd,
		     float output_min, float output_max)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	pid->integral = 0.0f;
	pid->previous_error = 0.0f;
	pid->output_min = output_min;
	pid->output_max = output_max;
}

/****************************************************************************
 * Name: pid_update
 *
 * Description:
 *   Update PID controller
 *
 ****************************************************************************/
static float pid_update(struct pid_controller_s *pid, float error, float dt)
{
	float output;

	/* Proportional term */
	float p_term = pid->kp * error;

	/* Integral term */
	pid->integral += error * dt;
	float i_term = pid->ki * pid->integral;

	/* Derivative term */
	float derivative = (error - pid->previous_error) / dt;
	float d_term = pid->kd * derivative;

	/* Calculate output */
	output = p_term + i_term + d_term;

	/* Clamp output */
	if (output > pid->output_max) {
		output = pid->output_max;
	} else if (output < pid->output_min) {
		output = pid->output_min;
	}

	/* Save error for next iteration */
	pid->previous_error = error;

	return output;
}

/****************************************************************************
 * Name: motor_init
 *
 * Description:
 *   Initialize motor control system
 *
 ****************************************************************************/
static int motor_init(struct motor_state_s *motor)
{
	/* Open encoder device */
	motor->encoder_fd = open("/dev/qe0", O_RDONLY);
	if (motor->encoder_fd < 0) {
		PX4_ERR("Failed to open encoder device");
		return -1;
	}

	/* Configure encoder */
	ioctl(motor->encoder_fd, QEIOC_SETRESOLUTION, MOTOR_PPR * 4); /* X4 mode */
	ioctl(motor->encoder_fd, QEIOC_POSITION, 0); /* Reset position */

	/* Subscribe to encoder topic */
	motor->encoder_sub = orb_subscribe(ORB_ID(sensor_quad_encoder));
	if (motor->encoder_sub < 0) {
		PX4_ERR("Failed to subscribe to encoder topic");
		close(motor->encoder_fd);
		return -1;
	}

	/* Advertise actuator output topic */
	struct actuator_outputs_s motor_output = {};
	motor->motor_pub = orb_advertise(ORB_ID(actuator_outputs), &motor_output);
	if (motor->motor_pub == nullptr) {
		PX4_ERR("Failed to advertise motor topic");
		orb_unsubscribe(motor->encoder_sub);
		close(motor->encoder_fd);
		return -1;
	}

	/* Initialize PID controllers */
	pid_init(&motor->pos_pid, 2.0f, 0.1f, 0.05f, -1.0f, 1.0f);
	pid_init(&motor->vel_pid, 0.5f, 0.05f, 0.01f, -1.0f, 1.0f);

	/* Initialize state */
	motor->target_position = 0.0f;
	motor->target_velocity = 0.0f;
	motor->current_position = 0.0f;
	motor->current_velocity = 0.0f;
	motor->position_mode = true;
	motor->running = false;

	return 0;
}

/****************************************************************************
 * Name: motor_update_feedback
 *
 * Description:
 *   Update motor feedback from encoder
 *
 ****************************************************************************/
static int motor_update_feedback(struct motor_state_s *motor)
{
	struct sensor_quad_encoder_s encoder_data;
	bool updated = false;

	/* Check for encoder updates */
	orb_check(motor->encoder_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(sensor_quad_encoder), motor->encoder_sub, &encoder_data);

		/* Update current state */
		motor->current_position = encoder_data.angle;
		motor->current_velocity = encoder_data.angular_velocity;

		return 1; /* Updated */
	}

	return 0; /* No update */
}

/****************************************************************************
 * Name: motor_control_step
 *
 * Description:
 *   Execute one step of motor control
 *
 ****************************************************************************/
static void motor_control_step(struct motor_state_s *motor, float dt)
{
	float error, output;
	struct actuator_outputs_s motor_output = {};

	if (motor->position_mode) {
		/* Position control mode */
		error = motor->target_position - motor->current_position;
		output = pid_update(&motor->pos_pid, error, dt);

		PX4_DEBUG("Pos Control: target=%.3f, current=%.3f, error=%.3f, output=%.3f",
			  (double)motor->target_position, (double)motor->current_position,
			  (double)error, (double)output);
	} else {
		/* Velocity control mode */
		error = motor->target_velocity - motor->current_velocity;
		output = pid_update(&motor->vel_pid, error, dt);

		PX4_DEBUG("Vel Control: target=%.3f, current=%.3f, error=%.3f, output=%.3f",
			  (double)motor->target_velocity, (double)motor->current_velocity,
			  (double)error, (double)output);
	}

	/* Publish motor command */
	motor_output.timestamp = hrt_absolute_time();
	motor_output.noutputs = 1;
	motor_output.output[0] = output;

	orb_publish(ORB_ID(actuator_outputs), motor->motor_pub, &motor_output);
}

/****************************************************************************
 * Name: motor_control_example
 *
 * Description:
 *   Main motor control example function
 *
 ****************************************************************************/
int motor_control_example(void)
{
	struct motor_state_s motor;
	hrt_abstime last_time, current_time;
	float dt;
	int step = 0;

	PX4_INFO("Starting motor control example...");

	/* Initialize motor control system */
	if (motor_init(&motor) < 0) {
		PX4_ERR("Failed to initialize motor control");
		return -1;
	}

	motor.running = true;
	last_time = hrt_absolute_time();

	PX4_INFO("Motor control running. Will execute position and velocity tests.");

	/* Control loop */
	while (motor.running && step < 1000) {
		current_time = hrt_absolute_time();
		dt = (current_time - last_time) / 1e6f; /* Convert to seconds */

		/* Update feedback */
		motor_update_feedback(&motor);

		/* Set targets based on test phase */
		if (step < 200) {
			/* Phase 1: Position control - move to 90 degrees */
			motor.position_mode = true;
			motor.target_position = M_PI_F / 2.0f; /* 90 degrees */

		} else if (step < 400) {
			/* Phase 2: Position control - move to -90 degrees */
			motor.position_mode = true;
			motor.target_position = -M_PI_F / 2.0f; /* -90 degrees */

		} else if (step < 600) {
			/* Phase 3: Velocity control - 1 rad/s */
			motor.position_mode = false;
			motor.target_velocity = 1.0f;

		} else if (step < 800) {
			/* Phase 4: Velocity control - -1 rad/s */
			motor.position_mode = false;
			motor.target_velocity = -1.0f;

		} else {
			/* Phase 5: Stop */
			motor.position_mode = false;
			motor.target_velocity = 0.0f;
		}

		/* Execute control step */
		motor_control_step(&motor, dt);

		/* Status output every 20 steps */
		if (step % 20 == 0) {
			PX4_INFO("Step %d: Mode=%s, Pos=%.3f rad (%.1f deg), Vel=%.3f rad/s",
				 step,
				 motor.position_mode ? "POS" : "VEL",
				 (double)motor.current_position,
				 (double)(motor.current_position * 180.0f / M_PI_F),
				 (double)motor.current_velocity);
		}

		last_time = current_time;
		step++;

		/* Control loop timing */
		px4_usleep(1000000 / CONTROL_FREQ);
	}

	/* Cleanup */
	orb_unadvertise(motor.motor_pub);
	orb_unsubscribe(motor.encoder_sub);
	close(motor.encoder_fd);

	PX4_INFO("Motor control example completed");
	return 0;
}
