/****************************************************************************
 *
 *   Copyright (C) 2025 PX4 Development Team. All rights reserved.
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

/*
 * @file motor_pwm.c
 *
 * High-frequency motor PWM driver supporting motor controllers connected to STM32 timer blocks.
 * Uses the same io_timer infrastructure as PWM servo but optimized for motor control frequencies.
 *
 * Works with any of the 'generic' or 'advanced' STM32 timers that have output pins.
 * Supports frequencies from 10 kHz to 100 kHz for motor control applications.
 */

 #include <px4_platform_common/log.h>
#include <px4_platform_common/px4_config.h>
#include <nuttx/arch.h>
#include <nuttx/irq.h>

#include <sys/types.h>
#include <stdbool.h>
#include <stdint.h>

#include <assert.h>
#include <debug.h>
#include <time.h>
#include <queue.h>
#include <errno.h>
#include <string.h>
#include <stdio.h>

#include <arch/board/board.h>
#include <drivers/drv_motor_pwm.h>

#include <px4_arch/io_timer.h>

#include <stm32_tim.h>

/* Store current duty cycles for each channel */
static float motor_pwm_duty_cycles[MAX_TIMER_IO_CHANNELS] = {0};

int up_motor_pwm_set(unsigned channel, uint16_t value)
{
	return io_timer_set_ccr(channel, value);
}

uint16_t up_motor_pwm_get(unsigned channel)
{
	return io_channel_get_ccr(channel);
}

int up_motor_pwm_set_duty_cycle(unsigned channel, float duty_cycle)
{
	if (channel >= MAX_TIMER_IO_CHANNELS) {
		return -EINVAL;
	}

	/* Clamp duty cycle */
	if (duty_cycle < 0.0f) duty_cycle = 0.0f;
	if (duty_cycle > 1.0f) duty_cycle = 1.0f;

	/* Store the duty cycle */
	motor_pwm_duty_cycles[channel] = duty_cycle;

	/* Get timer period for this channel - need to access timer registers */
	/* The period is calculated as (BOARD_PWM_FREQ / rate) - 1 in timer_set_rate() */
	uint32_t period = io_channel_get_timer_period(channel);
	if (period == 0) {
		return -EINVAL;  /* Timer not configured */
	}

	uint32_t ccr_value = (uint32_t)(duty_cycle * (period + 1));

	/* Ensure CCR doesn't exceed period */
	if (ccr_value > period) {
		ccr_value = period;
	}

	return io_timer_set_ccr(channel, ccr_value);
}

float up_motor_pwm_get_duty_cycle(unsigned channel)
{
	if (channel >= MAX_TIMER_IO_CHANNELS) {
		return -1.0f;
	}

	return motor_pwm_duty_cycles[channel];
}

int up_motor_pwm_init(uint32_t channel_mask)
{
	/* Init channels */
	uint32_t current = io_timer_get_mode_channels(IOTimerChanMode_MotorPWM);

	/* First free the current set of motor PWM channels */
	for (unsigned channel = 0; current != 0 && channel < MAX_TIMER_IO_CHANNELS; channel++) {
		if (current & (1 << channel)) {
			io_timer_set_enable(false, IOTimerChanMode_MotorPWM, 1 << channel);
			io_timer_unallocate_channel(channel);
			current &= ~(1 << channel);
		}
	}

	/* Now allocate the new set */
	int ret_val = OK;
	int channels_init_mask = 0;
	int last_error = 0;

	for (unsigned channel = 0; channel_mask != 0 && channel < MAX_TIMER_IO_CHANNELS; channel++) {
		if (channel_mask & (1 << channel)) {
			/* Initialize channel for motor PWM mode */
			ret_val = io_timer_channel_init(channel, IOTimerChanMode_MotorPWM, NULL, NULL);
			channel_mask &= ~(1 << channel);

			if (OK == ret_val) {
				channels_init_mask |= 1 << channel;
				motor_pwm_duty_cycles[channel] = 0.0f;  // Initialize duty cycle to 0

			} else if (ret_val == -EBUSY) {
				/* either timer or channel already used - this is not fatal */
				ret_val = 0;
			} else {
				/* This is the error case - log it but continue with other channels */
				last_error = ret_val;
				ret_val = 0; // Don't fail immediately, try other channels
			}
		}
	}

	/* Set default motor PWM frequency to 25kHz for all initialized channels */
	if (channels_init_mask > 0) {
		up_motor_pwm_set_rate(MOTOR_PWM_DEFAULT_FREQ);
		return channels_init_mask;  // Return success with the channels that worked
	} else {
		return last_error != 0 ? last_error : -EINVAL;
	}
}

void up_motor_pwm_deinit(uint32_t channel_mask)
{
	/* disable the motor PWM outputs */
	up_motor_pwm_arm(false, channel_mask);

	/* Unallocate channels similar to PWM servo */
	for (unsigned channel = 0; channel < MAX_TIMER_IO_CHANNELS; channel++) {
		if (channel_mask & (1 << channel)) {
			io_timer_unallocate_channel(channel);
			motor_pwm_duty_cycles[channel] = 0.0f;
		}
	}
}

int up_motor_pwm_set_rate(unsigned rate)
{
	/* Validate motor PWM rate range */
	if ((rate < MOTOR_PWM_RATE_LOWER_LIMIT) || (rate > MOTOR_PWM_RATE_UPPER_LIMIT)) {
		return -ERANGE;
	}

	/* Set rate for all motor PWM timer groups */
	int ret = OK;
	for (unsigned group = 0; group < MAX_IO_TIMERS; group++) {
		uint32_t group_channels = up_motor_pwm_get_rate_group(group);
		if (group_channels != 0) {
			int group_ret = up_motor_pwm_set_rate_group_update(group, rate);
			if (group_ret != OK) {
				ret = group_ret;
			}
		}
	}

	return ret;
}

uint32_t up_motor_pwm_get_rate_group(unsigned group)
{
	/* only return the set of channels in the group which we own */
	return io_timer_get_mode_channels(IOTimerChanMode_MotorPWM) &
	       io_timer_get_group(group);
}

int up_motor_pwm_set_rate_group_update(unsigned group, unsigned rate)
{
	if ((group >= MAX_IO_TIMERS) || (io_timers[group].base == 0)) {
		return ERROR;
	}

	/* Validate motor PWM rate range */
	if ((rate < MOTOR_PWM_RATE_LOWER_LIMIT) || (rate > MOTOR_PWM_RATE_UPPER_LIMIT)) {
		return -ERANGE;
	}

	return io_timer_set_pwm_rate(group, rate);
}

void up_motor_pwm_arm(bool armed, uint32_t channel_mask)
{
	io_timer_set_enable(armed, IOTimerChanMode_MotorPWM, channel_mask);
}

void up_motor_pwm_update(uint32_t channels_mask)
{
	io_timer_trigger(channels_mask);
}
