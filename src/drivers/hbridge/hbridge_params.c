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
 * H-Bridge PWM minimum duty cycle
 *
 * The minimum duty cycle for H-Bridge PWM output.
 * This is used to set the lower bound for motor control.
 *
 * @unit percent
 * @min 0.0
 * @max 50.0
 * @decimal 1
 * @group HBridge
 */
PARAM_DEFINE_FLOAT(HBRIDGE_PWM_MIN, 5.0f);

/**
 * H-Bridge PWM maximum duty cycle
 *
 * The maximum duty cycle for H-Bridge PWM output.
 * This is used to set the upper bound for motor control.
 *
 * @unit percent
 * @min 50.0
 * @max 100.0
 * @decimal 1
 * @group HBridge
 */
PARAM_DEFINE_FLOAT(HBRIDGE_PWM_MAX, 95.0f);

/**
 * H-Bridge PWM frequency
 *
 * The PWM frequency for H-Bridge output channels.
 * Higher frequencies can reduce motor noise but may increase switching losses.
 *
 * @unit Hz
 * @min 50
 * @max 50000
 * @decimal 0
 * @group HBridge
 */
PARAM_DEFINE_INT32(HBRIDGE_PWM_FREQ, 1000);

/**
 * H-Bridge PWM channel 0
 *
 * PWM channel number for H-Bridge channel 0.
 * Set to 0 to disable.
 *
 * @min 0
 * @max 16
 * @decimal 0
 * @group HBridge
 */
PARAM_DEFINE_INT32(HBRIDGE_CH0_PWM, 1);

/**
 * H-Bridge PWM channel 1
 *
 * PWM channel number for H-Bridge channel 1.
 * Set to 0 to disable.
 *
 * @min 0
 * @max 16
 * @decimal 0
 * @group HBridge
 */
PARAM_DEFINE_INT32(HBRIDGE_CH1_PWM, 2);

/**
 * H-Bridge channel 0 minimum duty cycle
 *
 * The minimum duty cycle for H-Bridge channel 0 PWM output.
 *
 * @unit percent
 * @min 0.0
 * @max 50.0
 * @decimal 1
 * @group HBridge
 */
PARAM_DEFINE_FLOAT(HBRIDGE_CH0_MIN, 5.0f);

/**
 * H-Bridge channel 0 maximum duty cycle
 *
 * The maximum duty cycle for H-Bridge channel 0 PWM output.
 *
 * @unit percent
 * @min 50.0
 * @max 100.0
 * @decimal 1
 * @group HBridge
 */
PARAM_DEFINE_FLOAT(HBRIDGE_CH0_MAX, 95.0f);

/**
 * H-Bridge channel 1 minimum duty cycle
 *
 * The minimum duty cycle for H-Bridge channel 1 PWM output.
 *
 * @unit percent
 * @min 0.0
 * @max 50.0
 * @decimal 1
 * @group HBridge
 */
PARAM_DEFINE_FLOAT(HBRIDGE_CH1_MIN, 5.0f);

/**
 * H-Bridge channel 1 maximum duty cycle
 *
 * The maximum duty cycle for H-Bridge channel 1 PWM output.
 *
 * @unit percent
 * @min 50.0
 * @max 100.0
 * @decimal 1
 * @group HBridge
 */
PARAM_DEFINE_FLOAT(HBRIDGE_CH1_MAX, 95.0f);

/**
 * H-Bridge deadtime
 *
 * Deadtime between switching the H-Bridge direction to prevent shoot-through.
 * This time is enforced when changing direction to ensure one set of MOSFETs
 * is fully off before the other set turns on.
 *
 * @unit ms
 * @min 0
 * @max 100
 * @decimal 1
 * @group HBridge
 */
PARAM_DEFINE_FLOAT(HBRIDGE_DEADTIME, 5.0f);

/**
 * H-Bridge enable auto-publish
 *
 * Enable automatic publishing of H-Bridge status messages.
 * When enabled, the driver will periodically publish status information.
 *
 * @boolean
 * @group HBridge
 */
PARAM_DEFINE_INT32(HBRIDGE_AUTO_PUB, 1);

/**
 * H-Bridge status publish rate
 *
 * Rate at which H-Bridge status messages are published when auto-publish is enabled.
 *
 * @unit Hz
 * @min 1
 * @max 100
 * @decimal 0
 * @group HBridge
 */
PARAM_DEFINE_INT32(HBRIDGE_PUB_RATE, 10);
