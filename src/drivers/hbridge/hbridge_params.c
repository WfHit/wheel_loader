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
 * H-Bridge PWM channel 0
 *
 * PWM channel number for H-Bridge channel 0.
 *
 * @min 0
 * @max 16
 * @group HBridge
 */
PARAM_DEFINE_INT32(HBRIDGE_CH0_PWM, 0);

/**
 * H-Bridge PWM channel 1
 *
 * PWM channel number for H-Bridge channel 1.
 *
 * @min 0
 * @max 16
 * @group HBridge
 */
PARAM_DEFINE_INT32(HBRIDGE_CH1_PWM, 1);

/**
 * H-Bridge channel 0 min PWM
 *
 * Minimum PWM pulse width for channel 0.
 *
 * @unit us
 * @min 800
 * @max 1500
 * @group HBridge
 */
PARAM_DEFINE_FLOAT(HBRIDGE_CH0_MIN, 1000.0f);

/**
 * H-Bridge channel 0 max PWM
 *
 * Maximum PWM pulse width for channel 0.
 *
 * @unit us
 * @min 1500
 * @max 2200
 * @group HBridge
 */
PARAM_DEFINE_FLOAT(HBRIDGE_CH0_MAX, 2000.0f);

/**
 * H-Bridge channel 1 min PWM
 *
 * Minimum PWM pulse width for channel 1.
 *
 * @unit us
 * @min 800
 * @max 1500
 * @group HBridge
 */
PARAM_DEFINE_FLOAT(HBRIDGE_CH1_MIN, 1000.0f);

/**
 * H-Bridge channel 1 max PWM
 *
 * Maximum PWM pulse width for channel 1.
 *
 * @unit us
 * @min 1500
 * @max 2200
 * @group HBridge
 */
PARAM_DEFINE_FLOAT(HBRIDGE_CH1_MAX, 2000.0f);

/**
 * H-Bridge PWM frequency
 *
 * PWM frequency for all H-Bridge channels.
 *
 * @unit Hz
 * @min 50
 * @max 10000
 * @group HBridge
 */
PARAM_DEFINE_FLOAT(HBRIDGE_PWM_FREQ, 1000.0f);
