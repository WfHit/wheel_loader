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
 * Left wheel PWM channel
 *
 * PWM channel for left wheel motor control.
 * PWM0 is typically used for left wheel on this board.
 *
 * @min 0
 * @max 15
 * @group HBridge
 */
PARAM_DEFINE_INT32(HBRIDGE_L_PWM, 0);

/**
 * Right wheel PWM channel
 *
 * PWM channel for right wheel motor control.
 * PWM1 is typically used for right wheel on this board.
 *
 * @min 0
 * @max 15
 * @group HBridge
 */
PARAM_DEFINE_INT32(HBRIDGE_R_PWM, 1);

/**
 * H-Bridge PWM frequency
 *
 * PWM frequency for all H-Bridge channels.
 * Motor control uses high frequencies (10-100 kHz) for smooth operation.
 *
 * @unit Hz
 * @min 10000
 * @max 100000
 * @group HBridge
 */
PARAM_DEFINE_FLOAT(HBRIDGE_PWM_FREQ, 25000.0f);
