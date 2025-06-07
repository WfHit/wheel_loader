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
 * Quadrature Encoder update rate
 *
 * Rate at which encoder data is read and published.
 *
 * @group Quadrature Encoder
 * @unit Hz
 * @min 10
 * @max 1000
 */
PARAM_DEFINE_INT32(QE_UPDATE_RATE, 100);

/**
 * Number of active encoders
 *
 * Number of quadrature encoders to read and publish.
 * Set to 0 to auto-detect available encoders.
 *
 * @group Quadrature Encoder
 * @min 0
 * @max 4
 */
PARAM_DEFINE_INT32(QE_NUM_ENCODERS, 0);

/**
 * Encoder 0 pulses per revolution
 *
 * Number of encoder pulses per complete revolution for encoder 0.
 * This depends on the encoder resolution and any gear reduction.
 *
 * @group Quadrature Encoder
 * @min 100
 * @max 10000
 */
PARAM_DEFINE_INT32(QE_PPR_0, 1024);

/**
 * Encoder 1 pulses per revolution
 *
 * Number of encoder pulses per complete revolution for encoder 1.
 *
 * @group Quadrature Encoder
 * @min 100
 * @max 10000
 */
PARAM_DEFINE_INT32(QE_PPR_1, 1024);

/**
 * Encoder 2 pulses per revolution
 *
 * Number of encoder pulses per complete revolution for encoder 2.
 *
 * @group Quadrature Encoder
 * @min 100
 * @max 10000
 */
PARAM_DEFINE_INT32(QE_PPR_2, 1024);

/**
 * Encoder 3 pulses per revolution
 *
 * Number of encoder pulses per complete revolution for encoder 3.
 *
 * @group Quadrature Encoder
 * @min 100
 * @max 10000
 */
PARAM_DEFINE_INT32(QE_PPR_3, 1024);

/**
 * Invert encoder 0 direction
 *
 * Invert the direction of encoder 0.
 *
 * @group Quadrature Encoder
 * @boolean
 */
PARAM_DEFINE_INT32(QE_INVERT_0, 0);

/**
 * Invert encoder 1 direction
 *
 * Invert the direction of encoder 1.
 *
 * @group Quadrature Encoder
 * @boolean
 */
PARAM_DEFINE_INT32(QE_INVERT_1, 0);

/**
 * Invert encoder 2 direction
 *
 * Invert the direction of encoder 2.
 *
 * @group Quadrature Encoder
 * @boolean
 */
PARAM_DEFINE_INT32(QE_INVERT_2, 0);

/**
 * Invert encoder 3 direction
 *
 * Invert the direction of encoder 3.
 *
 * @group Quadrature Encoder
 * @boolean
 */
PARAM_DEFINE_INT32(QE_INVERT_3, 0);
