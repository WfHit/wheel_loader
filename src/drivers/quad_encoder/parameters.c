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

#include <px4_platform_common/px4_config.h>
#include <lib/parameters/param.h>

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
 * @max 8
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

/**
 * Enable wheel odometry publishing
 *
 * Enable publishing of computed vehicle odometry from wheel encoder data.
 * Only enable this for wheel applications.
 *
 * @group Quadrature Encoder
 * @boolean
 */
PARAM_DEFINE_INT32(QE_ENABLE_ODOM, 0);

/**
 * Wheel base distance (for odometry)
 *
 * Distance between left and right wheels in meters.
 * Used for differential drive odometry calculations when QE_ENABLE_ODOM is true.
 *
 * @group Quadrature Encoder
 * @unit m
 * @min 0.1
 * @max 5.0
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(QE_WHEEL_BASE, 1.5f);

/**
 * Wheel radius (for odometry)
 *
 * Radius of the wheels in meters.
 * Used to convert encoder pulses to linear distance when QE_ENABLE_ODOM is true.
 *
 * @group Quadrature Encoder
 * @unit m
 * @min 0.05
 * @max 1.0
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(QE_WHEEL_RADIUS, 0.3f);

/**
 * Encoder 0 type
 *
 * Type of encoder 0: 0=Rotary (wheel/motor), 1=Linear (actuator)
 *
 * @group Quadrature Encoder
 * @value 0 Rotary
 * @value 1 Linear
 */
PARAM_DEFINE_INT32(QE_TYPE_0, 0);

/**
 * Encoder 1 type
 *
 * Type of encoder 1: 0=Rotary (wheel/motor), 1=Linear (actuator)
 *
 * @group Quadrature Encoder
 * @value 0 Rotary
 * @value 1 Linear
 */
PARAM_DEFINE_INT32(QE_TYPE_1, 0);

/**
 * Encoder 2 type
 *
 * Type of encoder 2: 0=Rotary (wheel/motor), 1=Linear (actuator)
 *
 * @group Quadrature Encoder
 * @value 0 Rotary
 * @value 1 Linear
 */
PARAM_DEFINE_INT32(QE_TYPE_2, 0);

/**
 * Encoder 3 type
 *
 * Type of encoder 3: 0=Rotary (wheel/motor), 1=Linear (actuator)
 *
 * @group Quadrature Encoder
 * @value 0 Rotary
 * @value 1 Linear
 */
PARAM_DEFINE_INT32(QE_TYPE_3, 0);

/**
 * Encoder 0 gear ratio / screw pitch
 *
 * For rotary encoders: gear ratio (output/input)
 * For linear encoders: screw pitch in mm per revolution
 *
 * @group Quadrature Encoder
 * @min 0.001
 * @max 1000.0
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(QE_GEAR_RATIO_0, 1.0f);

/**
 * Encoder 1 gear ratio / screw pitch
 *
 * For rotary encoders: gear ratio (output/input)
 * For linear encoders: screw pitch in mm per revolution
 *
 * @group Quadrature Encoder
 * @min 0.001
 * @max 1000.0
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(QE_GEAR_RATIO_1, 1.0f);

/**
 * Encoder 2 gear ratio / screw pitch
 *
 * For rotary encoders: gear ratio (output/input)
 * For linear encoders: screw pitch in mm per revolution
 *
 * @group Quadrature Encoder
 * @min 0.001
 * @max 1000.0
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(QE_GEAR_RATIO_2, 1.0f);

/**
 * Encoder 3 gear ratio / screw pitch
 *
  * For rotary encoders: gear ratio (output/input)
 * For linear encoders: screw pitch in mm per revolution
 *
 * @group Quadrature Encoder
 * @min 0.001
 * @max 1000.0
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(QE_GEAR_RATIO_3, 1.0f);
