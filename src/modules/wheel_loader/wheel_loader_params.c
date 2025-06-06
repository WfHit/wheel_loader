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
 * Default control mode
 *
 * Default control mode for wheel loader operations (0=Manual, 1=Semi-Auto, 2=Auto)
 *
 * @unit mode
 * @min 0
 * @max 2
 * @decimal 0
 * @group Wheel Loader Control
 */
PARAM_DEFINE_INT32(WL_CONTROL_MODE, 0);

/**
 * Safety timeout for autonomous operations
 *
 * Maximum time in seconds before triggering emergency stop in autonomous mode
 *
 * @unit s
 * @min 1.0
 * @max 30.0
 * @decimal 1
 * @group Wheel Loader Control
 */
PARAM_DEFINE_FLOAT(WL_SAFE_TIMEOUT, 10.0f);

/**
 * Control command timeout
 *
 * Maximum time in seconds without receiving manual control commands before switching to safety mode
 *
 * @unit s
 * @min 0.5
 * @max 5.0
 * @decimal 1
 * @group Wheel Loader Control
 */
PARAM_DEFINE_FLOAT(WL_CTRL_TIMEOUT, 2.0f);

/**
 * Maximum vehicle speed
 *
 * Maximum allowed speed for wheel loader movement in m/s
 *
 * @unit m/s
 * @min 0.1
 * @max 10.0
 * @decimal 1
 * @group Wheel Loader Chassis
 */
PARAM_DEFINE_FLOAT(WL_MAX_SPEED, 2.0f);

/**
 * Maximum steering rate
 *
 * Maximum allowed steering rate in degrees per second
 *
 * @unit deg/s
 * @min 10.0
 * @max 180.0
 * @decimal 1
 * @group Wheel Loader Chassis
 */
PARAM_DEFINE_FLOAT(WL_MAX_STEER_RATE, 90.0f);

/**
 * Steering controller P gain
 *
 * Proportional gain for steering angle control
 *
 * @min 0.1
 * @max 10.0
 * @decimal 2
 * @group Wheel Loader Chassis
 */
PARAM_DEFINE_FLOAT(WL_STEER_P_GAIN, 2.0f);

/**
 * Steering controller I gain
 *
 * Integral gain for steering angle control
 *
 * @min 0.0
 * @max 5.0
 * @decimal 3
 * @group Wheel Loader Chassis
 */
PARAM_DEFINE_FLOAT(WL_STEER_I_GAIN, 0.1f);

/**
 * Steering controller D gain
 *
 * Derivative gain for steering angle control
 *
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @group Wheel Loader Chassis
 */
PARAM_DEFINE_FLOAT(WL_STEER_D_GAIN, 0.05f);

/**
 * Default boom movement speed
 *
 * Default speed for boom movements in degrees per second
 *
 * @unit deg/s
 * @min 1.0
 * @max 45.0
 * @decimal 1
 * @group Wheel Loader Boom
 */
PARAM_DEFINE_FLOAT(WL_BOOM_SPEED, 15.0f);

/**
 * Minimum boom angle
 *
 * Minimum allowed boom angle in degrees
 *
 * @unit deg
 * @min -30.0
 * @max 30.0
 * @decimal 1
 * @group Wheel Loader Boom
 */
PARAM_DEFINE_FLOAT(WL_BOOM_MIN_ANGLE, -10.0f);

/**
 * Maximum boom angle
 *
 * Maximum allowed boom angle in degrees
 *
 * @unit deg
 * @min 30.0
 * @max 90.0
 * @decimal 1
 * @group Wheel Loader Boom
 */
PARAM_DEFINE_FLOAT(WL_BOOM_MAX_ANGLE, 75.0f);

/**
 * Boom loading position angle
 *
 * Boom angle in degrees for material loading operations
 *
 * @unit deg
 * @min -20.0
 * @max 40.0
 * @decimal 1
 * @group Wheel Loader Boom
 */
PARAM_DEFINE_FLOAT(WL_BOOM_LOAD_ANGLE, 10.0f);

/**
 * Boom dumping position angle
 *
 * Boom angle in degrees for material dumping operations
 *
 * @unit deg
 * @min 30.0
 * @max 80.0
 * @decimal 1
 * @group Wheel Loader Boom
 */
PARAM_DEFINE_FLOAT(WL_BOOM_DUMP_ANGLE, 50.0f);

/**
 * Default bucket movement speed
 *
 * Default speed for bucket movements in degrees per second
 *
 * @unit deg/s
 * @min 1.0
 * @max 90.0
 * @decimal 1
 * @group Wheel Loader Bucket
 */
PARAM_DEFINE_FLOAT(WL_BUCKET_SPEED, 30.0f);

/**
 * Minimum bucket angle
 *
 * Minimum allowed bucket angle in degrees (curl position)
 *
 * @unit deg
 * @min -45.0
 * @max 0.0
 * @decimal 1
 * @group Wheel Loader Bucket
 */
PARAM_DEFINE_FLOAT(WL_BUCKET_MIN_ANGLE, -30.0f);

/**
 * Maximum bucket angle
 *
 * Maximum allowed bucket angle in degrees (dump position)
 *
 * @unit deg
 * @min 45.0
 * @max 120.0
 * @decimal 1
 * @group Wheel Loader Bucket
 */
PARAM_DEFINE_FLOAT(WL_BUCKET_MAX_ANGLE, 90.0f);

/**
 * Bucket curl position angle
 *
 * Bucket angle in degrees for material curling/scooping
 *
 * @unit deg
 * @min -45.0
 * @max 0.0
 * @decimal 1
 * @group Wheel Loader Bucket
 */
PARAM_DEFINE_FLOAT(WL_BUCKET_CURL_ANGLE, -20.0f);

/**
 * Bucket dump position angle
 *
 * Bucket angle in degrees for material dumping
 *
 * @unit deg
 * @min 45.0
 * @max 120.0
 * @decimal 1
 * @group Wheel Loader Bucket
 */
PARAM_DEFINE_FLOAT(WL_BUCKET_DUMP_ANGLE, 80.0f);

/**
 * Autonomous approach distance
 *
 * Distance in meters to approach material pile during autonomous loading
 *
 * @unit m
 * @min 0.5
 * @max 5.0
 * @decimal 1
 * @group Wheel Loader Autonomous
 */
PARAM_DEFINE_FLOAT(WL_AUTO_APPROACH_DIST, 2.0f);

/**
 * Autonomous loading time
 *
 * Time in seconds to spend loading material in autonomous mode
 *
 * @unit s
 * @min 2.0
 * @max 20.0
 * @decimal 1
 * @group Wheel Loader Autonomous
 */
PARAM_DEFINE_FLOAT(WL_AUTO_LOAD_TIME, 8.0f);

/**
 * Autonomous dumping time
 *
 * Time in seconds to spend dumping material in autonomous mode
 *
 * @unit s
 * @min 2.0
 * @max 15.0
 * @decimal 1
 * @group Wheel Loader Autonomous
 */
PARAM_DEFINE_FLOAT(WL_AUTO_DUMP_TIME, 5.0f);

/**
 * Autonomous cycle count
 *
 * Number of load/dump cycles to perform in autonomous mode (0 = infinite)
 *
 * @min 0
 * @max 100
 * @decimal 0
 * @group Wheel Loader Autonomous
 */
PARAM_DEFINE_INT32(WL_AUTO_CYCLE_COUNT, 5);

/**
 * Emergency brake deceleration
 *
 * Maximum deceleration in m/s² during emergency braking
 *
 * @unit m/s²
 * @min 1.0
 * @max 10.0
 * @decimal 1
 * @group Wheel Loader Safety
 */
PARAM_DEFINE_FLOAT(WL_EMERG_BRAKE_DECEL, 3.0f);

/**
 * Maximum tilt angle
 *
 * Maximum allowed vehicle tilt angle in degrees before triggering safety stop
 *
 * @unit deg
 * @min 10.0
 * @max 45.0
 * @decimal 1
 * @group Wheel Loader Safety
 */
PARAM_DEFINE_FLOAT(WL_MAX_TILT_ANGLE, 25.0f);

/**
 * Health check frequency
 *
 * Frequency in Hz for subsystem health monitoring
 *
 * @unit Hz
 * @min 1.0
 * @max 20.0
 * @decimal 1
 * @group Wheel Loader Safety
 */
PARAM_DEFINE_FLOAT(WL_HEALTH_CHECK_FREQ, 10.0f);
