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
 * H-bridge channel for boom motor
 *
 * DRV8701 H-bridge channel (0 or 1) used for the boom actuator motor.
 *
 * @unit channel
 * @min 0
 * @max 1
 * @decimal 0
 * @reboot_required true
 * @group Boom Control
 */
PARAM_DEFINE_INT32(BOOM_HBRIDGE_CH, 0);

/**
 * Boom angle sensor instance
 *
 * AS5600 magnetic encoder instance for boom angle measurement.
 *
 * @unit instance
 * @min 0
 * @max 3
 * @decimal 0
 * @reboot_required true
 * @group Boom Control
 */
PARAM_DEFINE_INT32(BOOM_ANGLE_SENS, 0);

/**
 * Boom angle minimum limit
 *
 * Minimum allowed boom angle in degrees.
 * Prevents boom from moving below ground level.
 *
 * @unit deg
 * @min -90.0
 * @max 90.0
 * @decimal 1
 * @group Boom Control
 */
PARAM_DEFINE_FLOAT(BOOM_ANGLE_MIN, -10.0f);

/**
 * Boom angle maximum limit
 *
 * Maximum allowed boom angle in degrees.
 * Prevents boom from exceeding mechanical limits.
 *
 * @unit deg
 * @min -90.0
 * @max 90.0
 * @decimal 1
 * @group Boom Control
 */
PARAM_DEFINE_FLOAT(BOOM_ANGLE_MAX, 75.0f);

/**
 * Boom ground position angle
 *
 * Boom angle in degrees when bucket is at ground level.
 *
 * @unit deg
 * @min -90.0
 * @max 90.0
 * @decimal 1
 * @group Boom Control
 */
PARAM_DEFINE_FLOAT(BOOM_POS_GROUND, -5.0f);

/**
 * Boom carry position angle
 *
 * Boom angle in degrees for standard carry position.
 *
 * @unit deg
 * @min -90.0
 * @max 90.0
 * @decimal 1
 * @group Boom Control
 */
PARAM_DEFINE_FLOAT(BOOM_POS_CARRY, 30.0f);

/**
 * Boom maximum height position angle
 *
 * Boom angle in degrees for maximum dump height.
 *
 * @unit deg
 * @min -90.0
 * @max 90.0
 * @decimal 1
 * @group Boom Control
 */
PARAM_DEFINE_FLOAT(BOOM_POS_MAX, 70.0f);

/**
 * Boom PID proportional gain
 *
 * Proportional gain for boom angle control.
 * Higher values increase responsiveness but may cause oscillation.
 *
 * @unit %/deg
 * @min 0.0
 * @max 20.0
 * @decimal 3
 * @group Boom Control
 */
PARAM_DEFINE_FLOAT(BOOM_PID_P, 2.5f);

/**
 * Boom PID integral gain
 *
 * Integral gain for boom angle control.
 * Helps eliminate steady-state error.
 *
 * @unit %/(deg*s)
 * @min 0.0
 * @max 5.0
 * @decimal 3
 * @group Boom Control
 */
PARAM_DEFINE_FLOAT(BOOM_PID_I, 0.1f);

/**
 * Boom PID derivative gain
 *
 * Derivative gain for boom angle control.
 * Provides damping and reduces overshoot.
 *
 * @unit %*s/deg
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @group Boom Control
 */
PARAM_DEFINE_FLOAT(BOOM_PID_D, 0.05f);

/**
 * Boom maximum motor output
 *
 * Maximum motor output percentage for boom control.
 * Limits motor current to protect mechanical components.
 *
 * @unit %
 * @min 10.0
 * @max 100.0
 * @decimal 1
 * @group Boom Control
 */
PARAM_DEFINE_FLOAT(BOOM_MAX_OUTPUT, 80.0f);

/**
 * Boom motion acceleration
 *
 * Maximum acceleration for boom motion planning.
 * Controls smoothness of boom movements.
 *
 * @unit deg/s²
 * @min 5.0
 * @max 100.0
 * @decimal 1
 * @group Boom Control
 */
PARAM_DEFINE_FLOAT(BOOM_ACCEL, 20.0f);

/**
 * Boom motion velocity
 *
 * Maximum velocity for boom motion planning.
 * Controls speed of boom movements.
 *
 * @unit deg/s
 * @min 5.0
 * @max 50.0
 * @decimal 1
 * @group Boom Control
 */
PARAM_DEFINE_FLOAT(BOOM_VEL, 15.0f);

/**
 * Boom motion jerk
 *
 * Maximum jerk for boom motion planning.
 * Controls acceleration smoothness.
 *
 * @unit deg/s³
 * @min 10.0
 * @max 500.0
 * @decimal 1
 * @group Boom Control
 */
PARAM_DEFINE_FLOAT(BOOM_JERK, 100.0f);

/**
 * Boom deadband
 *
 * Position deadband for boom control.
 * Prevents oscillation around target position.
 *
 * @unit deg
 * @min 0.1
 * @max 5.0
 * @decimal 2
 * @group Boom Control
 */
PARAM_DEFINE_FLOAT(BOOM_DEADBAND, 0.5f);

/**
 * Boom pivot to actuator base distance
 *
 * Distance from boom pivot point to actuator base mount.
 * Used for kinematic calculations.
 *
 * @unit mm
 * @min 100.0
 * @max 2000.0
 * @decimal 1
 * @group Boom Control
 */
PARAM_DEFINE_FLOAT(BOOM_KIN_L1, 500.0f);

/**
 * Boom pivot to actuator attachment distance
 *
 * Distance from boom pivot point to actuator attachment on boom.
 * Used for kinematic calculations.
 *
 * @unit mm
 * @min 100.0
 * @max 2000.0
 * @decimal 1
 * @group Boom Control
 */
PARAM_DEFINE_FLOAT(BOOM_KIN_L2, 300.0f);

/**
 * Boom actuator base angle
 *
 * Angle of actuator base mount from horizontal.
 * Used for kinematic calculations.
 *
 * @unit deg
 * @min -90.0
 * @max 90.0
 * @decimal 1
 * @group Boom Control
 */
PARAM_DEFINE_FLOAT(BOOM_KIN_ANG, 15.0f);

/**
 * Boom actuator minimum length
 *
 * Minimum actuator length for mechanical limits.
 * Used for kinematic calculations.
 *
 * @unit mm
 * @min 100.0
 * @max 1000.0
 * @decimal 1
 * @group Boom Control
 */
PARAM_DEFINE_FLOAT(BOOM_ACT_MIN, 200.0f);

/**
 * Boom actuator maximum length
 *
 * Maximum actuator length for mechanical limits.
 * Used for kinematic calculations.
 *
 * @unit mm
 * @min 200.0
 * @max 1500.0
 * @decimal 1
 * @group Boom Control
 */
PARAM_DEFINE_FLOAT(BOOM_ACT_MAX, 800.0f);
