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
 * Maximum steering angle
 *
 * Maximum allowed steering angle in radians for the articulated wheel loader.
 * Default corresponds to ±45 degrees (±0.785 rad).
 *
 * @unit rad
 * @min 0.1
 * @max 1.57
 * @decimal 3
 * @group Steering Controller
 */
PARAM_DEFINE_FLOAT(STEER_MAX_ANGLE, 0.785398f);

/**
 * Maximum steering rate
 *
 * Maximum steering angular velocity in rad/s.
 * Default corresponds to 60°/s (1.047 rad/s).
 *
 * @unit rad/s
 * @min 0.1
 * @max 3.14
 * @decimal 3
 * @group Steering Controller
 */
PARAM_DEFINE_FLOAT(STEER_MAX_RATE, 1.047198f);

/**
 * Steering deadband
 *
 * Deadband around target position to reduce jitter.
 * Default corresponds to 0.5 degrees (0.00873 rad).
 *
 * @unit rad
 * @min 0.0
 * @max 0.1
 * @decimal 5
 * @group Steering Controller
 */
PARAM_DEFINE_FLOAT(STEER_DEADBAND, 0.00873f);

/**
 * Steering trim offset
 *
 * Constant offset added to steering commands for calibration.
 * Used to correct for servo mounting or mechanical misalignment.
 *
 * @unit rad
 * @min -0.2
 * @max 0.2
 * @decimal 4
 * @group Steering Controller
 */
PARAM_DEFINE_FLOAT(STEER_TRIM, 0.0f);

/**
 * Reverse steering direction
 *
 * Invert steering direction if servo is mounted backwards
 * or steering geometry requires reversal.
 *
 * @boolean
 * @group Steering Controller
 */
PARAM_DEFINE_INT32(STEER_REVERSE, 0);

/**
 * Enable slip compensation
 *
 * Use PredictiveTraction data for slip-based steering compensation.
 * Helps maintain vehicle control on slippery surfaces.
 *
 * @boolean
 * @group Steering Controller
 */
PARAM_DEFINE_INT32(STEER_SLIP_COMP_EN, 1);

/**
 * Slip compensation gain
 *
 * Gain for slip-based steering compensation.
 * Higher values provide more aggressive slip correction.
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @group Steering Controller
 */
PARAM_DEFINE_FLOAT(STEER_SLIP_COMP_GAIN, 0.5f);

/**
 * Maximum slip compensation
 *
 * Maximum slip compensation angle in radians.
 * Limits the maximum steering correction due to slip.
 * Default corresponds to 10 degrees (0.174533 rad).
 *
 * @unit rad
 * @min 0.0
 * @max 0.5
 * @decimal 4
 * @group Steering Controller
 */
PARAM_DEFINE_FLOAT(STEER_SLIP_COMP_MAX, 0.174533f);

/**
 * Feedforward gain
 *
 * Feedforward gain for improved dynamic response.
 * Helps reduce tracking lag during rapid steering maneuvers.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group Steering Controller
 */
PARAM_DEFINE_FLOAT(STEER_FF_GAIN, 0.2f);

/**
 * Speed-dependent feedforward scaling
 *
 * Scaling factor for speed-dependent feedforward control.
 * Increases feedforward gain at higher vehicle speeds.
 *
 * @min 0.0
 * @max 0.5
 * @decimal 2
 * @group Steering Controller
 */
PARAM_DEFINE_FLOAT(STEER_FF_SPEED_SCALE, 0.1f);

/**
 * Servo minimum PWM
 *
 * Minimum PWM value for servo (deprecated - kept for compatibility).
 * Used with legacy PWM-based servo control.
 *
 * @unit us
 * @min 500.0
 * @max 1500.0
 * @decimal 0
 * @group Steering Controller
 */
PARAM_DEFINE_FLOAT(STEER_PWM_MIN, 1000.0f);

/**
 * Servo maximum PWM
 *
 * Maximum PWM value for servo (deprecated - kept for compatibility).
 * Used with legacy PWM-based servo control.
 *
 * @unit us
 * @min 1500.0
 * @max 2500.0
 * @decimal 0
 * @group Steering Controller
 */
PARAM_DEFINE_FLOAT(STEER_PWM_MAX, 2000.0f);

/**
 * Servo current limit
 *
 * Maximum current limit for ST3125 servo.
 * Protects servo from overcurrent damage.
 *
 * @unit A
 * @min 0.5
 * @max 5.0
 * @decimal 1
 * @group Steering Controller
 */
PARAM_DEFINE_FLOAT(STEER_CURR_LIMIT, 2.0f);

/**
 * Enable limit sensors
 *
 * Enable limit sensor monitoring for steering position limits.
 * When enabled, the controller will stop at limit positions to prevent damage.
 *
 * @boolean
 * @group Steering Controller
 */
PARAM_DEFINE_INT32(STEER_LIMIT_EN, 1);

/**
 * Left limit sensor instance
 *
 * Instance ID of the left steering limit sensor.
 * Must match the configured instance in the limit sensor module.
 *
 * @min 0
 * @max 255
 * @group Steering Controller
 */
PARAM_DEFINE_INT32(STEER_LIMIT_LEFT_IDX, 0);

/**
 * Right limit sensor instance
 *
 * Instance ID of the right steering limit sensor.
 * Must match the configured instance in the limit sensor module.
 *
 * @min 0
 * @max 255
 * @group Steering Controller
 */
PARAM_DEFINE_INT32(STEER_LIMIT_RIGHT_IDX, 1);

/**
 * Limit sensor margin
 *
 * Safety margin in radians before limit sensor activation.
 * Controller will respect this margin to avoid hitting hard limits.
 *
 * @unit rad
 * @min 0.0
 * @max 0.349
 * @decimal 3
 * @group Steering Controller
 */
PARAM_DEFINE_FLOAT(STEER_LIMIT_MARGIN, 0.087f);

/**
 * Enable safety manager
 *
 * Enable safety manager for fault detection and emergency stop functionality.
 * Monitors servo faults, sensor faults, and safety violations.
 *
 * @boolean
 * @group Steering Controller
 */
PARAM_DEFINE_INT32(STEER_SAFETY_EN, 1);

/**
 * Safety position
 *
 * Safe steering position in radians (typically center position).
 * Controller will move to this position during safety violations.
 *
 * @unit rad
 * @min -0.785
 * @max 0.785
 * @decimal 3
 * @group Steering Controller
 */
PARAM_DEFINE_FLOAT(STEER_SAFE_POS, 0.0f);

/**
 * Fault timeout
 *
 * Timeout in milliseconds before clearing safety violations.
 * Safety violations are cleared after this timeout if no active faults exist.
 *
 * @unit ms
 * @min 1000
 * @max 30000
 * @decimal 0
 * @group Steering Controller
 */
PARAM_DEFINE_FLOAT(STEER_FAULT_TIMEOUT, 5000.0f);

/**
 * Maximum violations
 *
 * Maximum number of safety violations before triggering emergency stop.
 * After this many violations, manual intervention is required to clear.
 *
 * @min 1
 * @max 100
 * @group Steering Controller
 */
PARAM_DEFINE_INT32(STEER_MAX_VIOLATIONS, 10);
