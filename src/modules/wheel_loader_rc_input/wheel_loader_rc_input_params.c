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
 * Wheel loader RC maximum wheel speed
 *
 * Maximum wheel speed when using RC control.
 *
 * @unit rad/s
 * @min 0.5
 * @max 10.0
 * @decimal 1
 * @group Wheel Loader RC Input
 */
PARAM_DEFINE_FLOAT(WL_RC_MAX_SPEED, 5.0f);

/**
 * Wheel loader RC maximum steering angle
 *
 * Maximum steering angle when using RC control.
 *
 * @unit rad
 * @min 0.1
 * @max 1.5
 * @decimal 2
 * @group Wheel Loader RC Input
 */
PARAM_DEFINE_FLOAT(WL_RC_MAX_STEER, 0.7f);

/**
 * Wheel loader RC maximum boom rate
 *
 * Maximum boom movement rate when using RC control.
 *
 * @unit rad/s
 * @min 0.1
 * @max 2.0
 * @decimal 1
 * @group Wheel Loader RC Input
 */
PARAM_DEFINE_FLOAT(WL_RC_MAX_BOOM, 0.5f);

/**
 * Wheel loader RC maximum bucket rate
 *
 * Maximum bucket movement rate when using RC control.
 *
 * @unit rad/s
 * @min 0.1
 * @max 3.0
 * @decimal 1
 * @group Wheel Loader RC Input
 */
PARAM_DEFINE_FLOAT(WL_RC_MAX_BUCKET, 1.0f);

/**
 * Wheel loader RC deadzone
 *
 * RC input deadzone around center position.
 *
 * @min 0.0
 * @max 0.2
 * @decimal 2
 * @group Wheel Loader RC Input
 */
PARAM_DEFINE_FLOAT(WL_RC_DEADZONE, 0.05f);

/**
 * Wheel loader RC timeout
 *
 * Time after which RC input is considered lost and failsafe is activated.
 *
 * @unit s
 * @min 0.1
 * @max 5.0
 * @decimal 1
 * @group Wheel Loader RC Input
 */
PARAM_DEFINE_FLOAT(WL_RC_TIMEOUT, 1.0f);

/**
 * Wheel loader RC enable
 *
 * Enable RC input processing for wheel loader control.
 *
 * @boolean
 * @group Wheel Loader RC Input
 */
PARAM_DEFINE_INT32(WL_RC_ENABLE, 1);