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
 * Wheel loader controller maximum speed
 *
 * Maximum allowed vehicle speed for safety limiting.
 *
 * @unit m/s
 * @min 0.1
 * @max 20.0
 * @decimal 1
 * @group Wheel Loader Controller
 */
PARAM_DEFINE_FLOAT(WLC_MAX_SPEED, 5.0f);

/**
 * Wheel loader controller maximum acceleration
 *
 * Maximum allowed acceleration for safety limiting.
 *
 * @unit
 * @min 0.1
 * @max 10.0
 * @decimal 1
 * @group Wheel Loader Controller
 */
PARAM_DEFINE_FLOAT(WLC_MAX_ACCEL, 2.0f);

/**
 * Command timeout
 *
 * Maximum time without receiving commands before entering safe mode.
 *
 * @unit s
 * @min 0.1
 * @max 5.0
 * @decimal 1
 * @group Wheel Loader Controller
 */
PARAM_DEFINE_FLOAT(WLC_CMD_TIMEOUT, 0.5f);

/**
 * Health monitoring timeout
 *
 * Maximum time without receiving status from subsystems before marking as unhealthy.
 *
 * @unit s
 * @min 0.1
 * @max 5.0
 * @decimal 1
 * @group Wheel Loader Controller
 */
PARAM_DEFINE_FLOAT(WLC_HEALTH_TO, 1.0f);

/**
 * Emergency stop enable
 *
 * Enable emergency stop functionality.
 *
 * @boolean
 * @group Wheel Loader Controller
 */
PARAM_DEFINE_INT32(WLC_ESTOP_EN, 1);

/**
 * Diagnostic mode enable
 *
 * Enable diagnostic output and verbose logging.
 *
 * @boolean
 * @group Wheel Loader Controller
 */
PARAM_DEFINE_INT32(WLC_DIAG_EN, 0);

/**
 * Control loop rate
 *
 * Main control loop execution rate.
 *
 * @unit
 * @min 10.0
 * @max 200.0
 * @decimal 1
 * @group Wheel Loader Controller
 */
PARAM_DEFINE_FLOAT(WLC_CTRL_RATE, 50.0f);

/**
 * Safe acceleration limit
 *
 * Conservative acceleration limit used during autonomous operation.
 *
 * @unit
 * @min 0.1
 * @max 5.0
 * @decimal 1
 * @group Wheel Loader Controller
 */
PARAM_DEFINE_FLOAT(WLC_SAFE_ACCEL, 1.0f);

/**
 * Safe speed limit
 *
 * Conservative speed limit used during autonomous operation.
 *
 * @unit m/s
 * @min 0.1
 * @max 10.0
 * @decimal 1
 * @group Wheel Loader Controller
 */
PARAM_DEFINE_FLOAT(WLC_SAFE_SPEED, 2.0f);
