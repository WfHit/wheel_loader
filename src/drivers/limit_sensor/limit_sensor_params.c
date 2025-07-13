/****************************************************************************
 *
 *   Copyright (c) 2024 PX4 Development Team. All rights reserved.
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
 * @file limit_sensor_params.c
 * Limit sensor module global parameters.
 *
 * NOTE: Hardware configuration (GPIO pins, functions, redundancy) is now
 * defined in board_config.h and board-specific configuration files.
 * These parameters only control runtime behavior.
 */

/**
 * Limit sensor polling rate
 *
 * Rate at which GPIO pins are polled for state changes in Hz.
 * Higher rates provide better responsiveness but use more CPU.
 * This setting affects all limit sensor instances.
 *
 * @min 10
 * @max 1000
 * @decimal 0
 * @increment 10
 * @unit Hz
 * @group Limit Sensors
 */
PARAM_DEFINE_INT32(LS_POLL_RATE, 200);

/**
 * Limit sensor debounce time
 *
 * Debounce time for limit switch GPIO inputs in microseconds.
 * Helps filter out electrical noise and switch bounce.
 * This setting affects all limit sensor instances.
 *
 * @min 1000
 * @max 100000
 * @decimal 0
 * @increment 1000
 * @unit us
 * @group Limit Sensors
 */
PARAM_DEFINE_INT32(LS_DEBOUNCE_US, 10000);

/**
 * Limit sensor diagnostics enable
 *
 * Enable diagnostic features including activation counting and health monitoring.
 * This setting affects all limit sensor instances.
 *
 * @boolean
 * @group Limit Sensors
 */
PARAM_DEFINE_INT32(LS_DIAG_ENABLE, 1);

/**
 * Limit sensor system enable
 *
 * Master enable switch for the limit sensor system.
 * When disabled, no limit sensor instances will be started.
 *
 * @boolean
 * @group Limit Sensors
 */
PARAM_DEFINE_INT32(LIMIT_SENSOR_EN, 0);

/**
 * Limit sensor instance 0 enable
 *
 * Enable limit sensor instance 0.
 * Function depends on board configuration.
 *
 * @boolean
 * @group Limit Sensors
 */
PARAM_DEFINE_INT32(LS0_ENABLE, 0);

/**
 * Limit sensor instance 1 enable
 *
 * Enable limit sensor instance 1.
 * Function depends on board configuration.
 *
 * @boolean
 * @group Limit Sensors
 */
PARAM_DEFINE_INT32(LS1_ENABLE, 0);

/**
 * Limit sensor instance 2 enable
 *
 * Enable limit sensor instance 2.
 * Function depends on board configuration.
 *
 * @boolean
 * @group Limit Sensors
 */
PARAM_DEFINE_INT32(LS2_ENABLE, 0);

/**
 * Limit sensor instance 3 enable
 *
 * Enable limit sensor instance 3.
 * Function depends on board configuration.
 *
 * @boolean
 * @group Limit Sensors
 */
PARAM_DEFINE_INT32(LS3_ENABLE, 0);

/**
 * Limit sensor instance 4 enable
 *
 * Enable limit sensor instance 4.
 * Function depends on board configuration.
 *
 * @boolean
 * @group Limit Sensors
 */
PARAM_DEFINE_INT32(LS4_ENABLE, 0);

/**
 * Limit sensor instance 5 enable
 *
 * Enable limit sensor instance 5.
 * Function depends on board configuration.
 *
 * @boolean
 * @group Limit Sensors
 */
PARAM_DEFINE_INT32(LS5_ENABLE, 0);

/**
 * Limit sensor instance 6 enable
 *
 * Enable limit sensor instance 6.
 * Function depends on board configuration.
 *
 * @boolean
 * @group Limit Sensors
 */
PARAM_DEFINE_INT32(LS6_ENABLE, 0);

/**
 * Limit sensor instance 7 enable
 *
 * Enable limit sensor instance 7.
 * Function depends on board configuration.
 *
 * @boolean
 * @group Limit Sensors
 */
PARAM_DEFINE_INT32(LS7_ENABLE, 0);

