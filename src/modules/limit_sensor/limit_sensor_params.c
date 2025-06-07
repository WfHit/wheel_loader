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
 * Limit sensor module parameters.
 */

/**
 * Limit sensor system enable
 *
 * Enable the limit sensor system globally.
 * When disabled, no limit sensor instances will be started.
 *
 * @boolean
 * @group Limit Sensors
 */
PARAM_DEFINE_INT32(LS_SYS_ENABLE, 0);

/**
 * Limit sensor polling rate
 *
 * Rate at which GPIO pins are polled for state changes in Hz.
 * Higher rates provide better responsiveness but use more CPU.
 *
 * @min 10
 * @max 1000
 * @decimal 0
 * @increment 10
 * @group Limit Sensors
 */
PARAM_DEFINE_INT32(LS_POLL_RATE, 200);

/**
 * Limit sensor debounce time
 *
 * Debounce time for limit switch GPIO inputs in microseconds.
 * Helps filter out electrical noise and switch bounce.
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
 * Limit sensor redundancy enable
 *
 * Enable dual-switch redundancy checking globally.
 * When enabled, instances can use two GPIO pins for fault detection.
 *
 * @boolean
 * @group Limit Sensors
 */
PARAM_DEFINE_INT32(LS_REDUNDANCY_EN, 1);

/**
 * Limit sensor diagnostics enable
 *
 * Enable diagnostic features including activation counting and health monitoring.
 *
 * @boolean
 * @group Limit Sensors
 */
PARAM_DEFINE_INT32(LS_DIAG_ENABLE, 1);

/**
 * Limit Sensor 0 Primary GPIO Pin
 *
 * GPIO pin number for limit sensor instance 0 primary switch.
 * Instance 0 is typically used for bucket minimum (dump) position.
 * Set to 0 to disable this limit sensor instance.
 *
 * @min 0
 * @max 255
 * @decimal 0
 * @increment 1
 * @group Limit Sensors
 */
PARAM_DEFINE_INT32(LS0_GPIO_1, 0);

/**
 * Limit Sensor 0 Secondary GPIO Pin
 *
 * GPIO pin number for limit sensor instance 0 secondary switch (redundancy).
 * Used for dual-switch redundancy on bucket minimum position.
 * Set to 0 to disable redundant switch for this instance.
 *
 * @min 0
 * @max 255
 * @decimal 0
 * @increment 1
 * @group Limit Sensors
 */
PARAM_DEFINE_INT32(LS0_GPIO_2, 0);

/**
 * Limit Sensor 0 Type
 *
 * Type of limit sensor for instance 0.
 * 0=Bucket Min (dump), 1=Bucket Max (load), 2=Boom Min, 3=Boom Max, 4=Steering Min, 5=Steering Max
 *
 * @min 0
 * @max 5
 * @decimal 0
 * @increment 1
 * @group Limit Sensors
 * @value 0 Bucket Minimum (Dump Position)
 * @value 1 Bucket Maximum (Load Position)
 * @value 2 Boom Minimum (Lower Position)
 * @value 3 Boom Maximum (Upper Position)
 * @value 4 Steering Minimum (Left Limit)
 * @value 5 Steering Maximum (Right Limit)
 */
PARAM_DEFINE_INT32(LS0_TYPE, 0);

/**
 * Limit Sensor 0 Invert Logic
 *
 * Invert the logic of limit sensor instance 0.
 * Set to true if the sensor is active low (switch grounds the pin when activated).
 *
 * @boolean
 * @group Limit Sensors
 */
PARAM_DEFINE_INT32(LS0_INVERT, 0);

/**
 * Limit Sensor 0 Enable Redundancy
 *
 * Enable redundant switch checking for limit sensor instance 0.
 * Requires both LS0_GPIO_1 and LS0_GPIO_2 to be configured.
 * Provides fault detection when switches disagree.
 *
 * @boolean
 * @group Limit Sensors
 */
PARAM_DEFINE_INT32(LS0_REDUNDANCY, 0);

/**
 * Limit Sensor 0 Enable
 *
 * Enable limit sensor instance 0 (bucket minimum position).
 *
 * @boolean
 * @group Limit Sensors
 */
PARAM_DEFINE_INT32(LS0_ENABLE, 0);

/**
 * Limit Sensor 1 Primary GPIO Pin
 *
 * GPIO pin number for limit sensor instance 1 primary switch.
 * Instance 1 is typically used for bucket maximum (load) position.
 * Set to 0 to disable this limit sensor instance.
 *
 * @min 0
 * @max 255
 * @decimal 0
 * @increment 1
 * @group Limit Sensors
 */
PARAM_DEFINE_INT32(LS1_GPIO_1, 0);

/**
 * Limit Sensor 1 Secondary GPIO Pin
 *
 * GPIO pin number for limit sensor instance 1 secondary switch (redundancy).
 * Used for dual-switch redundancy on bucket maximum position.
 * Set to 0 to disable redundant switch for this instance.
 *
 * @min 0
 * @max 255
 * @decimal 0
 * @increment 1
 * @group Limit Sensors
 */
PARAM_DEFINE_INT32(LS1_GPIO_2, 0);

/**
 * Limit Sensor 1 Type
 *
 * Type of limit sensor for instance 1.
 * 0=Bucket Min (dump), 1=Bucket Max (load), 2=Boom Min, 3=Boom Max, 4=Steering Min, 5=Steering Max
 *
 * @min 0
 * @max 5
 * @decimal 0
 * @increment 1
 * @group Limit Sensors
 * @value 0 Bucket Minimum (Dump Position)
 * @value 1 Bucket Maximum (Load Position)
 * @value 2 Boom Minimum (Lower Position)
 * @value 3 Boom Maximum (Upper Position)
 * @value 4 Steering Minimum (Left Limit)
 * @value 5 Steering Maximum (Right Limit)
 */
PARAM_DEFINE_INT32(LS1_TYPE, 1);

/**
 * Limit Sensor 1 Invert Logic
 *
 * Invert the logic of limit sensor instance 1.
 * Set to true if the sensor is active low (switch grounds the pin when activated).
 *
 * @boolean
 * @group Limit Sensors
 */
PARAM_DEFINE_INT32(LS1_INVERT, 0);

/**
 * Limit Sensor 1 Enable Redundancy
 *
 * Enable redundant switch checking for limit sensor instance 1.
 * Requires both LS1_GPIO_1 and LS1_GPIO_2 to be configured.
 * Provides fault detection when switches disagree.
 *
 * @boolean
 * @group Limit Sensors
 */
PARAM_DEFINE_INT32(LS1_REDUNDANCY, 0);

/**
 * Limit Sensor 1 Enable
 *
 * Enable limit sensor instance 1 (bucket maximum position).
 *
 * @boolean
 * @group Limit Sensors
 */
PARAM_DEFINE_INT32(LS1_ENABLE, 0);

/**
 * Limit Sensor 2 Primary GPIO Pin
 *
 * GPIO pin number for limit sensor instance 2 primary switch.
 * Set to 0 to disable this limit sensor instance.
 *
 * @min 0
 * @max 255
 * @decimal 0
 * @increment 1
 * @group Limit Sensors
 */
PARAM_DEFINE_INT32(LS2_GPIO_1, 0);

/**
 * Limit Sensor 2 Secondary GPIO Pin
 *
 * GPIO pin number for limit sensor instance 2 secondary switch (redundancy).
 * Set to 0 to disable redundant switch for this instance.
 *
 * @min 0
 * @max 255
 * @decimal 0
 * @increment 1
 * @group Limit Sensors
 */
PARAM_DEFINE_INT32(LS2_GPIO_2, 0);

/**
 * Limit Sensor 2 Type
 *
 * Type of limit sensor for instance 2.
 * 0=Bucket Min, 1=Bucket Max, 2=Boom Min, 3=Boom Max, 4=Steering Min, 5=Steering Max
 *
 * @min 0
 * @max 5
 * @decimal 0
 * @increment 1
 * @group Limit Sensors
 */
PARAM_DEFINE_INT32(LS2_TYPE, 2);

/**
 * Limit Sensor 2 Invert Logic
 *
 * Invert the logic of limit sensor instance 2.
 * Set to true if the sensor is active low.
 *
 * @boolean
 * @group Limit Sensors
 */
PARAM_DEFINE_INT32(LS2_INVERT, 0);

/**
 * Limit Sensor 2 Enable Redundancy
 *
 * Enable redundant switch checking for limit sensor instance 2.
 * Requires both LS2_GPIO_1 and LS2_GPIO_2 to be configured.
 *
 * @boolean
 * @group Limit Sensors
 */
PARAM_DEFINE_INT32(LS2_REDUNDANCY, 0);

/**
 * Limit Sensor 2 Enable
 *
 * Enable limit sensor instance 2.
 *
 * @boolean
 * @group Limit Sensors
 */
PARAM_DEFINE_INT32(LS2_ENABLE, 0);

/**
 * Limit Sensor 3 Primary GPIO Pin
 *
 * GPIO pin number for limit sensor instance 3 primary switch.
 * Set to 0 to disable this limit sensor instance.
 *
 * @min 0
 * @max 255
 * @decimal 0
 * @increment 1
 * @group Limit Sensors
 */
PARAM_DEFINE_INT32(LS3_GPIO_1, 0);

/**
 * Limit Sensor 3 Secondary GPIO Pin
 *
 * GPIO pin number for limit sensor instance 3 secondary switch (redundancy).
 * Set to 0 to disable redundant switch for this instance.
 *
 * @min 0
 * @max 255
 * @decimal 0
 * @increment 1
 * @group Limit Sensors
 */
PARAM_DEFINE_INT32(LS3_GPIO_2, 0);

/**
 * Limit Sensor 3 Type
 *
 * Type of limit sensor for instance 3.
 * 0=Bucket Min, 1=Bucket Max, 2=Boom Min, 3=Boom Max, 4=Steering Min, 5=Steering Max
 *
 * @min 0
 * @max 5
 * @decimal 0
 * @increment 1
 * @group Limit Sensors
 */
PARAM_DEFINE_INT32(LS3_TYPE, 3);

/**
 * Limit Sensor 3 Invert Logic
 *
 * Invert the logic of limit sensor instance 3.
 * Set to true if the sensor is active low.
 *
 * @boolean
 * @group Limit Sensors
 */
PARAM_DEFINE_INT32(LS3_INVERT, 0);

/**
 * Limit Sensor 3 Enable Redundancy
 *
 * Enable redundant switch checking for limit sensor instance 3.
 * Requires both LS3_GPIO_1 and LS3_GPIO_2 to be configured.
 *
 * @boolean
 * @group Limit Sensors
 */
PARAM_DEFINE_INT32(LS3_REDUNDANCY, 0);

/**
 * Limit Sensor 3 Enable
 *
 * Enable limit sensor instance 3.
 *
 * @boolean
 * @group Limit Sensors
 */
PARAM_DEFINE_INT32(LS3_ENABLE, 0);
