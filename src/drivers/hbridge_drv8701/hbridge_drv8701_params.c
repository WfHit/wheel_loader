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
 * H-Bridge DRV8701 PWM frequency
 *
 * PWM frequency for motor control in Hz.
 * Higher frequencies reduce motor noise but increase switching losses.
 *
 * @unit Hz
 * @min 1000
 * @max 100000
 * @decimal 0
 * @increment 1000
 * @group H-Bridge DRV8701
 */
PARAM_DEFINE_INT32(HBRDG_PWM_FREQ, 25000);

/**
 * H-Bridge DRV8701 update rate
 *
 * Control loop update rate in Hz.
 * Higher rates provide better control responsiveness.
 *
 * @unit Hz
 * @min 50
 * @max 1000
 * @decimal 0
 * @increment 10
 * @group H-Bridge DRV8701
 */
PARAM_DEFINE_INT32(HBRDG_UPDATE_RT, 200);

/**
 * H-Bridge Channel 0 PWM output
 *
 * PWM channel number for motor channel 0.
 * Must correspond to a valid PWM output on the flight controller.
 *
 * @min 0
 * @max 15
 * @decimal 0
 * @increment 1
 * @group H-Bridge DRV8701
 */
PARAM_DEFINE_INT32(HBRDG_PWM_CH0, 0);

/**
 * H-Bridge Channel 1 PWM output
 *
 * PWM channel number for motor channel 1.
 * Must correspond to a valid PWM output on the flight controller.
 *
 * @min 0
 * @max 15
 * @decimal 0
 * @increment 1
 * @group H-Bridge DRV8701
 */
PARAM_DEFINE_INT32(HBRDG_PWM_CH1, 1);

/**
 * H-Bridge Enable GPIO pin
 *
 * GPIO pin number for shared enable signal.
 * This pin controls the enable signal for both H-Bridge chips.
 *
 * @min 0
 * @max 255
 * @decimal 0
 * @increment 1
 * @group H-Bridge DRV8701
 */
PARAM_DEFINE_INT32(HBRDG_GPIO_EN, 100);

/**
 * H-Bridge Channel 0 Direction GPIO pin
 *
 * GPIO pin number for channel 0 direction control.
 * Controls the direction of motor channel 0.
 *
 * @min 0
 * @max 255
 * @decimal 0
 * @increment 1
 * @group H-Bridge DRV8701
 */
PARAM_DEFINE_INT32(HBRDG_GPIO_DIR0, 101);

/**
 * H-Bridge Channel 1 Direction GPIO pin
 *
 * GPIO pin number for channel 1 direction control.
 * Controls the direction of motor channel 1.
 *
 * @min 0
 * @max 255
 * @decimal 0
 * @increment 1
 * @group H-Bridge DRV8701
 */
PARAM_DEFINE_INT32(HBRDG_GPIO_DIR1, 102);

/**
 * H-Bridge Channel 0 Fault GPIO pin
 *
 * GPIO pin number for channel 0 fault monitoring.
 * Monitors fault status of H-Bridge chip 0.
 *
 * @min 0
 * @max 255
 * @decimal 0
 * @increment 1
 * @group H-Bridge DRV8701
 */
PARAM_DEFINE_INT32(HBRDG_GPIO_FLT0, 103);

/**
 * H-Bridge Channel 1 Fault GPIO pin
 *
 * GPIO pin number for channel 1 fault monitoring.
 * Monitors fault status of H-Bridge chip 1.
 *
 * @min 0
 * @max 255
 * @decimal 0
 * @increment 1
 * @group H-Bridge DRV8701
 */
PARAM_DEFINE_INT32(HBRDG_GPIO_FLT1, 104);

/**
 * H-Bridge Channel 0 Minimum Limit Sensor Instance
 *
 * Limit sensor instance for channel 0 minimum position limit.
 * Set to 255 to disable limit checking.
 *
 * @min 0
 * @max 255
 * @decimal 0
 * @increment 1
 * @group H-Bridge DRV8701
 */
PARAM_DEFINE_INT32(HBRDG_CH0_LM_MIN, 255);

/**
 * H-Bridge Channel 0 Maximum Limit Sensor Instance
 *
 * Limit sensor instance for channel 0 maximum position limit.
 * Set to 255 to disable limit checking.
 *
 * @min 0
 * @max 255
 * @decimal 0
 * @increment 1
 * @group H-Bridge DRV8701
 */
PARAM_DEFINE_INT32(HBRDG_CH0_LM_MAX, 255);

/**
 * H-Bridge Channel 1 Minimum Limit Sensor Instance
 *
 * Limit sensor instance for channel 1 minimum position limit.
 * Set to 255 to disable limit checking.
 *
 * @min 0
 * @max 255
 * @decimal 0
 * @increment 1
 * @group H-Bridge DRV8701
 */
PARAM_DEFINE_INT32(HBRDG_CH1_LM_MIN, 255);

/**
 * H-Bridge Channel 1 Maximum Limit Sensor Instance
 *
 * Limit sensor instance for channel 1 maximum position limit.
 * Set to 255 to disable limit checking.
 *
 * @min 0
 * @max 255
 * @decimal 0
 * @increment 1
 * @group H-Bridge DRV8701
 */
PARAM_DEFINE_INT32(HBRDG_CH1_LM_MAX, 255);

/**
 * H-Bridge Channel 0 Allow Into Min Limit
 *
 * Allow motion into minimum limit for channel 0.
 * Useful for zeroing operations.
 *
 * @boolean
 * @group H-Bridge DRV8701
 */
PARAM_DEFINE_INT32(HBRDG_CH0_AL_MIN, 0);

/**
 * H-Bridge Channel 0 Allow Into Max Limit
 *
 * Allow motion into maximum limit for channel 0.
 * Useful for calibration operations.
 *
 * @boolean
 * @group H-Bridge DRV8701
 */
PARAM_DEFINE_INT32(HBRDG_CH0_AL_MAX, 0);

/**
 * H-Bridge Channel 1 Allow Into Min Limit
 *
 * Allow motion into minimum limit for channel 1.
 * Useful for zeroing operations.
 *
 * @boolean
 * @group H-Bridge DRV8701
 */
PARAM_DEFINE_INT32(HBRDG_CH1_AL_MIN, 0);

/**
 * H-Bridge Channel 1 Allow Into Max Limit
 *
 * Allow motion into maximum limit for channel 1.
 * Useful for calibration operations.
 *
 * @boolean
 * @group H-Bridge DRV8701
 */
PARAM_DEFINE_INT32(HBRDG_CH1_AL_MAX, 0);
