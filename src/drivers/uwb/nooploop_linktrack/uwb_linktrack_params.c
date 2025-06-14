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
 * @file uwb_linktrack_params.c
 *
 * Parameters for Nooploop LinkTrack UWB driver.
 */

/**
 * UWB LinkTrack serial port
 *
 * Configure on which serial port to run NoopLoop LinkTrack UWB.
 *
 * @value 0 Disabled
 * @value 6 UART 6
 * @value 101 TELEM 1
 * @value 102 TELEM 2
 * @value 103 TELEM 3
 * @value 201 GPS 1
 * @value 202 GPS 2
 * @value 203 GPS 3
 * @value 300 Radio Controller
 * @value 301 Wifi Port
 * @value 401 EXT2
 *
 * @group UWB
 * @reboot_required true
 */
PARAM_DEFINE_INT32(UWB_PORT, 0);

/**
 * UWB tag X offset
 *
 * Position of UWB tag in body frame (X axis, positive forward)
 *
 * @unit m
 * @min -5.0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group UWB
 */
PARAM_DEFINE_FLOAT(UWB_X_OFF, 0.0f);

/**
 * UWB tag Y offset
 *
 * Position of UWB tag in body frame (Y axis, positive right)
 *
 * @unit m
 * @min -5.0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group UWB
 */
PARAM_DEFINE_FLOAT(UWB_Y_OFF, 0.0f);

/**
 * UWB tag Z offset
 *
 * Position of UWB tag in body frame (Z axis, positive down)
 *
 * @unit m
 * @min -5.0
 * @max 5.0
 * @decimal 2
 * @increment 0.01
 * @group UWB
 */
PARAM_DEFINE_FLOAT(UWB_Z_OFF, 0.0f);

/**
 * UWB velocity X bias
 *
 * Velocity bias for X axis in m/s
 *
 * @unit m/s
 * @min -1.0
 * @max 1.0
 * @decimal 3
 * @increment 0.001
 * @group UWB
 */
PARAM_DEFINE_FLOAT(UWB_VX_BIAS, 0.0f);

/**
 * UWB velocity Y bias
 *
 * Velocity bias for Y axis in m/s
 *
 * @unit m/s
 * @min -1.0
 * @max 1.0
 * @decimal 3
 * @increment 0.001
 * @group UWB
 */
PARAM_DEFINE_FLOAT(UWB_VY_BIAS, 0.0f);

/**
 * UWB velocity Z bias
 *
 * Velocity bias for Z axis in m/s
 *
 * @unit m/s
 * @min -1.0
 * @max 1.0
 * @decimal 3
 * @increment 0.001
 * @group UWB
 */
PARAM_DEFINE_FLOAT(UWB_VZ_BIAS, 0.0f);

/**
 * UWB position X offset
 *
 * Position offset for X axis in meters
 *
 * @unit m
 * @min -10.0
 * @max 10.0
 * @decimal 2
 * @increment 0.01
 * @group UWB
 */
PARAM_DEFINE_FLOAT(UWB_OFFSET_X, 0.0f);

/**
 * UWB position Y offset
 *
 * Position offset for Y axis in meters
 *
 * @unit m
 * @min -10.0
 * @max 10.0
 * @decimal 2
 * @increment 0.01
 * @group UWB
 */
PARAM_DEFINE_FLOAT(UWB_OFFSET_Y, 0.0f);

/**
 * UWB position Z offset
 *
 * Position offset for Z axis in meters
 *
 * @unit m
 * @min -10.0
 * @max 10.0
 * @decimal 2
 * @increment 0.01
 * @group UWB
 */
PARAM_DEFINE_FLOAT(UWB_OFFSET_Z, 0.0f);

/**
 * UWB position X scale factor
 *
 * Scale factor for X axis position measurements
 *
 * @min 0.1
 * @max 10.0
 * @decimal 3
 * @increment 0.001
 * @group UWB
 */
PARAM_DEFINE_FLOAT(UWB_SCALE_X, 1.0f);

/**
 * UWB position Y scale factor
 *
 * Scale factor for Y axis position measurements
 *
 * @min 0.1
 * @max 10.0
 * @decimal 3
 * @increment 0.001
 * @group UWB
 */
PARAM_DEFINE_FLOAT(UWB_SCALE_Y, 1.0f);

/**
 * UWB position Z scale factor
 *
 * Scale factor for Z axis position measurements
 *
 * @min 0.1
 * @max 10.0
 * @decimal 3
 * @increment 0.001
 * @group UWB
 */
PARAM_DEFINE_FLOAT(UWB_SCALE_Z, 1.0f);

/**
 * UWB operation mode
 *
 * Set the UWB operation mode
 *
 * @value 0 Passive mode (receive only)
 * @value 1 Active mode (initiate ranging)
 * @value 2 Auto mode
 *
 * @group UWB
 */
PARAM_DEFINE_INT32(UWB_MODE, 0);

/**
 * UWB data rate divider
 *
 * Divide the incoming UWB data rate by this factor
 *
 * @value 1 Full rate
 * @value 2 Half rate
 * @value 5 1/5 rate
 * @value 10 1/10 rate
 *
 * @group UWB
 */
PARAM_DEFINE_INT32(UWB_RATE_DIV, 5);

/**
 * UWB tag ID
 *
 * Tag ID to track. Set to 0 to track all tags.
 *
 * @group UWB
 * @min 0
 * @max 255
 */
PARAM_DEFINE_INT32(UWB_TAG_ID, 0);

/**
 * UWB minimum RSSI
 *
 * Measurements below this RSSI will be filtered out.
 *
 * @group UWB
 * @unit
 * @min -120.0
 * @max -20.0
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(UWB_MIN_RSSI, -85.0f);

/**
 * UWB publish all tags
 *
 * If enabled, publish ranges for all tags, not just UWB_TAG_ID.
 *
 * @group UWB
 * @boolean
 */
PARAM_DEFINE_INT32(UWB_PUB_ALL, 0);

/**
 * UWB enable
 *
 * Enable UWB LinkTrack positioning system.
 *
 * @group UWB
 * @boolean
 */
PARAM_DEFINE_INT32(UWB_EN, 0);

/**
 * UWB baud rate
 *
 * Serial baud rate for UWB LinkTrack communication.
 *
 * @group UWB
 * @value 9600 9600 baud
 * @value 38400 38400 baud
 * @value 57600 57600 baud
 * @value 115200 115200 baud
 * @value 230400 230400 baud
 * @value 460800 460800 baud
 * @value 921600 921600 baud
 */
PARAM_DEFINE_INT32(UWB_BAUD, 921600);
