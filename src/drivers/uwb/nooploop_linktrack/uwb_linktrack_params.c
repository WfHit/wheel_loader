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
 * UWB LinkTrack UART baud rate
 *
 * @reboot_required true
 * @group Sensors
 * @value 115200 115200 baud
 * @value 230400 230400 baud
 * @value 460800 460800 baud
 * @value 921600 921600 baud
 */
PARAM_DEFINE_INT32(SENS_UWB_BAUD, 921600);

/**
 * UWB LinkTrack tag ID
 *
 * Tag ID to track. Set to 0 to track all tags.
 *
 * @group Sensors
 * @min 0
 * @max 255
 */
PARAM_DEFINE_INT32(SENS_UWB_TAG_ID, 0);

/**
 * UWB minimum RSSI threshold
 *
 * Measurements below this RSSI will be filtered out.
 *
 * @group Sensors
 * @unit dBm
 * @min -120.0
 * @max -20.0
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(SENS_UWB_MIN_RSSI, -85.0f);

/**
 * UWB publish all tag measurements
 *
 * If enabled, publish ranges for all tags, not just SENS_UWB_TAG_ID.
 *
 * @group Sensors
 * @boolean
 */
PARAM_DEFINE_INT32(SENS_UWB_PUB_ALL, 0);

/**
 * UWB sensor X offset
 *
 * X offset of the UWB sensor relative to the vehicle center of gravity.
 *
 * @group Sensors
 * @unit m
 * @min -5.0
 * @max 5.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(SENS_UWB_OFFSET_X, 0.0f);

/**
 * UWB sensor Y offset
 *
 * Y offset of the UWB sensor relative to the vehicle center of gravity.
 *
 * @group Sensors
 * @unit m
 * @min -5.0
 * @max 5.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(SENS_UWB_OFFSET_Y, 0.0f);

/**
 * UWB sensor Z offset
 *
 * Z offset of the UWB sensor relative to the vehicle center of gravity.
 *
 * @group Sensors
 * @unit m
 * @min -5.0
 * @max 5.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(SENS_UWB_OFFSET_Z, 0.0f);

/**
 * UWB LinkTrack enable
 *
 * Enable UWB LinkTrack positioning system.
 *
 * @group Sensors
 * @boolean
 */
PARAM_DEFINE_INT32(SENS_UWB_EN, 0);

/**
 * UWB LinkTrack UART port
 *
 * UART port for UWB LinkTrack communication.
 *
 * @group Sensors
 */
PARAM_DEFINE_INT32(SENS_UWB_PORT, 6);
