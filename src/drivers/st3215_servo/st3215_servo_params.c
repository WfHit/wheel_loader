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
 * ST3215 Servo ID
 *
 * ID of the ST3215 servo to control.
 *
 * @min 0
 * @max 253
 * @group ST3215 Servo
 */
PARAM_DEFINE_INT32(ST3215_ID, 1);

/**
 * ST3215 Serial baudrate
 *
 * Baudrate for serial communication with ST3215 servo.
 *
 * @min 9600
 * @max 1000000
 * @value 115200 115200 baud
 * @value 1000000 1000000 baud
 * @group ST3215 Servo
 */
PARAM_DEFINE_INT32(ST3215_BAUDRATE, 1000000);

/**
 * ST3215 Minimum position
 *
 * Minimum position limit for ST3215 servo in degrees.
 *
 * @unit deg
 * @min -180.0
 * @max 0.0
 * @decimal 1
 * @group ST3215 Servo
 */
PARAM_DEFINE_FLOAT(ST3215_MIN_POS, -150.0f);

/**
 * ST3215 Maximum position
 *
 * Maximum position limit for ST3215 servo in degrees.
 *
 * @unit deg
 * @min 0.0
 * @max 180.0
 * @decimal 1
 * @group ST3215 Servo
 */
PARAM_DEFINE_FLOAT(ST3215_MAX_POS, 150.0f);

/**
 * ST3215 Maximum speed
 *
 * Maximum speed limit for ST3215 servo in degrees per second.
 *
 * @unit deg/s
 * @min 1.0
 * @max 500.0
 * @decimal 1
 * @group ST3215 Servo
 */
PARAM_DEFINE_FLOAT(ST3215_MAX_SPD, 360.0f);
