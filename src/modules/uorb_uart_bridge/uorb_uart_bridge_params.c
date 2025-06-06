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
 * UART Bridge Device Path
 *
 * UART device path for distributed uORB communication
 *
 * @group uORB UART Bridge
 * @value_set {"/dev/ttyS0","/dev/ttyS1","/dev/ttyS2","/dev/ttyS3","/dev/ttyS4","/dev/ttyS5","/dev/ttyS6","/dev/ttyS7"}
 * @reboot_required true
 */
PARAM_DEFINE_STRING(UART_BRIDGE_DEV, "/dev/ttyS2", 32);

/**
 * UART Bridge Baudrate
 *
 * Baudrate for distributed uORB UART communication
 *
 * @group uORB UART Bridge
 * @unit baud
 * @min 9600
 * @max 3000000
 * @value 9600 9600 baud
 * @value 19200 19200 baud
 * @value 38400 38400 baud
 * @value 57600 57600 baud
 * @value 115200 115200 baud
 * @value 230400 230400 baud
 * @value 460800 460800 baud
 * @value 500000 500000 baud
 * @value 921600 921600 baud
 * @value 1000000 1000000 baud
 * @reboot_required true
 */
PARAM_DEFINE_INT32(UART_BRIDGE_BAUD, 115200);

/**
 * UART Bridge Enable
 *
 * Enable distributed uORB UART bridge
 *
 * @group uORB UART Bridge
 * @boolean
 * @reboot_required true
 */
PARAM_DEFINE_INT32(UART_BRIDGE_EN, 0);

/**
 * UART Bridge Statistics Interval
 *
 * Interval for printing bridge statistics (0 = disabled)
 *
 * @group uORB UART Bridge
 * @unit s
 * @min 0
 * @max 60
 */
PARAM_DEFINE_INT32(UART_BRIDGE_STAT, 10);

/**
 * UART Bridge Heartbeat Interval
 *
 * Interval for sending heartbeat messages
 *
 * @group uORB UART Bridge
 * @unit ms
 * @min 100
 * @max 5000
 */
PARAM_DEFINE_INT32(UART_BRIDGE_HB, 1000);

/**
 * UART Bridge Timeout
 *
 * Timeout for detecting offline boards
 *
 * @group uORB UART Bridge
 * @unit ms
 * @min 1000
 * @max 10000
 */
PARAM_DEFINE_INT32(UART_BRIDGE_TO, 3000);
