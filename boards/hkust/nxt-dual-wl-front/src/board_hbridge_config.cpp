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
 * @file board_hbridge_config.cpp
 *
 * Board-specific H-Bridge configuration for NXT-Dual-WL-Front
 */

#include <board_config.h>
#include <drivers/hbridge/hbridge_config.h>

#ifdef BOARD_HAS_HBRIDGE_CONFIG

// Define H-Bridge instances with their hardware mappings for front board
const hbridge_config_t g_hbridge_config[BOARD_NUM_HBRIDGES] = {
    // Instance 0: Front Wheel Drive H-Bridge
    {
        .instance_id = 0,
        .name = "front_wheel_drive",

        // PWM Configuration
        .left_pwm_channel = 0,          // Timer channel for left wheel
        .right_pwm_channel = 1,         // Timer channel for right wheel
        .pwm_frequency = 25000,         // 25 kHz PWM frequency

        // Direction Control GPIOs
        .left_dir_gpio = GPIO_DRV8701_LEFT_DIR,    // PE14 - Left wheel direction
        .right_dir_gpio = GPIO_DRV8701_RIGHT_DIR,  // PE13 - Right wheel direction
        .left_dir_inverted = false,     // Normal direction logic
        .right_dir_inverted = false,    // Normal direction logic

        // Enable GPIO
        .enable_gpio = GPIO_DRV8701_ENABLE,        // Enable pin for H-Bridge

        // Limit Sensor Integration (not used for wheel drive)
        .left_fwd_limit_function = 255,  // Disabled
        .left_rev_limit_function = 255,  // Disabled
        .right_fwd_limit_function = 255, // Disabled
        .right_rev_limit_function = 255, // Disabled

        // Current Sensing (future expansion)
        .current_sensing_enabled = false,
        .left_current_adc_channel = 255,  // Disabled
        .right_current_adc_channel = 255, // Disabled
        .current_scale_factor = 0.0f,

        // Fault Detection (future expansion)
        .fault_gpio = 0,                 // Not used
        .fault_inverted = false
    }
};

#endif // BOARD_HAS_HBRIDGE_CONFIG
