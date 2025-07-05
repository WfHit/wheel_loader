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
 * @file board_quad_encoder_config.cpp
 * @brief Board-specific quadrature encoder configuration for nxt-dual-wl-front
 *
 * This board has two quadrature encoders connected to:
 * - Encoder 0: Motor encoder 1 on PC6/PC7 (A/B phases)
 * - Encoder 1: Motor encoder 2 on PD5/PD6 (A/B phases)
 */

#include <px4_platform_common/px4_config.h>
#include <stm32_gpio.h>
#include <drivers/quadrature_encoder/quadrature_encoder_types.hpp>
#include "board_config.h"

/**
 * Board-specific quadrature encoder configurations for nxt-dual-wl-front
 *
 * This board has two motor encoders connected to:
 * - Encoder 0: Motor encoder 1 on PC6/PC7 (A/B phases)
 * - Encoder 1: Motor encoder 2 on PD5/PD6 (A/B phases)
 */
const struct QuadratureEncoderConfig g_quadrature_encoder_config[] = {
    {
        // Encoder 0 - Motor encoder 1
        .gpio_a = QENCODER1_A_GPIO_RAW,   // PC6 - driver will configure as input with pullup and interrupt
        .gpio_b = QENCODER1_B_GPIO_RAW,   // PC7 - driver will configure as input with pullup
        .pulses_per_revolution = 1024,     // 1024 PPR encoder
        .invert_direction = false,         // Normal direction
        .enable_filtering = true,          // Enable noise filtering
        .filter_window = 3                 // 3-sample filter window
    },
    {
        // Encoder 1 - Motor encoder 2
        .gpio_a = QENCODER2_A_GPIO_RAW,   // PD5 - driver will configure as input with pullup and interrupt
        .gpio_b = QENCODER2_B_GPIO_RAW,   // PD6 - driver will configure as input with pullup
        .pulses_per_revolution = 1024,     // 1024 PPR encoder
        .invert_direction = false,         // Normal direction
        .enable_filtering = true,          // Enable noise filtering
        .filter_window = 3                 // 3-sample filter window
    }
};

// Number of configured encoders
const unsigned int g_quadrature_encoder_count = sizeof(g_quadrature_encoder_config) / sizeof(g_quadrature_encoder_config[0]);

/**
 * Board-specific notes:
 *
 * 1. GPIO Selection:
 *    - QENCODER1_A_GPIO_RAW/QENCODER1_B_GPIO_RAW are used for motor encoder 1 (A/B phases)
 *    - QENCODER2_A_GPIO_RAW/QENCODER2_B_GPIO_RAW are used for motor encoder 2 (A/B phases)
 *    - Defined in board_config.h as PC6/PC7 and PD5/PD6 respectively
 *    - These pins support external interrupts (EXTI)
 *    - Pull-up resistors are enabled for reliable signal detection
 *
 * 2. Encoder Configuration:
 *    - Both encoders use 1024 PPR resolution
 *    - Normal counting direction for both
 *    - Digital filtering enabled with 3-sample window
 *    - Suitable for motor speed feedback applications
 *
 * 3. Hardware Considerations:
 *    - Keep encoder cables short or use shielded cables
 *    - Add 100nF capacitors close to MCU for noise filtering if needed
 *    - Ensure proper grounding between encoder and MCU
 */
