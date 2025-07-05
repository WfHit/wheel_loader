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
 * @brief Board-specific quadrature encoder configuration for nxt-dual-wl-rear
 *
 * This board has one quadrature encoder connected to:
 * - Encoder: Motor encoder on PD5/PD6 (A/B phases)
 */

#include <px4_platform_common/px4_config.h>
#include <stm32_gpio.h>
#include <drivers/quadrature_encoder/quadrature_encoder_types.hpp>
#include "board_config.h"

#ifdef BOARD_HAS_QUADRATURE_ENCODER_CONFIG

/**
 * Board-specific quadrature encoder configurations for nxt-dual-wl-rear
 *
 * This board has one motor encoder connected to:
 * - Encoder 0: Motor encoder on PD5/PD6 (A/B phases)
 */

const struct QuadratureEncoderConfig g_quadrature_encoder_config[BOARD_NUM_QUADRATURE_ENCODERS] = {
    {
        // Encoder 0 - Motor encoder
        .gpio_a = QENCODER_A_GPIO_RAW,     // PD5 - driver will configure as input with pullup and interrupt
        .gpio_b = QENCODER_B_GPIO_RAW,     // PD6 - driver will configure as input with pullup
        .pulses_per_revolution = 1024,     // 1024 PPR encoder
        .invert_direction = false,         // Normal direction
        .enable_filtering = true,          // Enable noise filtering
        .filter_window = 3                 // 3-sample filter window
    }
};

// Number of configured encoders
const unsigned int g_quadrature_encoder_count = BOARD_NUM_QUADRATURE_ENCODERS;

#endif // BOARD_HAS_QUADRATURE_ENCODER_CONFIG

/**
 * Board-specific notes:
 *
 * 1. GPIO Selection:
 *    - QENCODER_A_GPIO_RAW/QENCODER_B_GPIO_RAW are used for the motor encoder (A/B phases)
 *    - Defined in board_config.h as PD5/PD6
 *    - These pins support external interrupts (EXTI)
 *    - Pull-up resistors are enabled for reliable signal detection
 *
 * 2. Encoder Configuration:
 *    - 1024 PPR encoder resolution
 *    - Normal counting direction
 *    - Digital filtering enabled with 3-sample window
 *    - Suitable for motor speed feedback applications
 *
 * 3. Hardware Considerations:
 *    - Keep encoder cables short or use shielded cables
 *    - Add 100nF capacitors close to MCU for noise filtering if needed
 *    - Ensure proper grounding between encoder and MCU
 */
