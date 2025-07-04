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
 * @file board_encoder_config_example.cpp
 * @brief Example board configuration for quadrature encoder driver
 *
 * This file should be placed in the board-specific directory and renamed
 * to board_quadrature_encoder_config.cpp
 *
 * For example:
 * boards/px4/fmu-v5/src/board_quadrature_encoder_config.cpp
 */

#include <px4_platform_common/px4_config.h>
#include <stm32_gpio.h>

// Include the driver header to get the configuration structure
namespace quadrature_encoder {
    struct EncoderConfig;
}

/**
 * Board-specific quadrature encoder configurations
 *
 * This example shows configuration for a board with:
 * - Encoder 0: Wheel speed encoder on PE9/PE11
 * - Encoder 1: Arm position encoder on PD12/PD13
 */
const struct quadrature_encoder::EncoderConfig board_encoder_configs[] = {
    {
        // Encoder 0 - Wheel speed encoder
        .gpio_a = GPIO_PORTE | GPIO_PIN9 | GPIO_INPUT | GPIO_PULLUP,   // PE9
        .gpio_b = GPIO_PORTE | GPIO_PIN11 | GPIO_INPUT | GPIO_PULLUP,  // PE11
        .pulses_per_revolution = 2048,     // 2048 PPR encoder
        .invert_direction = false,         // Normal direction
        .enable_filtering = true,          // Enable noise filtering
        .filter_window = 3                 // 3-sample filter window
    },
    {
        // Encoder 1 - Arm position encoder
        .gpio_a = GPIO_PORTD | GPIO_PIN12 | GPIO_INPUT | GPIO_PULLUP,  // PD12
        .gpio_b = GPIO_PORTD | GPIO_PIN13 | GPIO_INPUT | GPIO_PULLUP,  // PD13
        .pulses_per_revolution = 4096,     // 4096 PPR encoder
        .invert_direction = true,          // Inverted direction
        .enable_filtering = true,          // Enable noise filtering
        .filter_window = 5                 // 5-sample filter window (more filtering)
    }
};

// Number of configured encoders
const unsigned int board_encoder_count = sizeof(board_encoder_configs) / sizeof(board_encoder_configs[0]);

/**
 * Additional board-specific initialization (if needed)
 *
 * This function can be called from board init to perform any
 * additional setup required for the encoders.
 */
void board_quadrature_encoder_init(void)
{
    // Example: Enable GPIO clocks if not already enabled
    // stm32_configgpio(GPIO_PORTE | GPIO_PIN9);

    // Example: Configure alternate functions if needed
    // Note: For basic GPIO interrupts, no alternate function is needed
}

/**
 * Board-specific notes:
 *
 * 1. GPIO Selection:
 *    - Choose pins that support external interrupts (EXTI)
 *    - Avoid pins used by other peripherals
 *    - Consider electrical characteristics (voltage levels, pull-ups)
 *
 * 2. Interrupt Conflicts:
 *    - Only one interrupt per pin number across all ports
 *    - E.g., PA0, PB0, PC0 share the same EXTI line
 *    - Choose pins with unique numbers for phase A signals
 *
 * 3. Encoder Types:
 *    - Incremental encoders: Standard quadrature output
 *    - Absolute encoders: May need different driver
 *    - Consider encoder voltage (3.3V vs 5V)
 *
 * 4. Wiring:
 *    - Keep encoder cables short or use shielded cables
 *    - Add 100nF capacitors close to MCU for noise filtering
 *    - Consider using Schmidt trigger inputs if available
 */
