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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include "board_config.h"
#include <drivers/quad_encoder_gpio/quad_encoder_gpio.hpp>

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_DRIVERS_QUAD_ENCODER_GPIO

/* Board-specific GPIO encoder configurations for nxt-dual-wl-rear */
const struct quad_encoder_gpio_config_s board_quad_encoder_gpio_configs[] =
{
  /* Motor Encoder - GPIO interrupt mode */
  {
    .gpio_a = QENCODER_A_GPIO_RAW,      /* PD5 */
    .gpio_b = QENCODER_B_GPIO_RAW,      /* PD6 */
    .resolution = 1024,                  /* 1024 CPR encoder */
    .invert_direction = false,           /* Normal direction */
    .use_index = false,                  /* No index signal for motor encoder */
    .x4_mode = true,                     /* 4x counting mode for higher resolution */
    .max_frequency = 10000,              /* 10 kHz maximum frequency */
    .filter_samples = 3,                 /* 3-sample digital filter */
  },
};

/* Number of GPIO encoders on this board */
const unsigned int board_quad_encoder_gpio_count = sizeof(board_quad_encoder_gpio_configs) / sizeof(board_quad_encoder_gpio_configs[0]);

#else

/* Empty configuration when GPIO encoder support is disabled */
const struct quad_encoder_gpio_config_s board_quad_encoder_gpio_configs[] = {};
const unsigned int board_quad_encoder_gpio_count = 0;

#endif
