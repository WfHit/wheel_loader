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
#include <px4_arch/nuttx_qencoder.h>
#include "board_config.h"

/****************************************************************************
 * Public Data
 ****************************************************************************/

#ifdef CONFIG_BOARD_QUADRATURE_ENCODER

/* Board-specific encoder configurations for nxt-dual-wl-rear */
const struct nuttx_qe_config_s board_quad_encoder_configs[] =
{
  /* Motor Encoder - GPIO mode */
  {
    .gpio = {
      .phase_a = QENCODER_A_GPIO_RAW,
      .phase_b = QENCODER_B_GPIO_RAW,
      .index = 0,  /* No index signal */
    },
    .resolution = 1024,    /* 1024 CPR encoder */
    .use_index = false,    /* No index signal for motor encoder */
    .x4_mode = true,       /* 4x counting mode for higher resolution */
    .invert_dir = false,
  },
};

/* Number of encoders on this board */
const unsigned int board_quad_encoder_count = sizeof(board_quad_encoder_configs) / sizeof(board_quad_encoder_configs[0]);

#else

/* Empty configuration when encoder support is disabled */
const struct nuttx_qe_config_s board_quad_encoder_configs[] = {};
const unsigned int board_quad_encoder_count = 0;

#endif /* CONFIG_BOARD_QUADRATURE_ENCODER */
