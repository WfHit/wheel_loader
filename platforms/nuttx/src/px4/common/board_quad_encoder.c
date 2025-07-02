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
#include <sys/types.h>
#include <errno.h>

#include <px4_arch/quad_encoder.h>
#include <px4_platform_common/board_common.h>
#include <board_quad_encoder.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * External Data - Board must provide these
 ****************************************************************************/

/* Board must provide these symbols */
/* Board-specific encoder configuration array */
extern const struct nuttx_qe_config_s board_quad_encoder_configs[];

/* Number of encoders on the board */
extern const unsigned int board_quad_encoder_count;

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: board_quad_encoder_initialize
 *
 * Description:
 *   Initialize all board-specific quadrature encoders
 *
 ****************************************************************************/

__EXPORT int board_quad_encoder_initialize(void)
{
#ifdef CONFIG_BOARD_QUADRATURE_ENCODER
    if (board_quad_encoder_count == 0) {
        return OK;  /* No encoders to initialize */
    }

    return px4_arch_quad_encoder_initialize(board_quad_encoder_configs, board_quad_encoder_count);
#else
    return OK;
#endif
}

/****************************************************************************
 * Name: board_quad_encoder_get_config
 *
 * Description:
 *   Get board-specific encoder configuration for a specific encoder
 *
 ****************************************************************************/

__EXPORT int board_quad_encoder_get_config(int encoder_id,
                                          FAR struct nuttx_qe_config_s *config)
{
#ifdef CONFIG_BOARD_QUADRATURE_ENCODER
    if (encoder_id < 0 || encoder_id >= (int)board_quad_encoder_count || !config) {
        return -EINVAL;
    }

    *config = board_quad_encoder_configs[encoder_id];
    return OK;
#else
    return -ENOTSUP;
#endif
}

/****************************************************************************
 * Name: board_quad_encoder_get_count
 *
 * Description:
 *   Get the number of available encoders on this board
 *
 ****************************************************************************/

__EXPORT int board_quad_encoder_get_count(void)
{
#ifdef CONFIG_BOARD_QUADRATURE_ENCODER
    return board_quad_encoder_count;
#else
    return 0;
#endif
}
