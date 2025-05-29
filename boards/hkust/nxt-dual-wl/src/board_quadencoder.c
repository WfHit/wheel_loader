/****************************************************************************
 *
 *   Copyright (c) 2023 PX4 Development Team. All rights reserved.
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
 * @file board_quadencoder.c
 *
 * Board-specific quadrature encoder initialization for NXT Dual WL
 */

#define MODULE_NAME "board_quadenc"

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>

#include <arch/board/board.h>

#include <sys/types.h>
#include <errno.h>

#include "board_config.h"
#include "../../../../src/drivers/quad_encoder/quadencoder_common.h"
#include "../../../../src/lib/quad_encoder/quadencoder_types.h"

/* NXT Dual WL Encoder Pin Definitions */

/* Encoder 1 (Left wheel / Primary axis) */
#define NXT_ENCODER1_PIN_A      GPIO_PC0   /* PC0 - using spare GPIO */
#define NXT_ENCODER1_PIN_B      GPIO_PC1   /* PC1 - using spare GPIO */
#define NXT_ENCODER1_PIN_Z      0          /* No index pin */

/* Encoder 2 (Right wheel / Secondary axis) */
#define NXT_ENCODER2_PIN_A      GPIO_PA4   /* PA4 - using spare GPIO */
#define NXT_ENCODER2_PIN_B      (GPIO_INPUT|GPIO_PULLUP|GPIO_PORTA|GPIO_PIN5)  /* PA5 */
#define NXT_ENCODER2_PIN_Z      0          /* No index pin */

/* Pin mode configuration */
#define NXT_ENCODER_PIN_MODE    (GPIO_INPUT | GPIO_PULLUP)

/* Board-specific encoder configurations */
static struct quadencoder_config_s g_encoder1_config = {
	.pins = {
		.pin_a = NXT_ENCODER1_PIN_A,
		.pin_b = NXT_ENCODER1_PIN_B,
		.pin_z = NXT_ENCODER1_PIN_Z,
		.pin_mode = NXT_ENCODER_PIN_MODE,
		.invert_a = false,
		.invert_b = false,
		.invert_z = false
	},
	.hw_type = QUADENCODER_HW_GPIO,
};

static struct quadencoder_config_s g_encoder2_config = {
	.pins = {
		.pin_a = NXT_ENCODER2_PIN_A,
		.pin_b = NXT_ENCODER2_PIN_B,
		.pin_z = NXT_ENCODER2_PIN_Z,
		.pin_mode = NXT_ENCODER_PIN_MODE,
		.invert_a = false,
		.invert_b = false,
		.invert_z = false
	},
	.hw_type = QUADENCODER_HW_GPIO,
};

/****************************************************************************
 * Name: nxt_dual_wl_quadencoder_initialize
 *
 * Description:
 *   Initialize quadrature encoders for NXT Dual WL board
 *   This board supports two encoders for dual wheel control
 *
 ****************************************************************************/
int nxt_dual_wl_quadencoder_initialize(void)
{
	FAR struct quadencoder_dev_s *dev1;
	FAR struct quadencoder_dev_s *dev2;
	int ret;

	PX4_INFO("Initializing NXT Dual WL quadrature encoders");

	/* Initialize encoder 1 (left wheel or primary axis) */
	ret = quadencoder_gpio_initialize(&g_encoder1_config, &dev1);

	if (ret < 0) {
		PX4_ERR("Failed to initialize encoder 1: %d", ret);
		return ret;
	}

	/* Initialize encoder 2 (right wheel or secondary axis) */
	ret = quadencoder_gpio_initialize(&g_encoder2_config, &dev2);

	if (ret < 0) {
		PX4_ERR("Failed to initialize encoder 2: %d", ret);
		return ret;
	}

	PX4_INFO("NXT Dual WL quadrature encoders initialized successfully");
	return OK;
}

/****************************************************************************
 * Name: nxt_dual_wl_quadencoder_set_pins
 *
 * Description:
 *   Runtime pin reconfiguration (if needed)
 *
 ****************************************************************************/
int nxt_dual_wl_quadencoder_set_pins(int encoder_id,
				      struct quadencoder_pins_s *pins)
{
	struct quadencoder_config_s *config;

	/* Select encoder configuration */
	switch (encoder_id) {
	case 0:
		config = &g_encoder1_config;
		break;

	case 1:
		config = &g_encoder2_config;
		break;

	default:
		return -EINVAL;
	}

	/* Update pin configuration */
	memcpy(&config->pins, pins, sizeof(struct quadencoder_pins_s));

	/* Note: This requires re-initialization of the encoder */
	return OK;
}
