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
 * @file nuttx_qencoder.h
 * 
 * Quadrature encoder support for NuttX/PX4
 */

#pragma once

#include <nuttx/config.h>
#include <stdint.h>
#include <stdbool.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Types
 ****************************************************************************/

/* GPIO configuration for quadrature encoder */
struct nuttx_qe_gpio_config_s
{
	uint32_t phase_a;	/* Phase A GPIO configuration */
	uint32_t phase_b;	/* Phase B GPIO configuration */
	uint32_t index;		/* Index GPIO configuration (optional, 0 if not used) */
};

/* Quadrature encoder configuration */
struct nuttx_qe_config_s
{
	struct nuttx_qe_gpio_config_s gpio;	/* GPIO configuration */
	uint32_t resolution;			/* Encoder resolution (CPR) */
	bool use_index;				/* Whether to use index signal */
	bool x4_mode;				/* 4x counting mode */
	bool invert_dir;			/* Invert direction */
};

/****************************************************************************
 * Public Function Prototypes
 ****************************************************************************/

#ifdef __cplusplus
extern "C"
{
#endif

/**
 * Initialize a quadrature encoder
 *
 * @param config	Encoder configuration
 * @param devpath	Device path for the encoder
 * @return		0 on success, negative error code on failure
 */
int nuttx_qencoder_initialize(const struct nuttx_qe_config_s *config, const char *devpath);

#ifdef __cplusplus
}
#endif