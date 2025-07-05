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

#pragma once

/**
 * @file quadrature_encoder_types.hpp
 * @brief Type definitions for quadrature encoder configuration
 *
 * This header contains only the type definitions needed for board configuration
 * files, without any module dependencies.
 */

#include <stdint.h>

/**
 * Board-specific quadrature encoder configuration structure
 */
struct QuadratureEncoderConfig {
	uint32_t gpio_a;                 ///< GPIO pin for channel A (with all flags)
	uint32_t gpio_b;                 ///< GPIO pin for channel B (with all flags)
	uint32_t pulses_per_revolution;  ///< Number of pulses per full revolution
	bool invert_direction;           ///< Whether to invert the counting direction
	bool enable_filtering;           ///< Enable digital filtering for noise immunity
	uint8_t filter_window;           ///< Filter window size (2-8 samples)
};

// Board-specific quadrature encoder configuration (provided by drivers_board)
__EXPORT extern const QuadratureEncoderConfig g_quadrature_encoder_config[];
__EXPORT extern const unsigned int g_quadrature_encoder_count;
