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
 * @file quadrature_encoder_state.hpp
 * @brief Quadrature encoder state data structures for quadrature encoder driver
 */

#include <px4_platform_common/atomic.h>
#include <drivers/drv_hrt.h>

/**
 * @brief Quadrature event structure for interrupt -> work queue communication
 *
 * Simple event structure to capture GPIO state changes in interrupt context
 * and defer processing to work queue context.
 */
struct QuadratureEvent {
	hrt_abstime timestamp;        ///< Event timestamp
	uint8_t gpio_state;          ///< Combined GPIO state (B|A)
};

/**
 * @brief Quadrature encoder state information
 *
 * This structure contains all the runtime state information for a quadrature encoder.
 * Uses volatile for variables that are updated from work queue context and read from main thread.
 * Work queue processing ensures single-threaded access to state variables.
 */
struct QuadratureEncoderState {
	// Core state (updated in work queue, read in main thread)
	volatile int32_t position{0};          ///< Current position (counts)
	volatile float velocity{0.0f};         ///< Current velocity (rad/s)
	volatile float angle{0.0f};            ///< Cumulative angle (rad)
	volatile bool healthy{false};          ///< Health status

	// Quadrature tracking (updated in work queue)
	volatile uint8_t quadrature_state{0};  ///< Current quadrature state (2-bit)
	volatile uint32_t transitions{0};      ///< Total state transitions
	volatile uint32_t errors{0};           ///< Total errors detected
	volatile uint64_t last_update{0};      ///< Last update timestamp

	// Velocity calculation variables (work queue only)
	hrt_abstime velocity_timestamp{0};     ///< Timestamp for velocity calculation
	int32_t velocity_position{0};          ///< Position for velocity calculation
};
