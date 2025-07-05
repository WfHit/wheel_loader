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
 * @file quadrature_signal_filter.hpp
 * @brief Digital signal filter for quadrature encoder noise reduction
 */

#include <cstdint>

/**
 * @brief Digital filter for quadrature encoder noise reduction
 *
 * This class provides a majority-vote digital filter for reducing noise
 * in quadrature encoder signals. It maintains a sliding window of samples
 * and requires a majority consensus before changing the output state.
 */
class QuadratureSignalFilter {
public:
	/**
	 * @brief Constructor
	 * @param window_size Size of the filter window (3-5 recommended)
	 */
	explicit QuadratureSignalFilter(uint8_t window_size = 3);

	/**
	 * @brief Update filter with new samples
	 * @param a_state Current state of channel A
	 * @param b_state Current state of channel B
	 * @return True if output is stable
	 */
	bool update(bool a_state, bool b_state);

	/**
	 * @brief Get filtered output for channel A
	 * @return Filtered state of channel A
	 */
	bool get_filtered_a() const { return _output_a; }

	/**
	 * @brief Get filtered output for channel B
	 * @return Filtered state of channel B
	 */
	bool get_filtered_b() const { return _output_b; }

	/**
	 * @brief Reset filter state
	 * Clears all samples and resets outputs to false
	 */
	void reset();

	/**
	 * @brief Get current stability count
	 * @return Number of consecutive stable readings
	 */
	uint8_t get_stability_count() const { return _stable_count; }

private:
	static constexpr uint8_t MAX_WINDOW = 5;  ///< Maximum filter window size

	uint8_t _window_size;                     ///< Current filter window size
	uint8_t _index{0};                        ///< Current sample index
	bool _samples_a[MAX_WINDOW]{};            ///< Sample buffer for channel A
	bool _samples_b[MAX_WINDOW]{};            ///< Sample buffer for channel B
	bool _output_a{false};                    ///< Filtered output for channel A
	bool _output_b{false};                    ///< Filtered output for channel B
	uint8_t _stable_count{0};                 ///< Count of stable readings
};
