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
 * @file quadrature_signal_filter.cpp
 * @brief Implementation of digital signal filter for quadrature encoder
 */

#include "quadrature_signal_filter.hpp"
#include <lib/mathlib/mathlib.h>

QuadratureSignalFilter::QuadratureSignalFilter(uint8_t window_size) :
	_window_size(math::min(window_size, MAX_WINDOW))
{
	reset();
}

bool QuadratureSignalFilter::update(bool a_state, bool b_state)
{
	// Store new samples
	_samples_a[_index] = a_state;
	_samples_b[_index] = b_state;
	_index = (_index + 1) % _window_size;

	// Calculate majority vote
	uint8_t a_count = 0, b_count = 0;
	for (uint8_t i = 0; i < _window_size; i++) {
		if (_samples_a[i]) a_count++;
		if (_samples_b[i]) b_count++;
	}

	bool new_a = (a_count > _window_size / 2);
	bool new_b = (b_count > _window_size / 2);

	// Check stability
	if (new_a == _output_a && new_b == _output_b) {
		_stable_count = math::min(static_cast<int>(_stable_count + 1), 255);
	} else {
		_stable_count = 0;
		_output_a = new_a;
		_output_b = new_b;
	}

	// Require minimum stability
	return (_stable_count >= 2);
}

void QuadratureSignalFilter::reset()
{
	_index = 0;
	_stable_count = 0;
	_output_a = false;
	_output_b = false;

	for (uint8_t i = 0; i < MAX_WINDOW; i++) {
		_samples_a[i] = false;
		_samples_b[i] = false;
	}
}
