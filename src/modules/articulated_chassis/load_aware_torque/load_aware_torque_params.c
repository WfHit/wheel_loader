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
 * @file load_aware_torque_params.c
 * Parameters for load-aware torque distribution
 *
 * @author PX4 Development Team
 */

/**
 * Minimum front axle torque ratio
 *
 * Minimum allowable torque allocation to front axle (0.0 = all rear, 1.0 = all front)
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Load Aware Torque
 */
PARAM_DEFINE_FLOAT(LAT_MIN_FRONT_RATIO, 0.2f);

/**
 * Maximum front axle torque ratio
 *
 * Maximum allowable torque allocation to front axle (0.0 = all rear, 1.0 = all front)
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Load Aware Torque
 */
PARAM_DEFINE_FLOAT(LAT_MAX_FRONT_RATIO, 0.8f);

/**
 * Distribution filter time constant
 *
 * Time constant for low-pass filtering of torque distribution changes
 *
 * @min 0.01
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @unit s
 * @group Load Aware Torque
 */
PARAM_DEFINE_FLOAT(LAT_FILTER_TC, 0.1f);

/**
 * Distribution rate limit
 *
 * Maximum rate of change for torque distribution ratio
 *
 * @min 0.1
 * @max 5.0
 * @decimal 1
 * @increment 0.1
 * @unit 1/s
 * @group Load Aware Torque
 */
PARAM_DEFINE_FLOAT(LAT_RATE_LIMIT, 1.0f);

/**
 * Stability correction gain
 *
 * Gain for stability-based torque distribution corrections
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @increment 0.1
 * @group Load Aware Torque
 */
PARAM_DEFINE_FLOAT(LAT_STABILITY_GAIN, 0.5f);

/**
 * Dynamic adjustment gain
 *
 * Gain for dynamic weight transfer compensation
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @increment 0.1
 * @group Load Aware Torque
 */
PARAM_DEFINE_FLOAT(LAT_DYNAMIC_GAIN, 0.8f);

/**
 * Terrain adaptation gain
 *
 * Gain for terrain-based torque distribution adjustments
 *
 * @min 0.0
 * @max 2.0
 * @decimal 2
 * @increment 0.1
 * @group Load Aware Torque
 */
PARAM_DEFINE_FLOAT(LAT_TERRAIN_GAIN, 0.3f);

/**
 * Efficiency optimization weight
 *
 * Weight factor for fuel efficiency optimization in torque distribution
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.1
 * @group Load Aware Torque
 */
PARAM_DEFINE_FLOAT(LAT_EFFICIENCY_WEIGHT, 0.4f);

/**
 * Traction optimization weight
 *
 * Weight factor for traction optimization in torque distribution
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @increment 0.1
 * @group Load Aware Torque
 */
PARAM_DEFINE_FLOAT(LAT_TRACTION_WEIGHT, 0.6f);

/**
 * Center of gravity X offset
 *
 * Manual offset for center of gravity X position estimation
 *
 * @min -1.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @unit m
 * @group Load Aware Torque
 */
PARAM_DEFINE_FLOAT(LAT_COG_X_OFFSET, 0.0f);

/**
 * Center of gravity Z offset
 *
 * Manual offset for center of gravity height estimation
 *
 * @min -0.5
 * @max 0.5
 * @decimal 2
 * @increment 0.01
 * @unit m
 * @group Load Aware Torque
 */
PARAM_DEFINE_FLOAT(LAT_COG_Z_OFFSET, 0.0f);

/**
 * Adaptive control enable
 *
 * Enable adaptive torque distribution based on real-time conditions
 *
 * @boolean
 * @group Load Aware Torque
 */
PARAM_DEFINE_INT32(LAT_ADAPTIVE_EN, 1);

/**
 * Efficiency optimization enable
 *
 * Enable fuel efficiency optimization in torque distribution
 *
 * @boolean
 * @group Load Aware Torque
 */
PARAM_DEFINE_INT32(LAT_EFFICIENCY_OPT, 1);
