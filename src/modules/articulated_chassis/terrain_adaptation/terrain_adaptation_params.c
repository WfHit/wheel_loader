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
 * @file terrain_adaptation_params.c
 * Parameters for terrain adaptation module.
 *
 * @author PX4 Development Team
 */

#include <px4_platform_common/px4_config.h>
#include <parameters/param.h>

/**
 * Terrain adaptation enable
 *
 * Enable terrain-adaptive control system
 *
 * @boolean
 * @group Terrain Adaptation
 */
PARAM_DEFINE_INT32(TA_ENABLE, 1);

/**
 * Surface roughness threshold
 *
 * Threshold for detecting rough terrain requiring control adaptation
 *
 * @min 0.01
 * @max 1.0
 * @decimal 3
 * @increment 0.01
 * @unit m
 * @group Terrain Adaptation
 */
PARAM_DEFINE_FLOAT(TA_ROUGH_THR, 0.1f);

/**
 * Slope threshold
 *
 * Slope angle threshold for triggering terrain-specific control modes
 *
 * @min 0.0
 * @max 45.0
 * @decimal 1
 * @increment 1.0
 * @unit deg
 * @group Terrain Adaptation
 */
PARAM_DEFINE_FLOAT(TA_SLOPE_THR, 15.0f);

/**
 * Friction estimation alpha
 *
 * Low-pass filter coefficient for friction coefficient estimation
 *
 * @min 0.01
 * @max 1.0
 * @decimal 3
 * @increment 0.01
 * @group Terrain Adaptation
 */
PARAM_DEFINE_FLOAT(TA_FRIC_ALPHA, 0.1f);

/**
 * Speed reduction factor
 *
 * Factor for reducing speed on challenging terrain (1.0 = no reduction)
 *
 * @min 0.1
 * @max 1.0
 * @decimal 2
 * @increment 0.05
 * @group Terrain Adaptation
 */
PARAM_DEFINE_FLOAT(TA_SPEED_RED, 0.8f);

/**
 * Stability margin
 *
 * Safety margin for stability calculations on slopes
 *
 * @min 0.0
 * @max 0.5
 * @decimal 3
 * @increment 0.01
 * @group Terrain Adaptation
 */
PARAM_DEFINE_FLOAT(TA_STAB_MARGIN, 0.1f);

/**
 * Surface classifier method
 *
 * Method for surface type classification
 * 0: Vibration-based
 * 1: Slip-based
 * 2: Combined
 *
 * @min 0
 * @max 2
 * @value 0 Vibration-based
 * @value 1 Slip-based
 * @value 2 Combined
 * @group Terrain Adaptation
 */
PARAM_DEFINE_INT32(TA_CLASSIFIER, 2);
