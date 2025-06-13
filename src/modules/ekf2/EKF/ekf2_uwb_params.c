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
 * @file ekf2_uwb_params.c
 *
 * Parameters for EKF2 UWB fusion
 */

/**
 * Enable UWB range fusion
 *
 * Enables fusion of UWB range measurements for position estimation.
 *
 * @boolean
 * @group EKF2
 */
PARAM_DEFINE_INT32(EKF2_UWB_EN, 0);

/**
 * UWB measurement noise
 *
 * Standard deviation of UWB range measurements.
 *
 * @group EKF2
 * @unit m
 * @min 0.01
 * @max 1.0
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(EKF2_UWB_NOISE, 0.1f);

/**
 * UWB innovation gate
 *
 * Number of standard deviations for UWB measurement gating.
 *
 * @group EKF2
 * @min 1.0
 * @max 10.0
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(EKF2_UWB_GATE, 5.0f);

/**
 * UWB maximum range
 *
 * Maximum expected UWB range for measurement validation.
 *
 * @group EKF2
 * @unit m
 * @min 1.0
 * @max 500.0
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(EKF2_UWB_MAX_RNG, 100.0f);

/**
 * UWB minimum RSSI
 *
 * Minimum RSSI threshold for UWB measurement acceptance.
 *
 * @group EKF2
 * @unit dBm
 * @min -120.0
 * @max -20.0
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(EKF2_UWB_MIN_RSSI, -90.0f);

/**
 * UWB NLOS RSSI threshold
 *
 * RSSI threshold below which measurements are considered NLOS.
 *
 * @group EKF2
 * @unit dBm
 * @min -120.0
 * @max -20.0
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(EKF2_UWB_NLOS_THR, -85.0f);

/**
 * UWB LOS confidence threshold
 *
 * LOS confidence threshold for measurement acceptance (0-100).
 *
 * @group EKF2
 * @min 0
 * @max 100
 */
PARAM_DEFINE_INT32(EKF2_UWB_LOS_THR, 70);

/**
 * UWB power ratio threshold
 *
 * First path to total path power ratio threshold for multipath detection.
 *
 * @group EKF2
 * @min 0.1
 * @max 1.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(EKF2_UWB_PWR_THR, 0.6f);

/**
 * UWB maximum range rate
 *
 * Maximum expected range rate for outlier detection.
 *
 * @group EKF2
 * @unit m/s
 * @min 1.0
 * @max 100.0
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(EKF2_UWB_MAX_RR, 20.0f);

/**
 * UWB Huber threshold
 *
 * Threshold for robust Huber loss function.
 *
 * @group EKF2
 * @unit m
 * @min 0.5
 * @max 5.0
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(EKF2_UWB_HUBER, 2.0f);

/**
 * UWB maximum velocity
 *
 * Maximum velocity constraint for state updates.
 *
 * @group EKF2
 * @unit m/s
 * @min 5.0
 * @max 100.0
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(EKF2_UWB_MAX_VEL, 50.0f);

/**
 * UWB maximum acceleration
 *
 * Maximum acceleration constraint for state updates.
 *
 * @group EKF2
 * @unit m/s^2
 * @min 1.0
 * @max 50.0
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(EKF2_UWB_MAX_ACC, 20.0f);

// Anchor position parameters (16 anchors maximum)
/**
 * UWB Anchor 0 X position
 *
 * X coordinate of UWB anchor 0 in local frame.
 *
 * @group EKF2
 * @unit m
 * @min -1000.0
 * @max 1000.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(EKF2_UWB_A0_X, 0.0f);

/**
 * UWB Anchor 0 Y position
 *
 * Y coordinate of UWB anchor 0 in local frame.
 *
 * @group EKF2
 * @unit m
 * @min -1000.0
 * @max 1000.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(EKF2_UWB_A0_Y, 0.0f);

/**
 * UWB Anchor 0 Z position
 *
 * Z coordinate of UWB anchor 0 in local frame.
 *
 * @group EKF2
 * @unit m
 * @min -100.0
 * @max 100.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(EKF2_UWB_A0_Z, 2.5f);

// Additional anchors 1-15 follow the same pattern
PARAM_DEFINE_FLOAT(EKF2_UWB_A1_X, 10.0f);
PARAM_DEFINE_FLOAT(EKF2_UWB_A1_Y, 0.0f);
PARAM_DEFINE_FLOAT(EKF2_UWB_A1_Z, 2.5f);

PARAM_DEFINE_FLOAT(EKF2_UWB_A2_X, 10.0f);
PARAM_DEFINE_FLOAT(EKF2_UWB_A2_Y, 10.0f);
PARAM_DEFINE_FLOAT(EKF2_UWB_A2_Z, 2.5f);

PARAM_DEFINE_FLOAT(EKF2_UWB_A3_X, 0.0f);
PARAM_DEFINE_FLOAT(EKF2_UWB_A3_Y, 10.0f);
PARAM_DEFINE_FLOAT(EKF2_UWB_A3_Z, 2.5f);

// Anchors 4-15 with default positions
PARAM_DEFINE_FLOAT(EKF2_UWB_A4_X, 0.0f);
PARAM_DEFINE_FLOAT(EKF2_UWB_A4_Y, 0.0f);
PARAM_DEFINE_FLOAT(EKF2_UWB_A4_Z, 0.0f);

PARAM_DEFINE_FLOAT(EKF2_UWB_A5_X, 0.0f);
PARAM_DEFINE_FLOAT(EKF2_UWB_A5_Y, 0.0f);
PARAM_DEFINE_FLOAT(EKF2_UWB_A5_Z, 0.0f);

PARAM_DEFINE_FLOAT(EKF2_UWB_A6_X, 0.0f);
PARAM_DEFINE_FLOAT(EKF2_UWB_A6_Y, 0.0f);
PARAM_DEFINE_FLOAT(EKF2_UWB_A6_Z, 0.0f);

PARAM_DEFINE_FLOAT(EKF2_UWB_A7_X, 0.0f);
PARAM_DEFINE_FLOAT(EKF2_UWB_A7_Y, 0.0f);
PARAM_DEFINE_FLOAT(EKF2_UWB_A7_Z, 0.0f);

PARAM_DEFINE_FLOAT(EKF2_UWB_A8_X, 0.0f);
PARAM_DEFINE_FLOAT(EKF2_UWB_A8_Y, 0.0f);
PARAM_DEFINE_FLOAT(EKF2_UWB_A8_Z, 0.0f);

PARAM_DEFINE_FLOAT(EKF2_UWB_A9_X, 0.0f);
PARAM_DEFINE_FLOAT(EKF2_UWB_A9_Y, 0.0f);
PARAM_DEFINE_FLOAT(EKF2_UWB_A9_Z, 0.0f);

PARAM_DEFINE_FLOAT(EKF2_UWB_A10_X, 0.0f);
PARAM_DEFINE_FLOAT(EKF2_UWB_A10_Y, 0.0f);
PARAM_DEFINE_FLOAT(EKF2_UWB_A10_Z, 0.0f);

PARAM_DEFINE_FLOAT(EKF2_UWB_A11_X, 0.0f);
PARAM_DEFINE_FLOAT(EKF2_UWB_A11_Y, 0.0f);
PARAM_DEFINE_FLOAT(EKF2_UWB_A11_Z, 0.0f);

PARAM_DEFINE_FLOAT(EKF2_UWB_A12_X, 0.0f);
PARAM_DEFINE_FLOAT(EKF2_UWB_A12_Y, 0.0f);
PARAM_DEFINE_FLOAT(EKF2_UWB_A12_Z, 0.0f);

PARAM_DEFINE_FLOAT(EKF2_UWB_A13_X, 0.0f);
PARAM_DEFINE_FLOAT(EKF2_UWB_A13_Y, 0.0f);
PARAM_DEFINE_FLOAT(EKF2_UWB_A13_Z, 0.0f);

PARAM_DEFINE_FLOAT(EKF2_UWB_A14_X, 0.0f);
PARAM_DEFINE_FLOAT(EKF2_UWB_A14_Y, 0.0f);
PARAM_DEFINE_FLOAT(EKF2_UWB_A14_Z, 0.0f);

PARAM_DEFINE_FLOAT(EKF2_UWB_A15_X, 0.0f);
PARAM_DEFINE_FLOAT(EKF2_UWB_A15_Y, 0.0f);
PARAM_DEFINE_FLOAT(EKF2_UWB_A15_Z, 0.0f);
