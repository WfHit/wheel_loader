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
 * Wheel radius
 *
 * Radius of the wheel in meters. Used for converting rotational velocity to linear velocity.
 *
 * @unit m
 * @min 0.1
 * @max 1.0
 * @decimal 2
 * @group Slip Estimator
 */
PARAM_DEFINE_FLOAT(SE_WHEEL_R, 0.5f);

/**
 * Vehicle wheelbase
 *
 * Distance between front and rear axles in meters. Used for lateral slip calculations.
 *
 * @unit m
 * @min 1.0
 * @max 5.0
 * @decimal 2
 * @group Slip Estimator
 */
PARAM_DEFINE_FLOAT(SE_WHEELBASE, 2.5f);

/**
 * Vehicle mass
 *
 * Total mass of the vehicle in kilograms. Used for force calculations in slip estimation.
 *
 * @unit kg
 * @min 500.0
 * @max 5000.0
 * @decimal 1
 * @group Slip Estimator
 */
PARAM_DEFINE_FLOAT(SE_VEHICLE_MASS, 1000.0f);

/**
 * Center of gravity height
 *
 * Height of the vehicle center of gravity above ground in meters.
 *
 * @unit m
 * @min 0.5
 * @max 3.0
 * @decimal 2
 * @group Slip Estimator
 */
PARAM_DEFINE_FLOAT(SE_COG_HEIGHT, 1.0f);

/**
 * Process noise
 *
 * Process noise covariance for the slip estimation EKF.
 *
 * @min 0.001
 * @max 0.1
 * @decimal 3
 * @group Slip Estimator
 */
PARAM_DEFINE_FLOAT(SE_PROC_NOISE, 0.01f);

/**
 * Measurement noise
 *
 * Measurement noise covariance for wheel speed measurements.
 *
 * @min 0.01
 * @max 1.0
 * @decimal 3
 * @group Slip Estimator
 */
PARAM_DEFINE_FLOAT(SE_MEAS_NOISE, 0.1f);

/**
 * Estimation method
 *
 * Slip estimation method: 0 = Basic calculation, 1 = Extended Kalman Filter
 *
 * @min 0
 * @max 1
 * @group Slip Estimator
 * @value 0 Basic calculation
 * @value 1 Extended Kalman Filter
 */
PARAM_DEFINE_INT32(SE_EST_METHOD, 1);

/**
 * Front encoder index
 *
 * Index of the front wheel encoder in the sensor_quad_encoder message.
 *
 * @min 0
 * @max 3
 * @group Slip Estimator
 */
PARAM_DEFINE_INT32(SE_FRONT_IDX, 0);

/**
 * Rear encoder index
 *
 * Index of the rear wheel encoder in the sensor_quad_encoder message.
 *
 * @min 0
 * @max 3
 * @group Slip Estimator
 */
PARAM_DEFINE_INT32(SE_REAR_IDX, 1);

/**
 * Slip threshold
 *
 * Threshold for slip detection. Values above this trigger traction control.
 *
 * @min 0.01
 * @max 0.5
 * @decimal 2
 * @group Slip Estimator
 */
PARAM_DEFINE_FLOAT(SE_SLIP_THRESH, 0.15f);

/**
 * Initial friction coefficient
 *
 * Initial friction coefficient estimate for the surface.
 *
 * @min 0.1
 * @max 1.5
 * @decimal 2
 * @group Slip Estimator
 */
PARAM_DEFINE_FLOAT(SE_FRIC_INIT, 0.8f);

/**
 * Update frequency
 *
 * Slip estimator update frequency in Hz.
 *
 * @unit Hz
 * @min 10
 * @max 200
 * @group Slip Estimator
 */
PARAM_DEFINE_INT32(SE_UPDATE_HZ, 100);
