/**
 * @file bucket_control_params.c
 * Bucket control module parameters.
 */

/**
 * Bucket motor index
 *
 * Index of the motor in the actuator_motors array for bucket control
 *
 * @group Bucket Control
 * @min 0
 * @max 15
 */
PARAM_DEFINE_INT32(BCT_MOT_IDX, 0);

/**
 * Bucket encoder index
 *
 * Index of the encoder in the wheel_encoders array for bucket position feedback
 *
 * @group Bucket Control
 * @min 0
 * @max 7
 */
PARAM_DEFINE_INT32(BCT_ENC_IDX, 0);

/**
 * Bucket coarse limit sensor instance
 *
 * Limit sensor instance ID for the coarse limit at bucket down position
 *
 * @group Bucket Control
 * @min 0
 * @max 7
 */
PARAM_DEFINE_INT32(BCT_LIM_COARSE, 0);

/**
 * Bucket fine limit sensor instance
 *
 * Limit sensor instance ID for the fine limit at bucket up position
 *
 * @group Bucket Control
 * @min 0
 * @max 7
 */
PARAM_DEFINE_INT32(BCT_LIM_FINE, 1);

/**
 * Actuator base attachment X
 *
 * X position of actuator base relative to boom pivot (mm)
 *
 * @group Bucket Geometry
 * @unit mm
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(BCT_ACT_BASE_X, -200.0);

/**
 * Actuator base attachment Y
 *
 * Y position of actuator base relative to boom pivot (mm)
 *
 * @group Bucket Geometry
 * @unit mm
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(BCT_ACT_BASE_Y, -100.0);

/**
 * Bellcrank boom attachment X
 *
 * X position of bellcrank pivot on boom (mm)
 *
 * @group Bucket Geometry
 * @unit mm
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(BCT_BELLCRANK_BOOM_X, 300.0);

/**
 * Bellcrank boom attachment Y
 *
 * Y position of bellcrank pivot on boom (mm)
 *
 * @group Bucket Geometry
 * @unit mm
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(BCT_BELLCRANK_BOOM_Y, 50.0);

/**
 * Bucket boom pivot X
 *
 * X position of bucket pivot on boom (mm)
 *
 * @group Bucket Geometry
 * @unit mm
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(BCT_BKT_BOOM_X, 600.0);

/**
 * Bucket boom pivot Y
 *
 * Y position of bucket pivot on boom (mm)
 *
 * @group Bucket Geometry
 * @unit mm
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(BCT_BKT_BOOM_Y, 0.0);

/**
 * Bellcrank length
 *
 * Total length of bellcrank (mm)
 *
 * @group Bucket Geometry
 * @unit mm
 * @min 10.0
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(BCT_BELLCRANK_LENGTH, 150.0);

/**
 * Coupler length
 *
 * Total length of coupler connecting bellcrank to bucket (mm)
 *
 * @group Bucket Geometry
 * @unit mm
 * @min 10.0
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(BCT_COUPLER_LENGTH, 200.0);

/**
 * Actuator attachment offset
 *
 * Distance from bellcrank pivot to actuator attachment point (mm)
 *
 * @group Bucket Geometry
 * @unit mm
 * @min 10.0
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(BCT_ACT_OFFSET, 50.0);

/**
 * Bucket arm length
 *
 * Distance from bucket pivot to coupler attachment point (mm)
 *
 * @group Bucket Geometry
 * @unit mm
 * @min 10.0
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(BCT_BKT_ARM_LEN, 100.0);

/**
 * Bellcrank internal angle
 *
 * Fixed angle between bellcrank arms (actuator to coupler) in radians
 *
 * @group Bucket Geometry
 * @unit rad
 * @min -3.14159
 * @max 3.14159
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(BCT_BELLCRANK_INT_ANG, 1.57);

/**
 * Bucket attachment offset
 *
 * Angular offset of bucket arm from coupler direction (rad)
 *
 * @group Bucket Geometry
 * @unit rad
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(BCT_BKT_OFFSET, 0.0);

/**
 * Boom length
 *
 * Distance from boom pivot to bucket pivot (mm)
 *
 * @group Bucket Geometry
 * @unit mm
 * @min 100.0
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(BCT_BOOM_LENGTH, 800.0);

/**
 * Minimum actuator length
 *
 * Minimum extension of the bucket actuator (mm)
 *
 * @group Bucket Control
 * @unit mm
 * @min 50.0
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(BCT_ACT_MIN, 200.0);

/**
 * Maximum actuator length
 *
 * Maximum extension of the bucket actuator (mm)
 *
 * @group Bucket Control
 * @unit mm
 * @min 100.0
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(BCT_ACT_MAX, 400.0);

/**
 * Minimum bucket angle
 *
 * Minimum bucket angle relative to boom (rad)
 *
 * @group Bucket Control
 * @unit rad
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(BCT_ANG_MIN, -1.57);

/**
 * Maximum bucket angle
 *
 * Maximum bucket angle relative to boom (rad)
 *
 * @group Bucket Control
 * @unit rad
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(BCT_ANG_MAX, 1.57);

/**
 * Encoder scale factor
 *
 * Conversion factor from encoder counts to mm of actuator travel
 *
 * @group Bucket Control
 * @unit mm/count
 * @decimal 6
 */
PARAM_DEFINE_FLOAT(BCT_ENC_SCALE, 0.001);

/**
 * Position control P gain
 *
 * Proportional gain for bucket position control
 *
 * @group Bucket Control
 * @min 0.0
 * @max 10.0
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(BCT_PID_P, 2.0);

/**
 * Position control I gain
 *
 * Integral gain for bucket position control
 *
 * @group Bucket Control
 * @min 0.0
 * @max 5.0
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(BCT_PID_I, 0.1);

/**
 * Position control D gain
 *
 * Derivative gain for bucket position control
 *
 * @group Bucket Control
 * @min 0.0
 * @max 2.0
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(BCT_PID_D, 0.05);

/**
 * Maximum actuator velocity
 *
 * Maximum velocity for bucket actuator (mm/s)
 *
 * @group Bucket Control
 * @unit mm/s
 * @min 10.0
 * @max 500.0
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(BCT_MAX_VEL, 100.0);

/**
 * Maximum actuator acceleration
 *
 * Maximum acceleration for bucket actuator (mm/s²)
 *
 * @group Bucket Control
 * @unit mm/s²
 * @min 50.0
 * @max 1000.0
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(BCT_MAX_ACC, 200.0);

/**
 * Maximum jerk limit
 *
 * Maximum jerk for S-curve trajectory (mm/s³)
 *
 * @group Bucket Control
 * @unit mm/s³
 * @min 100.0
 * @max 5000.0
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(BCT_JERK_LIM, 1000.0);

/**
 * Zeroing fast speed
 *
 * Speed factor for fast movement during zeroing (0.0 to 1.0)
 *
 * @group Bucket Control
 * @min 0.1
 * @max 1.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(BCT_ZERO_FAST, 0.7);

/**
 * Zeroing slow speed
 *
 * Speed factor for slow approach to fine limit (0.0 to 1.0)
 *
 * @group Bucket Control
 * @min 0.01
 * @max 0.2
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(BCT_ZERO_SLOW, 0.05);
