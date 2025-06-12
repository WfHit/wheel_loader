/**
 * Wheel velocity P gain
 * @group Wheel Controller
 * @min 0.0
 * @max 10.0
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(WC_P_GAIN, 1.0f);

/**
 * Wheel velocity I gain
 * @group Wheel Controller
 * @min 0.0
 * @max 5.0
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(WC_I_GAIN, 0.1f);

/**
 * Wheel velocity D gain
 * @group Wheel Controller
 * @min 0.0
 * @max 1.0
 * @decimal 3
 */
PARAM_DEFINE_FLOAT(WC_D_GAIN, 0.01f);

/**
 * Wheel velocity I max
 * @group Wheel Controller
 * @min 0.0
 * @max 100.0
 * @decimal 1
 */
PARAM_DEFINE_FLOAT(WC_I_MAX, 10.0f);

/**
 * Maximum wheel speed
 * @group Wheel Controller
 * @min 10.0
 * @max 500.0
 * @decimal 1
 * @unit rpm
 */
PARAM_DEFINE_FLOAT(WC_MAX_SPEED, 200.0f);

/**
 * Speed ramp rate
 * @group Wheel Controller
 * @min 1.0
 * @max 1000.0
 * @decimal 1
 * @unit
 */
PARAM_DEFINE_FLOAT(WC_RAMP_RATE, 100.0f);

/**
 * Enable traction control
 * @group Wheel Controller
 * @boolean
 */
PARAM_DEFINE_INT32(WC_TRACT_EN, 1);

/**
 * Slip threshold for traction control
 * @group Wheel Controller
 * @min 0.1
 * @max 10.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(WC_SLIP_THRS, 2.0f);

/**
 * Current limit
 * @group Wheel Controller
 * @min 1.0
 * @max 50.0
 * @decimal 1
 * @unit A
 */
PARAM_DEFINE_FLOAT(WC_CURR_LIM, 15.0f);

/**
 * Gear ratio between motor and wheel
 * @group Wheel Controller
 * @min 1.0
 * @max 200.0
 * @decimal 2
 */
PARAM_DEFINE_FLOAT(WC_GEAR_RAT, 50.0f);

/**
 * Encoder counts per revolution
 * @group Wheel Controller
 * @min 100
 * @max 10000
 */
PARAM_DEFINE_INT32(WC_ENC_CPR, 2048);
