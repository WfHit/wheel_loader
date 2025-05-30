/**
 * Quadrature Encoder Parameters
 *
 * @group Quadrature Encoder
 */

/**
 * Encoder channel A GPIO pin
 *
 * @group Quadrature Encoder
 * @unit pin
 * @decimal 0
 * @min -1
 * @max 255
 */
PARAM_DEFINE_INT32(QUADENC_PIN_A, 0);

/**
 * Encoder channel B GPIO pin
 *
 * @group Quadrature Encoder
 * @unit pin
 * @decimal 0
 * @min -1
 * @max 255
 */
PARAM_DEFINE_INT32(QUADENC_PIN_B, 1);

/**
 * Encoder index/Z GPIO pin
 *
 * @group Quadrature Encoder
 * @unit pin
 * @decimal 0
 * @min -1
 * @max 255
 * @value -1 Disabled
 */
PARAM_DEFINE_INT32(QUADENC_PIN_Z, -1);

/**
 * Encoder counting mode
 *
 * @group Quadrature Encoder
 * @value 1 X1 mode (count A edges only)
 * @value 2 X2 mode (count A and B edges)
 * @value 4 X4 mode (count all edges)
 */
PARAM_DEFINE_INT32(QUADENC_MODE, 4);

/**
 * Encoder pulses per revolution
 *
 * @group Quadrature Encoder
 * @unit pulses
 * @decimal 0
 * @min 1
 * @max 10000
 */
PARAM_DEFINE_INT32(QUADENC_PPR, 1024);

/**
 * Wheel radius
 *
 * For wheel applications, set radius to calculate linear distance/speed
 *
 * @group Quadrature Encoder
 * @unit m
 * @decimal 3
 * @min 0.0
 * @max 2.0
 */
PARAM_DEFINE_FLOAT(QUADENC_WHEEL_R, 0.0f);

/**
 * Enable quadrature encoder
 *
 * @group Quadrature Encoder
 * @boolean
 */
PARAM_DEFINE_INT32(QUADENC_EN, 0);
