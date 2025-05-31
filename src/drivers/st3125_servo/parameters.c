/**
 * @file parameters.c
 *
 * ST3125 Smart Servo Driver parameters
 *
 * @author GitHub Copilot
 */

/**
 * ST3125 Serial baudrate
 *
 * Baudrate for serial communication with ST3125 servos.
 * Valid values: 9600, 19200, 38400, 57600, 115200, 500000, 1000000
 *
 * @min 9600
 * @max 1000000
 * @value 9600 9600 baud
 * @value 19200 19200 baud
 * @value 38400 38400 baud
 * @value 57600 57600 baud
 * @value 115200 115200 baud
 * @value 500000 500000 baud
 * @value 1000000 1000000 baud
 * @group ST3125 Servo
 */
PARAM_DEFINE_INT32(ST3125_BAUDRATE, 115200);

/**
 * ST3125 Update rate
 *
 * Rate at which servo feedback is updated and commands are sent.
 *
 * @unit Hz
 * @min 10
 * @max 400
 * @value 50 50 Hz
 * @value 100 100 Hz
 * @value 200 200 Hz
 * @value 333 333 Hz
 * @group ST3125 Servo
 */
PARAM_DEFINE_INT32(ST3125_UPD_RAT, 100);

/**
 * ST3125 Minimum position
 *
 * Minimum position limit for all servos (in servo units).
 * ST3125 range: 0-4095 (0-300 degrees)
 *
 * @min 0
 * @max 4095
 * @group ST3125 Servo
 */
PARAM_DEFINE_INT32(ST3125_POS_MIN, 0);

/**
 * ST3125 Maximum position
 *
 * Maximum position limit for all servos (in servo units).
 * ST3125 range: 0-4095 (0-300 degrees)
 *
 * @min 0
 * @max 4095
 * @group ST3125 Servo
 */
PARAM_DEFINE_INT32(ST3125_POS_MAX, 4095);

/**
 * ST3125 Torque enable
 *
 * Enable torque output for all servos on startup.
 * 0: Disable torque, 1: Enable torque
 *
 * @min 0
 * @max 1
 * @value 0 Disable
 * @value 1 Enable
 * @group ST3125 Servo
 */
PARAM_DEFINE_INT32(ST3125_TORQUE_EN, 1);

/**
 * ST3125 Position P gain
 *
 * Position control P gain for all servos.
 * Higher values increase position responsiveness but may cause oscillation.
 *
 * @min 0
 * @max 32767
 * @group ST3125 Servo
 */
PARAM_DEFINE_INT32(ST3125_POS_P, 1024);

/**
 * ST3125 Position I gain
 *
 * Position control I gain for all servos.
 * Helps eliminate steady-state error.
 *
 * @min 0
 * @max 32767
 * @group ST3125 Servo
 */
PARAM_DEFINE_INT32(ST3125_POS_I, 0);

/**
 * ST3125 Position D gain
 *
 * Position control D gain for all servos.
 * Provides damping to reduce oscillation.
 *
 * @min 0
 * @max 32767
 * @group ST3125 Servo
 */
PARAM_DEFINE_INT32(ST3125_POS_D, 0);

/**
 * ST3125 Velocity limit
 *
 * Maximum velocity limit for all servos.
 * 0 = no limit, 1-1023 = velocity limit in servo units
 *
 * @min 0
 * @max 1023
 * @group ST3125 Servo
 */
PARAM_DEFINE_INT32(ST3125_VEL_LIMIT, 0);

/**
 * ST3125 Current limit
 *
 * Maximum current limit for all servos.
 * Protects servos from overcurrent conditions.
 *
 * @min 0
 * @max 2000
 * @group ST3125 Servo
 */
PARAM_DEFINE_INT32(ST3125_CUR_LIMIT, 1000);

/**
 * ST3125 Temperature limit
 *
 * Maximum operating temperature limit for all servos.
 * Servo will reduce torque or shutdown if exceeded.
 *
 * @min 0
 * @max 100
 * @group ST3125 Servo
 */
PARAM_DEFINE_INT32(ST3125_TMP_LIMIT, 80);

/**
 * ST3125 Auto-discovery
 *
 * Enable automatic discovery of connected servos.
 * 0: Disable, 1: Enable
 *
 * @min 0
 * @max 1
 * @value 0 Disable
 * @value 1 Enable
 * @group ST3125 Servo
 */
PARAM_DEFINE_INT32(ST3125_AUTO_DISC, 1);

/**
 * ST3125 Status publishing
 *
 * Enable publishing of detailed servo status information.
 * 0: Disable, 1: Enable
 *
 * @min 0
 * @max 1
 * @value 0 Disable
 * @value 1 Enable
 * @group ST3125 Servo
 */
PARAM_DEFINE_INT32(ST3125_PUB_STAT, 1);

/**
 * ST3125 Debug level
 *
 * Debug output level for ST3125 driver.
 * 0: None, 1: Errors, 2: Warnings, 3: Info, 4: Debug
 *
 * @min 0
 * @max 4
 * @value 0 None
 * @value 1 Errors only
 * @value 2 Errors + Warnings
 * @value 3 Errors + Warnings + Info
 * @value 4 All (Debug)
 * @group ST3125 Servo
 */
PARAM_DEFINE_INT32(ST3125_DEBUG, 1);
