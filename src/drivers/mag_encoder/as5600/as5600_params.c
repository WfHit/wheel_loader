/**
 * AS5600 magnetic encoder enable
 *
 * Enable AS5600 magnetic rotary position sensor
 *
 * @reboot_required true
 *
 * @group Sensors
 * @value 0 Disabled
 * @value 1 Enabled
 */
PARAM_DEFINE_INT32(SENS_EN_AS5600, 0);

/**
 * AS5600 I2C bus
 *
 * I2C bus for AS5600 magnetic encoder
 *
 * @reboot_required true
 *
 * @group Sensors
 * @value -1 Disabled
 * @value 1 I2C Bus 1
 * @value 2 I2C Bus 2
 * @value 3 I2C Bus 3
 * @value 4 I2C Bus 4
 */
PARAM_DEFINE_INT32(AS5600_I2C_BUS, -1);

/**
 * AS5600 angle offset
 *
 * Angular offset for AS5600 sensor in radians
 *
 * @group Sensors
 * @unit rad
 * @min -3.14159
 * @max 3.14159
 * @decimal 4
 */
PARAM_DEFINE_FLOAT(AS5600_OFFSET, 0.0f);
