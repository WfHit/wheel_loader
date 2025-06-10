# AS5600 Configuration Guide for HKUST nxt-dual-wl Board

## Overview
This guide documents the AS5600 magnetic rotary position sensor configuration for the HKUST nxt-dual-wl wheel loader board. The AS5600 provides 12-bit angular position measurements for wheel loader applications.

## Hardware Configuration
- **Board**: HKUST nxt-dual-wl (STM32H743VI-based)
- **Sensor**: AS5600 magnetic rotary encoder
- **Interface**: I2C communication
- **Available I2C buses**: I2C1 and I2C4

## Parameter Configuration

### AS5600 Parameters
The following parameters control AS5600 operation:

| Parameter | Description | Default | Configured Value | Range |
|-----------|-------------|---------|------------------|-------|
| `SENS_EN_AS5600` | Enable AS5600 sensor | 0 (Disabled) | 1 (Enabled) | 0-1 |
| `AS5600_I2C_BUS` | I2C bus selection | -1 (Disabled) | 1 (I2C Bus 1) | 1-4 |
| `AS5600_OFFSET` | Angle offset (radians) | 0.0 | 0.0 | -3.14159 to 3.14159 |

### Configuration Files Modified
1. **Board Defaults** (`boards/hkust/nxt-dual-wl/init/rc.board_defaults`):
   - Set `SENS_EN_AS5600` to 1 (enabled)
   - Set `AS5600_I2C_BUS` to 1 (I2C Bus 1)
   - Keep `AS5600_OFFSET` at 0.0 (can be adjusted as needed)

2. **Sensor Startup** (`ROMFS/px4fmu_common/init.d/rc.sensors`):
   - Added AS5600 conditional startup based on `SENS_EN_AS5600` parameter
   - Sensor starts automatically if enabled

3. **Board Sensors** (`boards/hkust/nxt-dual-wl/init/rc.board_sensors`):
   - Includes AS5600 startup command for board-specific initialization

## I2C Bus Selection
The AS5600 is configured to use **I2C Bus 1** based on:
- Available I2C buses on HKUST nxt-dual-wl: I2C1 and I2C4
- I2C1 is typically used for primary sensors
- AS5600 default I2C address: 0x36

## Build Status
✅ **Build Successful**: Firmware compiled without errors
- Memory usage: 98.87% flash utilization
- All AS5600 configurations included in ROMFS
- Firmware file: `hkust_nxt-dual-wl_default.px4`

## Testing and Verification

### 1. Parameter Verification
After flashing the firmware, verify parameters are set correctly:
```bash
# Connect to PX4 console and check parameters
param show SENS_EN_AS5600    # Should show: 1
param show AS5600_I2C_BUS    # Should show: 1
param show AS5600_OFFSET     # Should show: 0.000000
```

### 2. I2C Device Detection
Check if AS5600 is detected on I2C bus:
```bash
# Scan I2C bus 1 for devices
i2cdetect -X 1
# Look for device at address 0x36
```

### 3. AS5600 Driver Status
Check AS5600 driver status:
```bash
# Check if AS5600 driver is running
as5600 status

# Start AS5600 manually if needed
as5600 start

# Stop AS5600 driver
as5600 stop
```

### 4. Sensor Data Verification
Monitor AS5600 sensor output:
```bash
# Listen to AS5600 sensor data
listener sensor_mag

# Check uORB topics
uorb top sensor_mag
```

## Troubleshooting

### Common Issues and Solutions

1. **AS5600 not detected**:
   - Check I2C wiring and connections
   - Verify I2C bus configuration (`AS5600_I2C_BUS` parameter)
   - Use `i2cdetect` to scan for device

2. **No sensor data**:
   - Ensure `SENS_EN_AS5600` parameter is set to 1
   - Check AS5600 driver status with `as5600 status`
   - Verify magnet placement and alignment

3. **Incorrect angle readings**:
   - Adjust `AS5600_OFFSET` parameter for calibration
   - Check magnet position relative to sensor
   - Verify sensor orientation and mounting

### Parameter Adjustment
To modify AS5600 parameters during runtime:
```bash
# Enable/disable AS5600
param set SENS_EN_AS5600 1    # Enable
param set SENS_EN_AS5600 0    # Disable

# Change I2C bus (requires reboot)
param set AS5600_I2C_BUS 4    # Use I2C Bus 4

# Adjust angle offset for calibration
param set AS5600_OFFSET 0.785398  # 45 degrees in radians

# Save parameters to persistent storage
param save
```

## Integration Notes

### Wheel Loader Application
The AS5600 magnetic encoder is particularly suitable for wheel loader applications:
- **360° rotation sensing** for steering angle measurement
- **12-bit resolution** (0.087° accuracy)
- **Magnetic sensing** - no physical contact, immune to dust/dirt
- **I2C interface** - easy integration with flight controller

### Data Output
- **Topic**: `sensor_mag` (uORB topic)
- **Update Rate**: Configurable, typically 100Hz
- **Data Format**: Raw magnetic field values and calculated angle
- **Units**: Radians (angle), magnetic field strength (raw values)

## Hardware Connections
Ensure proper AS5600 connections to HKUST nxt-dual-wl board:
- **VCC**: 3.3V or 5V power supply
- **GND**: Ground connection
- **SDA**: I2C data line (I2C1 SDA pin)
- **SCL**: I2C clock line (I2C1 SCL pin)
- **DIR**: Direction pin (optional, can be left floating)

## Revision History
- **v1.0**: Initial AS5600 configuration for HKUST nxt-dual-wl
  - Enabled AS5600 driver
  - Configured I2C Bus 1
  - Set default parameters
  - Integrated with sensor startup scripts
