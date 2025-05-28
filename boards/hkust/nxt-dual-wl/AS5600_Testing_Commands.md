# AS5600 Testing Commands for HKUST nxt-dual-wl

This document contains all the necessary commands to test and verify the AS5600 magnetic rotary position sensor configuration on the HKUST nxt-dual-wl wheel loader board.

## Prerequisites

1. **Flash the firmware** to the HKUST nxt-dual-wl board:
   ```bash
   # Build firmware (if not already built)
   make hkust_nxt-dual-wl

   # Flash firmware to board (adjust port as needed)
   make hkust_nxt-dual-wl upload
   ```

2. **Connect to PX4 console** via serial or MAVLink:
   ```bash
   # Serial connection (adjust port as needed)
   miniterm.py /dev/ttyUSB0 57600

   # Or MAVLink shell
   ./Tools/mavlink_shell.py
   ```

## 1. Parameter Verification

### Check AS5600 Parameters
Verify that the AS5600 parameters are correctly set:

```bash
# Check if AS5600 is enabled (should return: 1)
param show SENS_EN_AS5600

# Check I2C bus configuration (should return: 1)
param show AS5600_I2C_BUS

# Check angle offset (should return: 0.000000)
param show AS5600_OFFSET
```

### Modify Parameters (if needed)
```bash
# Enable AS5600 sensor
param set SENS_EN_AS5600 1

# Set I2C bus (1 for I2C Bus 1, 4 for I2C Bus 4)
param set AS5600_I2C_BUS 1

# Set angle offset (example: 45 degrees = 0.785398 radians)
param set AS5600_OFFSET 0.785398

# Save parameters to persistent storage
param save

# Reboot to apply changes
reboot
```

## 2. I2C Bus Testing

### Scan for I2C Devices
Check if the AS5600 is detected on the I2C bus:

```bash
# Scan I2C Bus 1 for devices
i2cdetect -X 1

# Scan I2C Bus 4 for devices (if using I2C4)
i2cdetect -X 4

# Look for device at address 0x36 (AS5600 default address)
```

Expected output should show device at address `0x36`:
```
     0  1  2  3  4  5  6  7  8  9  a  b  c  d  e  f
00:          -- -- -- -- -- -- -- -- -- -- -- -- --
10: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
20: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
30: -- -- -- -- -- -- 36 -- -- -- -- -- -- -- -- --
40: -- -- -- -- -- -- -- -- -- -- -- -- -- -- -- --
...
```

## 3. AS5600 Driver Testing

### Check Driver Status
```bash
# Check AS5600 driver status
as5600 status
```

Expected output when running:
```
INFO  [as5600] Running on I2C Bus 1
INFO  [as5600] Device detected at address 0x36
INFO  [as5600] Sensor initialized successfully
```

### Manual Driver Control
```bash
# Start AS5600 driver manually
as5600 start

# Stop AS5600 driver
as5600 stop

# Restart AS5600 driver
as5600 stop
as5600 start
```

### Driver with Specific Parameters
```bash
# Start AS5600 on specific I2C bus
as5600 start -X -b 1    # Start on I2C Bus 1
as5600 start -X -b 4    # Start on I2C Bus 4

# Start with specific I2C address (if modified)
as5600 start -X -a 0x36
```

## 4. Sensor Data Verification

### Monitor Sensor Output
```bash
# Listen to sensor_mag topic (AS5600 publishes here)
listener sensor_mag

# Listen with specific update rate
listener sensor_mag -r 10  # 10Hz update rate
```

Expected output format:
```
TOPIC: sensor_mag
timestamp: 1234567890
device_id: 12345
x: 123.456         # Raw magnetic field X
y: 234.567         # Raw magnetic field Y
z: 345.678         # Raw magnetic field Z
temperature: 25.0  # Temperature (if available)
```

### Monitor uORB Topic Activity
```bash
# Show uORB topic statistics
uorb top

# Look for sensor_mag topic and check:
# - #SUB: Number of subscribers
# - #MSG: Message count
# - RATE: Publishing rate
```

### Check All Available Topics
```bash
# List all active uORB topics
uorb list

# Filter for sensor topics
uorb list | grep sensor
```

## 5. Data Analysis and Calibration

### Raw Data Collection
```bash
# Log raw sensor data to file
listener sensor_mag > /fs/microsd/as5600_data.txt

# Or use logger for comprehensive data logging
logger start -f -t -b 200 -p sensor_mag
# ... collect data by rotating magnet/wheel ...
logger stop
```

### Real-time Angle Monitoring
```bash
# Create a simple script to monitor angle changes
while true; do
    echo "Current sensor reading:"
    listener sensor_mag -n 1
    sleep 1
done
```

## 6. Advanced Testing

### Test Sensor Response
```bash
# Test sensor response by physically rotating the magnet
# Monitor output while slowly rotating the wheel/magnet 360°
listener sensor_mag -r 20  # 20Hz for smooth monitoring
```

### Performance Testing
```bash
# Check system performance with AS5600 active
top

# Monitor CPU usage and memory
perf

# Check for any error messages
dmesg
```

### Temperature Testing
```bash
# If temperature sensing is enabled, monitor temperature
listener sensor_mag | grep temperature
```

## 7. Troubleshooting Commands

### Common Issues and Diagnostic Commands

#### AS5600 Not Detected
```bash
# Check I2C bus configuration
cat /proc/bus/i2c/devices

# Verify I2C bus is enabled in NuttX
mount
ls /dev/i2c*

# Check for I2C errors
dmesg | grep i2c
```

#### No Sensor Data
```bash
# Check if driver is actually running
ps

# Check for driver errors
dmesg | grep as5600

# Verify parameter settings
param show | grep AS5600
param show | grep SENS_EN
```

#### Incorrect Readings
```bash
# Check magnet alignment - readings should change smoothly
# as magnet rotates. Look for:
# - Continuous value changes during rotation
# - Values returning to same position after 360° rotation
# - No sudden jumps or discontinuities

# Test with known positions
listener sensor_mag -n 5  # Take 5 readings at fixed position
# Rotate 90 degrees
listener sensor_mag -n 5  # Compare readings
```

### System Information
```bash
# Check PX4 version and build info
ver all

# Check board information
ver hwarch

# Check available memory
free

# Check mounted filesystems
mount
```

## 8. Production Testing Script

Create a comprehensive test script:

```bash
#!/bin/sh
# AS5600 Production Test Script

echo "=== AS5600 Production Test ==="
echo "1. Checking parameters..."
param show SENS_EN_AS5600
param show AS5600_I2C_BUS
param show AS5600_OFFSET

echo "2. Scanning I2C bus..."
i2cdetect -X 1

echo "3. Checking driver status..."
as5600 status

echo "4. Testing sensor data (10 samples)..."
listener sensor_mag -n 10

echo "5. Performance check..."
top | head -10

echo "=== Test Complete ==="
```

## Expected Results Summary

| Test | Expected Result | Pass/Fail Criteria |
|------|----------------|-------------------|
| Parameter Check | SENS_EN_AS5600=1, AS5600_I2C_BUS=1 | Parameters match configuration |
| I2C Detection | Device at 0x36 on Bus 1 | I2C device detected |
| Driver Status | "Running on I2C Bus 1" | Driver reports running |
| Sensor Data | Continuous sensor_mag messages | Data rate > 1Hz |
| Rotation Test | Smooth value changes during 360° rotation | Values change predictably |
| Temperature | Reasonable temperature reading (if enabled) | 0°C < temp < 60°C |

## Notes

- **Magnet Placement**: Ensure magnet is properly positioned over AS5600 sensor
- **Power Supply**: Verify 3.3V or 5V power supply to AS5600
- **I2C Pull-ups**: Ensure proper I2C pull-up resistors are installed
- **Update Rate**: Default update rate is typically 100Hz
- **Resolution**: AS5600 provides 12-bit resolution (4096 positions per revolution)
- **Range**: Full 360° rotation detection with 0.087° accuracy

## Support

For additional support or issues:
1. Check PX4 documentation: https://docs.px4.io/
2. Review AS5600 datasheet for hardware specifications
3. Verify electrical connections and power supply
4. Check for electromagnetic interference sources
