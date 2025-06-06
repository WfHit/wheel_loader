# AS5600 Magnetic Encoder Configuration for HKUST nxt-dual-wl Board

## Overview
The AS5600 magnetic rotary position sensor has been configured for the HKUST nxt-dual-wl wheel loader board. This sensor provides 12-bit angular position measurements and is commonly used for measuring boom angles, bucket positions, or other rotary applications in wheel loaders.

## Hardware Configuration
- **I2C Address**: 0x36 (fixed)
- **I2C Bus**: I2C1 (configured via parameter)
- **Data Rate**: 50 Hz
- **Resolution**: 12-bit (4096 steps per revolution)
- **Angular Range**: 0° to 360° (0 to 2π radians)

## Parameters
The following parameters have been configured with default values for the HKUST nxt-dual-wl board:

### SENS_EN_AS5600 (Enable/Disable)
- **Default**: 1 (Enabled)
- **Description**: Enable AS5600 magnetic rotary position sensor
- **Values**:
  - 0: Disabled
  - 1: Enabled
- **Reboot Required**: Yes

### AS5600_I2C_BUS (I2C Bus Selection)
- **Default**: 1 (I2C Bus 1)
- **Description**: I2C bus for AS5600 magnetic encoder
- **Values**:
  - -1: Disabled
  - 1: I2C Bus 1 (recommended for HKUST nxt-dual-wl)
  - 2: I2C Bus 2
  - 3: I2C Bus 3
  - 4: I2C Bus 4 (also available on HKUST nxt-dual-wl)
- **Reboot Required**: Yes

### AS5600_OFFSET (Angular Offset)
- **Default**: 0.0 radians
- **Description**: Angular offset for AS5600 sensor in radians
- **Range**: -3.14159 to 3.14159 (-180° to +180°)
- **Unit**: rad
- **Precision**: 4 decimal places
- **Reboot Required**: No

## Usage

### Starting the Driver
The driver will automatically start when the system boots if `SENS_EN_AS5600` is set to 1. You can also manually control the driver:

```bash
# Start the driver
as5600 start

# Stop the driver
as5600 stop

# Check driver status
as5600 status

# Test the driver
as5600 test

# Reset the driver
as5600 reset

# Print driver information
as5600 info
```

### Monitoring Sensor Data
The AS5600 sensor publishes data to the `sensor_mag_encoder` uORB topic at 50 Hz. The published data includes:

- **raw_angle**: Raw 12-bit angle value (0-4095)
- **angle**: Converted angle in radians (0 to 2π) with offset applied
- **magnitude**: Magnetic field strength measurement
- **automatic_gain_control**: AGC value
- **magnet_detected**: Boolean indicating if magnet is detected
- **magnet_too_strong**: Boolean indicating if magnetic field is too strong
- **magnet_too_weak**: Boolean indicating if magnetic field is too weak
- **error_count**: Communication error counter

### Parameter Configuration
You can modify parameters using the PX4 console:

```bash
# Enable the sensor
param set SENS_EN_AS5600 1

# Set I2C bus to bus 4 (if needed)
param set AS5600_I2C_BUS 4

# Set angular offset (example: 90 degrees = π/2 radians)
param set AS5600_OFFSET 1.5708

# Save parameters
param save

# Reboot required for bus and enable changes
reboot
```

### Troubleshooting

#### Sensor Not Detected
1. Check wiring and I2C bus configuration
2. Verify magnet is properly positioned near the sensor
3. Use `i2cdetect` to scan for devices on the I2C bus:
   ```bash
   i2cdetect -a 1  # For I2C bus 1
   ```

#### Poor Signal Quality
1. Check magnet positioning - it should be centered over the sensor
2. Verify magnet strength using the status readings:
   - `magnet_too_weak = true`: Move magnet closer or use stronger magnet
   - `magnet_too_strong = true`: Move magnet further or use weaker magnet
   - `magnet_detected = false`: Check magnet presence and positioning

#### Communication Errors
1. Check I2C bus wiring and connections
2. Verify correct I2C bus parameter setting
3. Check for I2C bus conflicts with other devices

## Installation Requirements
The AS5600 driver is already compiled and enabled in the HKUST nxt-dual-wl firmware build. No additional installation steps are required.

## Technical Specifications
- **Operating Voltage**: 3.3V or 5V
- **Current Consumption**: ~6.5mA
- **Temperature Range**: -40°C to +125°C
- **Magnetic Field Range**: 8-50 mT (80-500 Gauss)
- **Angular Accuracy**: ±0.09° (typical)
- **Hysteresis**: ±0.05° (typical)
- **I2C Speed**: Up to 1 MHz (configured for 400 kHz)
