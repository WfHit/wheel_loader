# Build Instructions for Pixhawk 6X Debug QE Board

## Overview
This board configuration provides GPIO-based quadrature encoder debugging capabilities using I2C3 pins as GPIO inputs.

## Key Features
- **Base**: STM32H753IIK6 (same as FMU-v6x)
- **QE Support**: GPIO-based quadrature encoder on I2C3 pins (PA8, PH8)
- **I2C**: I2C1, I2C2, I2C4 functional; I2C3 disabled for GPIO encoder use
- **Debug Optimized**: Simplified configuration for encoder debugging

## Hardware Connections
```
QE0 Phase A: PA8 (I2C3_SCL pin)
QE0 Phase B: PH8 (I2C3_SDA pin)
```

## Build Commands

### Default Configuration
```bash
make holybro_pixhawk-6x-debug-qe_default
```

### Debug Configuration (Minimal)
```bash
make holybro_pixhawk-6x-debug-qe_debug
```

### Bootloader
```bash
make holybro_pixhawk-6x-debug-qe_bootloader
```

## Usage After Flashing

### Start Quadrature Encoder
```bash
# Quadrature encoder auto-starts from init script
# or manually start with:
quad_encoder start -i 0 -d /dev/qe0

# Check status
quad_encoder status

# Monitor encoder data
listener sensor_quad_encoder

# Reset encoder position
quad_encoder reset
```

### Parameters
```bash
# Set encoder parameters
param set QE_PPR_0 1024        # Pulses per revolution
param set QE_INVERT_0 0        # Normal direction
param set QE_UPDATE_RATE 100   # 100 Hz update rate
```

### Debugging
```bash
# Check GPIO configuration
gpio read PA8
gpio read PH8

# Check I2C buses (I2C3 should not be listed)
i2cdetect -l
```

## Pin Compatibility
This configuration is designed to work with external mini boards that provide quadrature encoder signals on I2C3 connector pins.

## Files Modified/Created
- Board configuration: `board_config.h`
- I2C configuration: `i2c.cpp` (I2C3 disabled)
- NuttX board config: `board.h` (I2C3 pins as GPIO)
- Encoder config: `board_quad_encoder_config.c`
- Init scripts: `rc.board_sensors`
