# Holybro Pixhawk 6X Debug QE

This is a specialized board configuration based on the PX4 FMU-v6x hardware, specifically configured for quadrature encoder debugging and NuttX driver development.

## Key Features

- **Base Hardware**: STM32H753IIK6 processor (same as FMU-v6x)
- **Quad Encoder Debug**: Enhanced quadrature encoder support with multiple timer configurations
- **Modified I2C Pins**: External I2C bus pins modified for signal input/debugging
- **NuttX Driver Debug**: Optimized for low-level driver development and debugging

## Quadrature Encoder Configuration

### Hardware Setup
- **GPIO Mode**: Uses GPIO-based quadrature encoder driver instead of hardware timers
- **QE0**: GPIO-based encoder on I2C3 pins (/dev/qe0)
- **Mini Board Compatible**: Pin configuration suitable for external mini board connections

### Modified I2C Pin Configuration for GPIO Encoders
- **I2C3 Pins as QE0**: PA8 (SCL) -> Phase A, PH8 (SDA) -> Phase B

### Default Parameters
- `QE_UPDATE_RATE`: 100 (100 Hz update rate)
- `QE_PPR_0`: 1024 (Default pulses per revolution for QE0)
- `QE_INVERT_0`: 0 (Normal direction for QE0)

### Usage
The GPIO-based quad encoder drivers can be started with:

```bash
# Start GPIO quad encoder on I2C3 pins
quad_encoder start -i 0 -d /dev/qe0

# Check encoder status
quad_encoder status

# Monitor encoder data
listener sensor_quad_encoder

# Reset encoder positions
quad_encoder reset
```

### Debug Features
- Enhanced logging for GPIO quadrature encoder operations
- Direct GPIO register access for debugging
- I2C pins repurposed as GPIO inputs for encoder signals
- Support for oscilloscope probing on encoder signals
- Mini board compatible pin configuration

### Pin Assignments
```
# GPIO Quadrature Encoder Pins (using I2C3 pins only)
QE0_PHASE_A (PA8) - I2C3_SCL repurposed as GPIO input
QE0_PHASE_B (PH8) - I2C3_SDA repurposed as GPIO input

# Original I2C functionality disabled on these pins
# I2C3: PA8 (SCL), PH8 (SDA) -> GPIO mode for QE0
```
