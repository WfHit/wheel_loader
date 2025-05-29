# PX4 Quadrature Encoder Driver

A comprehensive quadrature encoder driver for PX4 that supports multiple encoder types and use cases including wheel odometry, motor control, linear actuators, and rotary position sensing.

## Overview

This driver provides a generic interface to quadrature encoders through the NuttX QEncoder framework while publishing standardized data via uORB topics. It supports multiple hardware implementations and can be easily adapted to different board configurations.

## Features

- **Multiple Hardware Support**: GPIO-based, timer-based, and dedicated encoder peripherals
- **Flexible Configuration**: Runtime parameter configuration and board-specific pin mapping
- **Real-time Performance**: Low-latency interrupt-driven operation with configurable update rates
- **Engineering Units**: Automatic scaling to radians, meters, RPM, etc.
- **Safety Features**: Configurable limits, error detection, and fault reporting
- **uORB Integration**: Standard PX4 message publishing for seamless integration
- **Diagnostics**: Signal quality monitoring and performance statistics

## Supported Hardware

### GPIO-Based Encoders
- Any two GPIO pins can be used for A/B signals
- Optional third pin for index (Z) signal
- Software-based quadrature decoding
- Configurable debouncing and filtering

### Timer-Based Encoders
- Uses hardware timer peripherals in quadrature decoder mode
- Higher resolution and better noise immunity
- Dedicated hardware for count and direction detection
- Platform-specific implementation

### Dedicated Encoder Peripherals
- Dedicated quadrature encoder hardware blocks
- Highest performance option where available
- Hardware-based filtering and error detection

## Board Integration

### HKUST NXT Dual WL Configuration

The driver is preconfigured for the HKUST NXT Dual WL board with two encoders:

- **Encoder 0** (Primary/Left): PC0 (A), PC1 (B)
- **Encoder 1** (Secondary/Right): PA4 (A), PA5 (B)

Pin configuration is defined in:
```
boards/hkust/nxt-dual-wl/src/board_quadencoder.c
```

### Adding New Board Support

1. Create board-specific encoder configuration:
```c
// boards/your_board/src/board_quadencoder.c
static struct quadencoder_config_s g_encoder_config = {
    .pins = {
        .pin_a = YOUR_PIN_A,
        .pin_b = YOUR_PIN_B,
        .pin_z = YOUR_PIN_Z,  // Optional
        .pin_mode = GPIO_INPUT | GPIO_PULLUP,
        .invert_a = false,
        .invert_b = false,
        .invert_z = false
    },
    .hw_type = QUADENCODER_HW_GPIO,
};
```

2. Implement initialization function:
```c
int your_board_quadencoder_initialize(void);
```

3. Add to board configuration header:
```c
// boards/your_board/src/board_config.h
#define BOARD_HAS_QUADENCODER 1
extern int your_board_quadencoder_initialize(void);
```

4. Call from board initialization:
```c
// boards/your_board/src/init.c
#ifdef BOARD_HAS_QUADENCODER
    your_board_quadencoder_initialize();
#endif
```

## Usage

### Starting the Driver

The driver can be started manually or automatically at boot:

```bash
# Manual start
quad_encoder start

# Check status
quad_encoder status

# Stop driver
quad_encoder stop
```

### Automatic Startup

Add to board's `rc.board_extras`:
```bash
if param compare QUADENC_ENABLE 1
then
    quad_encoder start
fi
```

### Parameter Configuration

The driver supports runtime configuration via PX4 parameters:

| Parameter | Description | Default | Range |
|-----------|-------------|---------|-------|
| `QUADENC_ENABLE` | Enable encoder driver | 0 | 0-1 |
| `QUADENC_0_PPR` | Encoder 0 pulses per revolution | 1024 | 64-10000 |
| `QUADENC_0_DIAMETER` | Wheel diameter (m) | 0.1 | 0.01-1.0 |
| `QUADENC_0_INVERT` | Invert direction | 0 | 0-1 |
| `QUADENC_0_SCALING` | Position scaling factor | 1.0 | 0.1-100.0 |
| `QUADENC_FILT_EN` | Enable filtering | 1 | 0-1 |
| `QUADENC_FILT_CUTOFF` | Filter cutoff (Hz) | 10 | 1-100 |
| `QUADENC_PUB_RATE` | Publish rate (Hz) | 50 | 1-1000 |

### uORB Topics

The driver publishes encoder data to the `sensor_quad_encoder` topic:

```c
struct sensor_quad_encoder_s {
    uint64_t timestamp;        // Timestamp (μs)
    uint32_t device_id;        // Device identifier
    int32_t position;          // Raw position (counts)
    int32_t velocity;          // Raw velocity (counts/s)
    float angle;               // Angle (radians)
    float angular_velocity;    // Angular velocity (rad/s)
    float speed;               // Linear speed (m/s)
    uint32_t pulse_count;      // Total pulse count
    uint16_t error_flags;      // Error flags
    uint8_t signal_quality;    // Signal quality (0-100%)
};
```

### Programming Examples

#### Basic Position Reading
```c
#include <uORB/topics/sensor_quad_encoder.h>

int encoder_sub = orb_subscribe(ORB_ID(sensor_quad_encoder));
struct sensor_quad_encoder_s encoder_data;

bool updated;
orb_check(encoder_sub, &updated);
if (updated) {
    orb_copy(ORB_ID(sensor_quad_encoder), encoder_sub, &encoder_data);
    printf("Position: %ld counts, Angle: %.2f deg\n",
           encoder_data.position,
           encoder_data.angle * 180.0f / M_PI);
}
```

#### Motor Control with Position Feedback
```c
// See motor_control_example.c for complete implementation
float target_position = M_PI / 2.0f;  // 90 degrees
float error = target_position - encoder_data.angle;
float output = pid_update(&pid, error, dt);
```

#### Wheel Odometry
```c
// Calculate distance traveled
static float last_position = 0.0f;
float distance_delta = (encoder_data.angle - last_position) * wheel_radius;
last_position = encoder_data.angle;

// Integrate for total distance
total_distance += distance_delta;
```

## Testing and Examples

The driver includes comprehensive test examples:

```bash
# Basic encoder test
quad_encoder_test test

# Motor control example
quad_encoder_test motor

# Linear actuator example
quad_encoder_test linear

# uORB subscription test
quad_encoder_test subscribe
```

### Example Applications

1. **Basic Encoder Test** (`quad_encoder_test test`)
   - Direct device access and parameter configuration
   - Position and velocity reading
   - Angle calculation and display

2. **Motor Control Example** (`quad_encoder_test motor`)
   - Closed-loop position and velocity control
   - PID controller implementation
   - Multiple control phases demonstration

3. **Linear Position Example** (`quad_encoder_test linear`)
   - Linear actuator control with safety limits
   - Homing sequence implementation
   - Position scaling for mechanical systems

## Troubleshooting

### Common Issues

1. **No encoder devices detected**
   - Verify `QUADENC_ENABLE` parameter is set to 1
   - Check board-specific initialization
   - Confirm GPIO pin configuration

2. **Incorrect position readings**
   - Verify encoder wiring (A/B phase order)
   - Check `QUADENC_X_INVERT` parameter
   - Confirm pulses per revolution setting

3. **Noisy velocity measurements**
   - Enable filtering with `QUADENC_FILT_EN`
   - Adjust filter cutoff frequency
   - Check encoder signal quality

4. **Performance issues**
   - Reduce publish rate if CPU usage is high
   - Use timer-based encoders for better performance
   - Check interrupt priority settings

### Debug Commands

```bash
# Show detailed status
quad_encoder status

# List all encoder devices
ls /dev/qe*

# Monitor uORB topic
listener sensor_quad_encoder

# Check parameters
param show QUADENC*
```

### Log Analysis

Enable encoder logging to analyze performance:
```bash
# Enable logging
param set SDLOG_DIRS_MAX 2
param set SDLOG_ENCODER 1

# Analyze in flight review
# Look for position discontinuities
# Check velocity noise levels
# Verify timing consistency
```

## Configuration Reference

### Kconfig Options

The driver behavior can be customized at build time:

- `DRIVERS_QUAD_ENCODER`: Enable driver support
- `QUADENCODER_GPIO_SUPPORT`: GPIO-based encoders
- `QUADENCODER_TIMER_SUPPORT`: Timer-based encoders
- `QUADENCODER_MAX_INSTANCES`: Maximum encoder count
- `QUADENCODER_VELOCITY_CALC`: Velocity calculation
- `QUADENCODER_FILTER_ENABLE`: Digital filtering
- `QUADENCODER_INDEX_SUPPORT`: Index signal support
- `QUADENCODER_DIAGNOSTICS`: Enhanced diagnostics

### Hardware Requirements

- **GPIO encoders**: 2-3 GPIO pins per encoder
- **Timer encoders**: 1 timer peripheral per encoder
- **Memory**: ~2KB flash, ~512B RAM per encoder instance
- **CPU**: <1% at 100Hz update rate per encoder

### Performance Characteristics

| Mode | Max Update Rate | Latency | CPU Usage |
|------|----------------|---------|-----------|
| GPIO | 1000 Hz | 1-2 ms | 0.5-2% |
| Timer | 10000 Hz | <100 μs | 0.1-0.5% |
| Dedicated | 50000 Hz | <50 μs | <0.1% |

## Contributing

When contributing to this driver:

1. Follow PX4 coding standards
2. Add appropriate unit tests
3. Update documentation for new features
4. Test on target hardware
5. Consider backward compatibility

## License

This driver is licensed under the BSD 3-Clause License, consistent with the PX4 project licensing.
