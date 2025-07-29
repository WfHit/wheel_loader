# Wheel Loader RC Input Module

## Overview

The Wheel Loader RC Input module provides remote control capability for the wheel loader system using SBUS RC protocol. It converts RC transmitter input into wheel loader control commands with comprehensive safety features.

## Features

- **SBUS RC Input Processing**: Full support for SBUS protocol RC receivers
- **Multi-Channel Control**: Chassis movement, boom, bucket, and steering control
- **Safety Systems**: Emergency stop, failsafe, signal validation, and timeout detection
- **Traction Control Integration**: Dynamic slip-based power reduction
- **Configurable Parameters**: Adjustable limits, deadzone, and timeout settings

## System Architecture

### Hardware Configuration
- **2 H-Bridge Devices**: Each with 2 channels (total 4 channels)
  - **Front H-Bridge**: Located on NXT front board
    - Channel 1: Front axle drive motor
    - Channel 2: Bucket hydraulic control
  - **Rear H-Bridge**: Located on NXT rear board  
    - Channel 1: Rear axle drive motor
    - Channel 2: Boom hydraulic control

### Control Mapping
- **Front/Rear Axles**: Controlled as units (not individual wheels)
  - Front axle: Driven by front motor via front H-bridge
  - Rear axle: Driven by rear motor via rear H-bridge
- **Hydraulic Controls**:
  - **Boom**: Velocity speed control (rate-based)
  - **Bucket**: Angle control (position-based)

## RC Channel Mapping

| Channel | Function | Range | Description |
|---------|----------|-------|-------------|
| 0 | Steering | ±0.7 rad (±40°) | Front axle steering angle |
| 2 | Throttle | ±5.0 rad/s | Forward/reverse chassis speed (front/rear axles) |
| 4 | Boom | ±1.0 rad/s | Boom velocity control (rate-based) |
| 5 | Bucket | ±0.5 rad | Bucket angle control (position-based) |
| 6 | E-Stop | Digital | Emergency stop (active low <1400us) |
| 7 | Mode | Digital | Control mode switch (future) |

## Quick Start

### 1. Hardware Setup
- Connect SBUS RC receiver to X7+ WL board SBUS input
- Verify RC transmitter is bound and configured for SBUS output
- Ensure emergency stop switch is configured on channel 6

### 2. Enable Module
```bash
# Enable RC input processing
param set WL_RC_ENABLE 1

# Start the module
wheel_loader_rc_input start

# Verify status
wheel_loader_rc_input status
```

### 3. Parameter Configuration
```bash
# Set maximum speeds and limits
param set WL_RC_MAX_SPEED 3.0      # Reduce max speed for safety
param set WL_RC_MAX_STEER 0.5      # Reduce max steering angle
param set WL_RC_DEADZONE 0.08      # Increase deadzone if needed
param set WL_RC_TIMEOUT 1.5        # Increase failsafe timeout

# Save parameters
param save
```

## Safety Features

### Emergency Stop
- **RC Channel 6**: Emergency stop when switch position <1400us
- **Immediate Response**: All motion stops within 100ms
- **Manual Reset**: Required after emergency stop activation

### Failsafe System
- **Signal Loss Detection**: Activates after 1.0s of no valid RC input
- **Automatic Stop**: All motion stops, emergency brake engages
- **Visual/Audio Alerts**: Status messages via PX4 logging system

### Input Validation
- **Signal Quality**: RSSI and frame loss monitoring
- **Range Checking**: Input values validated against 1000-2000us range
- **Timeout Protection**: Stale command rejection

## Traction Control Integration

The module integrates with the slip estimation system for enhanced safety:

### Slip Response Levels
- **Normal Operation**: 100% power (no slip detected)
- **Light Slip**: 70% power reduction (slip detected)
- **Critical Slip**: 30% power reduction (critical slip detected)
- **High Variance**: Additional 20% reduction for unstable conditions

### Recovery Behavior
- **Progressive Recovery**: Gradual power restoration as slip conditions improve
- **Adaptive Response**: Real-time adjustment based on terrain and load

## Configuration Parameters

### Speed and Motion Limits
```bash
WL_RC_MAX_SPEED     # Maximum axle speed (rad/s) [0.5-10.0]
WL_RC_MAX_STEER     # Maximum steering angle (rad) [0.1-1.5] 
WL_RC_MAX_BOOM      # Maximum boom velocity (rad/s) [0.1-2.0]
WL_RC_MAX_BUCKET    # Maximum bucket angle (rad) [0.1-1.5]
```

### Input Processing
```bash
WL_RC_DEADZONE      # Input deadzone [0.0-0.2]
WL_RC_TIMEOUT       # Failsafe timeout (s) [0.1-5.0]
WL_RC_ENABLE        # Enable/disable module [0/1]
```

## System Integration

### Message Flow
```
RC Transmitter → input_rc → wheel_loader_rc_input → wheel_loader_command
                                                          ↓
wheel_loader_controller → subsystem commands → uorb_uart_bridge
                    ↓
            NXT Front/Rear Boards → Motor Control
```

### Related Modules
- **wheel_loader_controller**: Main system coordinator
- **slip_estimator**: Provides traction control data  
- **uorb_uart_bridge**: Inter-board communication
- **rc_update**: Base RC input processing

## Troubleshooting

### No RC Input
1. Check SBUS receiver power and binding
2. Verify RC transmitter is in SBUS mode
3. Check parameter `WL_RC_ENABLE = 1`
4. Monitor `input_rc` topic for valid data

### Erratic Control
1. Check RC signal quality (RSSI)
2. Adjust deadzone parameter
3. Verify channel mapping in transmitter
4. Check for interference sources

### Emergency Stop Issues
1. Verify channel 6 configuration
2. Check switch direction (active low)
3. Test with `listener input_rc` command
4. Validate threshold values

### Performance Issues
1. Monitor slip estimation data
2. Check traction control parameters
3. Verify wheel speed limits
4. Review system load and timing

## Development and Testing

### Build Integration
The module is automatically built when `MODULES_WHEEL_LOADER_RC_INPUT=y` is enabled in the board configuration.

### Unit Tests
Run the included test suite:
```bash
make tests
./build/px4_sitl_default/unit-wheel_loader_rc_input_test
```

### Debug Logging
Enable verbose logging:
```bash
param set WL_RC_ENABLE 1
param set WLC_DIAG_EN 1  # Enable wheel loader controller diagnostics
```

## Advanced Configuration

### Custom Channel Mapping
Modify the channel constants in `wheel_loader_rc_input.hpp` for different transmitter configurations.

### Failsafe Behavior
Customize failsafe response in the `handleFailsafe()` method for specific operational requirements.

### Traction Control Tuning
Adjust slip response curves in the wheel loader controller's `processSlipEstimation()` method.

## Support

For technical support and development questions:
- PX4 Development Team
- Wheel Loader System Maintainers
- Issue tracking: GitHub repository