# Steering Controller

## Overview

The Steering Controller module provides precise steering control for articulated wheel loaders using the ST3125 servo controller. It implements position-based control with advanced features for challenging operating conditions.

## Key Features

- **ST3125 Servo Integration**: Direct position commands to ST3125 servo with internal PID control
- **Slip Compensation**: Uses PredictiveTraction data to adjust steering for slippery conditions
- **Rate Limiting**: Smooth steering transitions to prevent jerky movements
- **Feedforward Control**: Improved dynamic response during rapid maneuvers
- **Limit Sensors**: Hardware limit sensor integration for position safety
- **Safety Management**: Comprehensive fault detection and emergency stop functionality
- **Performance Monitoring**: Real-time metrics and diagnostics

## Architecture

### Control Loop
- 50Hz control rate for responsive steering
- Position commands sent to ST3125 servo
- Servo handles internal PID control and position feedback

### Safety Features
- Position limit monitoring with configurable margins
- Servo fault detection (overcurrent, temperature, communication)
- Command timeout handling (returns to center position)
- Emergency stop integration
- Configurable violation limits before emergency stop

### Slip Compensation
- Monitors front and rear wheel slip ratios
- Calculates slip asymmetry for understeer/oversteer detection
- Applies corrective steering adjustments
- Configurable compensation gain and limits

## Parameters

### Basic Control
- `STEER_MAX_ANGLE`: Maximum steering angle (±45° default)
- `STEER_MAX_RATE`: Maximum steering rate (60°/s default)
- `STEER_DEADBAND`: Position deadband to reduce jitter
- `STEER_TRIM`: Calibration offset for servo mounting
- `STEER_REVERSE`: Reverse steering direction if needed

### ST3125 Servo Configuration
- `STEER_ST3125_ID`: CAN ID for the servo (1 default)
- `STEER_ST3125_MAX_ANG`: Servo maximum angle capability
- `STEER_ST3125_MAX_VEL`: Servo maximum velocity
- `STEER_ST3125_CURR_LT`: Current limit protection
- `STEER_ST3125_DEADBAND`: Servo position deadband

### Slip Compensation
- `STEER_SLP_CP_EN`: Enable slip compensation (1 default)
- `STEER_SLP_CP_GN`: Compensation gain (0.5 default)
- `STEER_SLP_CP_MA`: Maximum compensation angle (10° default)

### Feedforward Control
- `STEER_FF_GAIN`: Feedforward gain (0.2 default)
- `STEER_FF_SPD_SC`: Speed-dependent scaling (0.1 default)

### Safety Limits
- `STEER_MAX_POS_ERR`: Maximum position error before fault (10° default)
- `STEER_CMD_TIMEOUT`: Command timeout (500ms default)
- `STEER_FB_TIMEOUT`: Servo feedback timeout (100ms default)
- `STEER_SENS_TIMEOUT`: Limit sensor timeout (200ms default)

### Limit Sensors
- `STEER_LIMIT_EN`: Enable limit sensor monitoring
- `STEER_LT_LF_ID`: Left limit sensor instance ID
- `STEER_LT_RT_ID`: Right limit sensor instance ID
- `STEER_LIMIT_MARG`: Safety margin before limit activation

### Safety Manager
- `STEER_SAFETY_EN`: Enable safety manager
- `STEER_SAFE_POS`: Safe position for violations (0° default)
- `STEER_FT_TIMEOUT`: Fault timeout before clearing (5s default)
- `STEER_MAX_VIOL`: Maximum violations before emergency stop

## uORB Topics

### Subscriptions
- `steering_setpoint`: Steering angle commands
- `vehicle_status`: Vehicle state for emergency stop detection
- `predictive_traction`: Slip data for compensation
- `robotic_servo_feedback`: ST3125 servo feedback
- `limit_sensor`: Position limit sensor data

### Publications
- `steering_status`: Current steering state and diagnostics
- `robotic_servo_command`: Commands to ST3125 servo

## Usage

### Starting the Module
```bash
steering_controller start
```

### Checking Status
```bash
steering_controller status
```

### Stopping the Module
```bash
steering_controller stop
```

## Implementation Details

### Control Pipeline
1. **Input Processing**: Read steering setpoints and apply trim/reverse
2. **Slip Compensation**: Adjust for wheel slip conditions
3. **Feedforward**: Add velocity-dependent feedforward term
4. **Rate Limiting**: Smooth rapid changes
5. **Safety Checks**: Verify position limits and sensor states
6. **Servo Command**: Send position/velocity to ST3125

### Safety State Machine
- **Normal Operation**: All systems healthy, full control authority
- **Safety Violation**: Fault detected, move to safe position
- **Emergency Stop**: Too many violations, require manual reset

### Performance Monitoring
- Position tracking error statistics
- Command and feedback timeout counters
- Sensor health monitoring
- Slip compensation activity tracking

## Troubleshooting

### Common Issues

1. **Servo Not Responding**
   - Check CAN connection and servo ID parameter
   - Verify servo power and configuration
   - Check feedback timeout counter

2. **Erratic Steering**
   - Adjust deadband parameter
   - Check for mechanical backlash
   - Verify limit sensor health

3. **Safety Violations**
   - Check limit sensor configuration
   - Verify position error limits
   - Reset violation counter if needed

4. **Poor Slip Compensation**
   - Verify PredictiveTraction module is running
   - Adjust compensation gain
   - Check slip data quality

### Diagnostic Commands
```bash
# View detailed status
steering_controller status

# Reset safety violations
steering_controller reset_safety
```

## Integration Notes

- Requires ST3125 servo driver for CAN communication
- Works with PredictiveTraction module for slip compensation
- Integrates with limit sensor module for position safety
- Compatible with standard PX4 vehicle status topics
