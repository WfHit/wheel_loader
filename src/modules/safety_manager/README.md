# System-Level Safety Manager

## Overview

The Safety Manager has been moved from the articulated chassis subsystem to become a system-level module that oversees all vehicle operations. This architectural change provides comprehensive safety oversight across all vehicle systems.

## Architecture

```
┌─────────────────────────────────────────────────────────┐
│                  System Safety Manager                  │
│                  (/src/modules/safety_manager)          │
├─────────────────────────────────────────────────────────┤
│  • Overall system safety coordination                   │
│  • Multi-system fault detection                        │
│  • Emergency response coordination                     │
│  • Safety permit management                            │
│  • System-wide risk assessment                         │
└─────────┬─────────┬─────────┬─────────┬─────────────────┘
          │         │         │         │
┌─────────▼───┐ ┌───▼─────┐ ┌─▼─────┐ ┌─▼─────────┐
│ Chassis     │ │Hydraulic│ │Sensors │ │ Operator  │
│ Systems     │ │ Systems │ │        │ │ Interface │
└─────────────┘ └─────────┘ └───────┘ └───────────┘
```

## Key Features

### System-Wide Monitoring
- **Chassis Systems**: Speed, steering, traction, stability
- **Hydraulic Systems**: Boom, bucket, pressure, temperature
- **Sensor Systems**: IMU, encoders, communication health
- **Power Systems**: Battery, electrical load, charging
- **Thermal Systems**: Engine, hydraulic, ambient temperature
- **Articulation**: Joint angles, rates, limits

### Multi-Layered Safety Levels
1. **NORMAL**: All systems operating within limits
2. **CAUTION**: Minor deviations, enhanced monitoring
3. **WARNING**: Approaching limits, prepare interventions
4. **CRITICAL**: Limits exceeded, active intervention
5. **EMERGENCY**: Immediate danger, emergency response

### Comprehensive Fault Detection
- Speed limit violations
- Steering system faults
- Communication timeouts
- Hardware failures
- Sensor anomalies
- Stability risks
- Load distribution issues
- Terrain hazards
- Hydraulic malfunctions
- Power system faults
- Thermal overloads

### Safety Permits System
The Safety Manager controls operational permits for:
- Vehicle motion
- Steering operations
- Autonomous mode
- Hydraulic operations
- Boom operations
- Bucket operations
- Articulation movement
- Engine starting
- Emergency overrides

### Emergency Response Capabilities
- **Emergency Stop**: Immediate halt of all operations
- **Controlled Stop**: Gradual, safe deceleration
- **Emergency Shutdown**: Safe system shutdown sequence
- **Backup Systems**: Activation of redundant controls
- **Operator Alerts**: Visual and audible warnings

## Integration Points

### Subscribed Topics
- `vehicle_status` - Overall vehicle state
- `wheel_loader_status` - Wheel loader specific status
- `boom_status` - Boom hydraulic system
- `bucket_status` - Bucket hydraulic system
- `load_aware_torque` - Chassis load distribution
- `vehicle_attitude` - Vehicle orientation
- `sensor_accel` - Acceleration data
- `sensor_gyro` - Angular rate data
- All module status topics

### Published Topics
- `system_safety` - Overall system safety status
- `vehicle_command` - Safety override commands
- `module_status` - Safety Manager health

## Configuration Parameters

### Speed Safety
- `SM_MAX_SPEED`: Maximum vehicle speed (m/s)
- `SM_MAX_ACCEL`: Maximum acceleration (m/s²)

### Steering Safety
- `SM_MAX_STEER_ANGLE`: Maximum steering angle (rad)
- `SM_MAX_STEER_RATE`: Maximum steering rate (rad/s)

### Stability Safety
- `SM_MAX_ROLL_ANGLE`: Maximum roll angle (rad)
- `SM_MAX_PITCH_ANGLE`: Maximum pitch angle (rad)
- `SM_STABILITY_MARGIN`: Required stability margin

### Hydraulic Safety
- `SM_MAX_HYDRAULIC_PRESSURE`: Maximum hydraulic pressure (bar)
- `SM_MAX_HYDRAULIC_TEMP`: Maximum hydraulic temperature (°C)

### Load Safety
- `SM_MAX_PAYLOAD`: Maximum payload (kg)
- `SM_MAX_COG_OFFSET`: Maximum center of gravity offset (m)

## Usage

### Starting the Safety Manager
```bash
safety_manager start
```

### Checking Status
```bash
safety_manager status
```

### Emergency Stop
```bash
safety_manager emergency_stop
```

### Reset After Emergency
```bash
safety_manager reset_emergency
```

## Benefits of System-Level Architecture

1. **Unified Safety Oversight**: Single point of safety coordination
2. **Cross-System Awareness**: Detects issues spanning multiple subsystems
3. **Reduced Complexity**: Eliminates redundant safety logic in subsystems
4. **Improved Response Time**: Centralized decision making
5. **Better Diagnostics**: Comprehensive system health monitoring
6. **Enhanced Reliability**: Independent safety oversight
7. **Easier Maintenance**: Single safety configuration point

## Safety Standards Compliance

The system-level Safety Manager is designed to meet:
- ISO 25119 (Functional Safety for Agricultural Machinery)
- EN 474 (Earth-moving machinery safety requirements)
- IEC 61508 (Functional Safety of Electrical Systems)

## Development Guidelines

### Adding New Safety Checks
1. Define monitoring structure in `SafetyManager.hpp`
2. Implement monitoring function
3. Add fault type to `FaultType` enum
4. Update risk assessment logic
5. Define appropriate safety response

### Safety-Critical Considerations
- All safety decisions must be deterministic
- Response times must be bounded and measured
- Fail-safe defaults for all operations
- Independent watchdog for safety manager itself
- Comprehensive logging of all safety events

## Testing

### Unit Tests
- Individual monitoring function validation
- Fault detection accuracy
- Response time verification

### Integration Tests
- Multi-system fault scenarios
- Emergency response coordination
- Safety permit validation

### Hardware-in-Loop Tests
- Real vehicle safety scenarios
- Sensor failure simulation
- Communication fault handling

## Future Enhancements

- Predictive safety analytics
- Machine learning fault detection
- Remote safety monitoring
- Adaptive safety thresholds
- Integration with fleet management
