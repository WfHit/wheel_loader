# H-Bridge Module Redesign Summary

## Overview
The H-bridge module has been completely redesigned to follow the PX4 coding standards and implement a multi-instance architecture similar to the limit sensor module. The new design supports 2 independent H-bridge channels with shared enable control and integrated limit sensor safety.

## Key Design Changes

### 1. Multi-Instance Architecture
- **2 Instances**: Each instance controls exactly one H-bridge channel
- **Instance 0**: Left motor/channel (manager instance)
- **Instance 1**: Right motor/channel
- **Shared Enable**: Instance 0 manages the shared enable pin for both channels

### 2. Instance-Specific Command Subscription
- Each instance subscribes to a different command message instance
- Parameter `HBRIDGE_MSG_INST0` controls which command instance channel 0 subscribes to
- Parameter `HBRIDGE_MSG_INST1` controls which command instance channel 1 subscribes to
- This allows independent control of each channel

### 3. Multi-Instance Status Publishing
- Each instance publishes its own status message
- Status includes current speed, limit sensor states, and enable status
- Forward and reverse limit states are included in each status message

### 4. Limit Sensor Integration
- Forward limit stops forward motion (positive duty cycle)
- Reverse limit stops reverse motion (negative duty cycle)
- Safety limits are applied before PWM output
- Integrated with existing limit sensor system

### 5. PWM Output Function
- Clean `output_pwm(float duty_cycle)` function
- Duty cycle range: -1.0 to 1.0 (negative = reverse)
- Automatic direction control via GPIO
- Configurable PWM min/max values

## File Structure

### Modified Files:
1. **`hbridge.hpp`**: Complete redesign with multi-instance support
2. **`hbridge.cpp`**: Complete reimplementation following limit sensor pattern
3. **`HbridgeCommand.msg`**: Updated to support instance-specific commands
4. **`HbridgeStatus.msg`**: Enhanced with limit sensor status
5. **`module.yaml`**: New parameter definitions

## New Architecture Details

### Class Structure
```cpp
class HBridge : public ModuleBase<HBridge>,
                public ModuleParams,
                public px4::ScheduledWorkItem
```

### Static Members (Shared Between Instances)
- `_instances[MAX_INSTANCES]`: Array of active instances
- `_num_instances`: Count of active instances
- `_enable_state`: Shared enable state (atomic)
- `_pwm_initialized`: PWM system initialization flag

### Instance-Specific Members
- `_instance`: Instance ID (0 or 1)
- `_board_config`: Board-specific configuration
- `_current_duty_cycle`: Current motor speed
- `_forward_limit_active`/`_reverse_limit_active`: Limit sensor states
- Multi-instance publications and subscriptions

### Key Methods
- `output_pwm(float duty_cycle)`: PWM output with safety limits
- `apply_safety_limits(float duty_cycle)`: Limit sensor integration
- `control_enable(bool enable)`: Shared enable control (manager only)
- `process_commands()`: Instance-specific command processing
- `process_limit_sensors()`: Limit sensor monitoring

## Usage Examples

### Starting Instances
```bash
# Start both channels
hbridge start -i 0  # Left motor (manager)
hbridge start -i 1  # Right motor

# Or start all at once
hbridge start
```

### Parameter Configuration
```bash
# Configure message instances
param set HBRIDGE_MSG_INST0 0  # Channel 0 uses command instance 0
param set HBRIDGE_MSG_INST1 1  # Channel 1 uses command instance 1

# Configure PWM limits
param set HBRIDGE_PWM_MIN 0.1   # 10% minimum speed
param set HBRIDGE_PWM_MAX 0.9   # 90% maximum speed

# Set command timeout
param set HBRIDGE_TIMEOUT 2.0   # 2 second timeout
```

### Command Messages
```cpp
// Send command to instance 0 (left motor)
hbridge_command_s cmd;
cmd.timestamp = hrt_absolute_time();
cmd.instance = 0;           // Target instance 0
cmd.duty_cycle = 0.5f;      // 50% forward speed
cmd.enable = true;

// Send command to instance 1 (right motor)
hbridge_command_s cmd2;
cmd2.timestamp = hrt_absolute_time();
cmd2.instance = 1;          // Target instance 1
cmd2.duty_cycle = -0.3f;    // 30% reverse speed
cmd2.enable = true;
```

## Safety Features

### Limit Sensor Integration
- Forward limit sensors prevent positive duty cycle output
- Reverse limit sensors prevent negative duty cycle output
- Automatic motor stop when limits are triggered
- Real-time limit status in status messages

### Command Timeout
- Configurable timeout stops motor if no commands received
- Safety feature prevents runaway motors
- Can be disabled by setting timeout to 0

### Shared Enable Control
- Only manager instance (0) controls shared enable pin
- Enable state is shared atomically between instances
- Proper cleanup when instances stop

## Message Definitions

### HbridgeCommand.msg
```
uint64 timestamp          # time since system start (microseconds)
uint8 instance            # Target H-bridge instance (0 or 1)
float32 duty_cycle        # PWM duty cycle (-1.0 to 1.0, negative for reverse)
bool enable               # Enable/disable the H-bridge channel
```

### HbridgeStatus.msg
```
uint64 timestamp          # time since system start (microseconds)
uint8 instance            # H-bridge instance (0 or 1)
float32 duty_cycle        # Current duty cycle (-1.0 to 1.0)
bool forward_limit        # Forward limit sensor active
bool reverse_limit        # Reverse limit sensor active
bool enabled              # H-bridge enable state
# ... (additional compatibility fields)
```

## Benefits of New Design

1. **Independent Control**: Each channel can be controlled separately
2. **Safety Integration**: Built-in limit sensor safety
3. **PX4 Standards**: Follows established coding patterns
4. **Scalability**: Easy to extend for more channels
5. **Robustness**: Proper error handling and cleanup
6. **Flexibility**: Configurable parameters for different setups

## Migration Notes

### For Users:
- Update parameter names (old parameters are obsolete)
- Use instance-specific command topics
- Subscribe to instance-specific status topics

### For Developers:
- New API with `output_pwm()` function
- Instance-based architecture
- Integrated safety systems
- Multi-instance message handling

This redesigned H-bridge module provides a robust, safe, and flexible motor control system that integrates seamlessly with the wheel loader's limit sensor system while following PX4 best practices.
