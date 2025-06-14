# Wheel Loader Controller Design Document

## Overview

This document describes the design and implementation of the PX4-based wheel loader controller module. The wheel loader controller serves as the central coordination point for the entire wheel loader system, managing subsystem commands, state arbitration, and safety oversight.

## Architecture

### System Context

The wheel loader consists of several independent subsystems:
- **Front Wheel Controller**: Controls front axle wheels
- **Rear Wheel Controller**: Controls rear axle wheels
- **Boom Controller**: Controls boom hydraulic actuators
- **Bucket Controller**: Controls bucket hydraulic actuators
- **Steering Controller**: Controls front axle steering and frame articulation

### Controller Role

The wheel loader controller acts as:
1. **Command Arbitrator**: Resolves conflicts between manual, autonomous, and external commands
2. **State Manager**: Maintains overall vehicle state and operational modes
3. **Safety Supervisor**: Enforces safety limits and emergency procedures
4. **Coordination Hub**: Ensures proper sequencing of subsystem operations

## Design Principles

### Adherence to PX4 Standards

The design follows PX4 coding style and architectural patterns:
- Snake_case file naming (`wheel_loader_controller.hpp`, `wheel_loader_controller.cpp`)
- CamelCase class names (`WheelLoaderController`)
- Underscore prefix for private members (`_control_state`, `_safety_manager`)
- Parameter naming with 16-character limit (`WLC_` prefix)
- Tab indentation (4 spaces width)
- Proper inheritance from PX4 base classes

### Modular Design

- **Independent Subsystems**: Each subsystem (boom, bucket, wheels, steering) operates independently
- **Clean Interfaces**: Communication through well-defined uORB messages
- **Separation of Concerns**: Controller logic separated from hardware specifics
- **Configurable Parameters**: All tuning parameters exposed through PX4 parameter system

### Safety-First Approach

- **Multiple Safety Layers**: Emergency stops, limit checking, health monitoring
- **Graceful Degradation**: System continues operating with reduced capability during faults
- **Predictable Behavior**: Clear state machine with deterministic transitions
- **Fail-Safe Defaults**: Safe default values for all parameters and states

## System Architecture

### Message Flow

```
External Commands → WheelLoaderController → Subsystem Controllers → Hardware
                 ↑
Manual Control ─┘

Hardware → Subsystem Status → WheelLoaderController → WheelLoaderStatus → External Systems
```

### Key uORB Messages

#### Input Messages
- `wheel_loader_command` - High-level control commands
- `manual_control_setpoint` - Manual joystick/RC input
- `task_execution_command` - Autonomous task commands
- `wheel_status[2]` - Front/rear wheel controller feedback
- `boom_status` - Boom controller feedback
- `bucket_status` - Bucket controller feedback
- `steering_status` - Steering controller feedback

#### Output Messages
- `wheel_speeds_setpoint[2]` - Commands to front/rear wheel controllers
- `boom_command` - Commands to boom controller
- `bucket_command` - Commands to bucket controller
- `steering_command` - Commands to steering controller
- `wheel_loader_status` - Overall system status

### Class Structure

```cpp
class WheelLoaderController : public ModuleBase<WheelLoaderController>,
                             public ModuleParams,
                             public px4::ScheduledWorkItem
{
public:
    // Module interface
    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    // Main execution
    void Run() override;
    bool init();

private:
    // State management
    enum class ControlState : uint8_t {
        INITIALIZING,
        IDLE,
        MANUAL_CONTROL,
        TASK_EXECUTION,
        EMERGENCY_STOP,
        ERROR
    };

    enum class CommandSource : uint8_t {
        NONE,
        MANUAL,
        TASK_EXECUTION,
        EXTERNAL
    };

    // Core processing functions
    void processWheelLoaderCommand();
    void processTaskExecution();
    void processVehicleCommand();
    void updateControlState();
    void publishCommands();
    void publishStatus();

    // Safety and health
    void performSafetyChecks();
    void updateSubsystemHealth();
    void handleEmergencyStop();

    // Command arbitration
    CommandSource selectActiveCommandSource();
    bool validateCommand(const wheel_loader_command_s &cmd);
    void applyCommandLimits();

    // uORB subscriptions and publications
    // ...
};
```

## State Machine

### Primary States

1. **INITIALIZING**
   - Waiting for all subsystems to become ready
   - Performing initial safety checks
   - Loading parameters

2. **IDLE**
   - No active commands
   - All subsystems in safe idle state
   - Ready to accept commands

3. **MANUAL_CONTROL**
   - Processing manual joystick/RC commands
   - Direct operator control
   - Highest priority for safety

4. **TASK_EXECUTION**
   - Executing autonomous tasks
   - Following predefined work sequences
   - Continuous progress monitoring

5. **EMERGENCY_STOP**
   - All motion stopped immediately
   - Hydraulics disabled
   - Requires manual reset

6. **ERROR**
   - System fault detected
   - Limited functionality
   - Diagnostic mode active

### State Transitions

```
INITIALIZING → IDLE (when all subsystems ready)
IDLE → MANUAL_CONTROL (on manual input)
IDLE → TASK_EXECUTION (on task command)
MANUAL_CONTROL → IDLE (when input stops)
TASK_EXECUTION → IDLE (when task completes)
Any State → EMERGENCY_STOP (on emergency trigger)
EMERGENCY_STOP → IDLE (on manual reset)
Any State → ERROR (on critical fault)
ERROR → IDLE (after fault cleared)
```

## Command Processing

### Command Sources Priority

1. **Emergency Stop** (highest priority)
2. **Manual Control** (operator override)
3. **Task Execution** (autonomous operation)
4. **External Commands** (MAVLink, etc.)

### Command Validation

All commands undergo validation:
- Range checking for all setpoints
- Rate limiting for safety
- Consistency checks between subsystems
- Hardware limit enforcement

### Command Arbitration

When multiple command sources are active:
1. Emergency stops override everything
2. Manual control overrides autonomous
3. Most recent valid command within source priority
4. Smooth transitions between command sources

## Two-Wheel Controller Design

### Architecture Decision

The system uses two independent wheel controllers instead of a single four-wheel controller:

**Benefits:**
- **Modularity**: Each axle can be developed and tested independently
- **Flexibility**: Different control strategies for front/rear axles
- **Fault Tolerance**: System can operate with one axle disabled
- **Scalability**: Easy to add more axles or change wheel configurations
- **Maintenance**: Independent diagnostics and parameter tuning

### Implementation

```cpp
// Two separate publications for front and rear wheel controllers
uORB::PublicationMulti<wheel_speeds_setpoint_s> _front_wheel_setpoint_pub{ORB_ID(wheel_speeds_setpoint)};
uORB::PublicationMulti<wheel_speeds_setpoint_s> _rear_wheel_setpoint_pub{ORB_ID(wheel_speeds_setpoint)};

// Two separate status subscriptions using SubscriptionMultiArray
uORB::SubscriptionMultiArray<wheel_status_s, 2> _wheel_status_subs{ORB_ID(wheel_status)};
```

### Wheel Controller Interface

Each wheel controller receives:
- Speed setpoints (rad/s)
- Acceleration limits
- Torque limits
- Safety flags

Each wheel controller provides:
- Actual speeds
- Motor currents
- Temperature data
- Fault status
- Health metrics

## Safety System

### Multi-Layer Safety

1. **Hardware Safety**
   - Emergency stop buttons
   - Motor current limits
   - Temperature monitoring
   - Limit switches

2. **Software Safety**
   - Speed limits
   - Acceleration limits
   - Operational envelopes
   - Watchdog timers

3. **Operator Safety**
   - Manual override capability
   - Visual and audio warnings
   - Clear system status indication
   - Predictable behavior

### Emergency Procedures

**Emergency Stop Activation:**
1. Immediately set all motor commands to zero
2. Disable hydraulic systems
3. Set emergency_stop flag in all subsystem commands
4. Publish emergency status
5. Log emergency event
6. Wait for manual reset

**Fault Handling:**
1. Isolate faulty subsystem
2. Continue operation with remaining systems
3. Notify operator of degraded capability
4. Log fault for diagnostics
5. Attempt automatic recovery if safe

## Parameter System

### Parameter Naming Convention

All parameters use `WLC_` prefix (Wheel Loader Controller):

```c
// Control parameters
PARAM_DEFINE_FLOAT(WLC_CTRL_RATE, 50.0f);    // Control loop rate (Hz)
PARAM_DEFINE_FLOAT(WLC_CMD_TIMEOUT, 0.5f);   // Command timeout (s)

// Safety parameters
PARAM_DEFINE_FLOAT(WLC_MAX_SPEED, 5.0f);     // Maximum vehicle speed (m/s)
PARAM_DEFINE_FLOAT(WLC_MAX_ACCEL, 2.0f);     // Maximum acceleration (m/s²)
PARAM_DEFINE_INT32(WLC_ESTOP_EN, 1);         // Emergency stop enable

// Health monitoring
PARAM_DEFINE_FLOAT(WLC_HEALTH_TO, 1.0f);     // Health timeout (s)
PARAM_DEFINE_INT32(WLC_DIAG_EN, 0);          // Diagnostic mode enable
```

### Parameter Categories

1. **Control Parameters**: Loop rates, timeouts, gains
2. **Safety Parameters**: Limits, thresholds, enable flags
3. **Health Parameters**: Monitoring intervals, fault thresholds
4. **Diagnostic Parameters**: Debug modes, logging levels

## Performance Requirements

### Real-Time Constraints

- **Control Loop Rate**: 50 Hz (20ms cycle time)
- **Command Latency**: < 10ms from input to output
- **Safety Response**: < 5ms for emergency stop
- **Status Update Rate**: 10 Hz for normal status, 50 Hz during faults

### Resource Usage

- **CPU Usage**: < 5% average, < 20% peak
- **Memory**: < 64KB RAM, < 128KB flash
- **Network**: < 1MB/s total message traffic
- **Storage**: Minimal logging, parameter persistence only

## Testing Strategy

### Unit Testing

- State machine transitions
- Command validation logic
- Safety function behavior
- Parameter handling
- Message parsing/generation

### Integration Testing

- Subsystem communication
- Emergency stop procedures
- Command arbitration
- Health monitoring
- Performance under load

### System Testing

- Full vehicle operation
- Operator interface validation
- Safety system verification
- Environmental testing
- Long-duration reliability

## Implementation Guidelines

### Code Organization

```
wheel_loader/
├── CMakeLists.txt                    # Build configuration
├── Kconfig                          # Module configuration
├── wheel_loader_controller_main.cpp # Entry point
├── wheel_loader_controller.hpp      # Class declaration
├── wheel_loader_controller.cpp      # Class implementation
└── wheel_loader_controller_params.c # Parameter definitions
```

### Key Implementation Points

1. **Follow PX4 Patterns**: Use standard PX4 module structure and conventions
2. **Error Handling**: Comprehensive error checking and recovery
3. **Logging**: Appropriate use of PX4_INFO, PX4_WARN, PX4_ERR
4. **Documentation**: Clear comments explaining complex logic
5. **Performance**: Efficient algorithms, minimal dynamic allocation

### Code Quality

- **Static Analysis**: Clean builds with no warnings
- **Code Reviews**: Peer review of all changes
- **Regression Testing**: Automated test suite
- **Performance Profiling**: Regular performance monitoring
- **Memory Leak Detection**: Valgrind or similar tools

## Future Enhancements

### Planned Features

1. **Load-Aware Control**: Adjust control based on payload weight
2. **Predictive Maintenance**: Monitor component wear and predict failures
3. **Advanced Autonomy**: Support for complex multi-step tasks
4. **Machine Learning**: Adaptive control based on usage patterns
5. **Remote Operation**: Enhanced teleoperation capabilities

### Scalability Considerations

- **Multi-Vehicle Fleets**: Coordination between multiple loaders
- **Different Configurations**: Support for various wheel loader types
- **Hardware Abstraction**: Easy adaptation to different actuators/sensors
- **Protocol Evolution**: Forward compatibility with message updates

## Conclusion

This design provides a robust, safety-focused foundation for wheel loader control while maintaining the modularity and maintainability required for a complex autonomous system. The two-wheel controller architecture offers the right balance of flexibility and simplicity, while the comprehensive safety system ensures reliable operation in demanding environments.

The implementation follows PX4 best practices and coding standards, ensuring consistency with the broader PX4 ecosystem and ease of maintenance by the development team.
