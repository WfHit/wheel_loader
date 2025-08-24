# VLA Dual-Mode Wheel Loader Robot

## Overview

This implementation extends the existing wheel loader robot to support dual operation modes:

1. **MANUAL MODE**: Traditional RC/joystick control
2. **AUTO MODE**: Autonomous operation driven by VLA algorithm

## Architecture

### New Components

#### Message Definitions
- `VlaCommand.msg`: Interface for VLA bucket position and pose outputs
- `OperationModeCommand.msg`: Mode switching control between manual and auto

#### Extended State Machine
- `AUTO_OPERATION`: VLA autonomous control state
- `MODE_TRANSITION`: Safe mode switching state

#### Command Sources
- `VLA`: New command source for autonomous operations

### Integration Points

#### With Existing Controller
- Extends current state machine (INITIALIZING, IDLE, MANUAL_CONTROL, TASK_EXECUTION, EMERGENCY_STOP, ERROR)
- Uses existing command arbitration with enhanced priority system
- Maintains all existing safety systems

#### Command Priority (Highest to Lowest)
1. Emergency Stop
2. Manual Control (always overrides autonomous)
3. VLA (when in AUTO mode)
4. Task Execution
5. External Commands

## Safety Features

### Manual Override
- Manual control always takes immediate priority over autonomous
- Emergency mode switch forces immediate manual mode
- Manual override can be activated at any time

### Mode Transition Safety
- Validation of mode transitions
- Timeout protection during transitions
- System health checks before enabling auto mode
- AI communication monitoring

### Autonomous Operation Safety
- AI confidence score validation (>0.5 required)
- Communication timeout detection (automatic fallback to manual)
- Emergency stop capability from AI
- Conservative speed and acceleration limits in auto mode

## Auto Load/Dump Sequences

### Auto Load Sequence
1. **Approach**: Lower bucket and boom for digging position
2. **Engage**: Begin scooping motion with bucket curl and forward movement
3. **Complete**: Lift loaded bucket to transport position

### Auto Dump Sequence
1. **Approach**: Maintain loaded position while navigating to dump site
2. **Position**: Raise boom to dump height
3. **Execute**: Tip bucket to release material
4. **Complete**: Ensure complete dump and return to neutral

## Parameters

### New Parameters
- `WLR_MODE_TO`: Mode transition timeout (default: 2.0s)
- `WLR_AUTO_EN`: Enable autonomous mode (default: enabled)
- `WLR_VLA_TIMEOUT`: VLA communication timeout (default: 1.0s)

### Enhanced Parameters
All existing wheel loader robot parameters remain active and apply safety limits in both modes.

## Usage

### Mode Switching
Send `OperationModeCommand` message with:
- `mode_switch_request = true`
- `operation_mode = MODE_AUTO` or `MODE_MANUAL`

### AI Integration
Publish `AiOutput` messages containing:
- Bucket position and pose commands
- Confidence score and validation flags

### Manual Override
- Any manual control input immediately activates manual mode
- RC transmitter mode switch provides dedicated mode control
- Emergency mode switch forces manual mode regardless of state

## Testing

### Basic Functionality
1. Verify mode switching between manual and auto
2. Test manual override during autonomous operation
3. Validate timeout behavior when AI communication is lost
4. Check emergency stop functionality from both RC and AI

### Safety Validation
1. Confirm manual control always overrides autonomous
2. Test mode transition timeout and recovery
3. Verify emergency stop behavior in all modes
4. Validate AI confidence score filtering

### Auto Sequences
1. Test auto load sequence with varying approach angles
2. Validate dump sequence completion detection
3. Check sequence step progression and timeout handling

## Implementation Notes

### Minimal Changes Approach
- Extends existing controller rather than replacing it
- Maintains backward compatibility with current manual control
- Uses existing uORB message infrastructure where possible
- Follows PX4 coding standards and patterns

### Integration with AI
- Designed for loose coupling with AI algorithm
- Confidence-based command filtering
- Graceful degradation when AI is unavailable
- Clear separation between perception (AI) and control (wheel loader)

This implementation provides a solid foundation for dual-mode operation while maintaining the safety and reliability of the existing wheel loader robot.