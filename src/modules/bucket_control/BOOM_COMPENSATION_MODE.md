# Bucket Control Simplification - Boom Compensation Mode

## Overview
The bucket control module has been simplified to use only one control mode: **Boom Compensation Mode**. In this mode, the bucket maintains its absolute angle regardless of boom movement.

## Key Changes

### 1. **Removed Control Mode Enum**
- Completely removed the `ControlMode` enum and all related variables
- No more mode switching or complex control logic
- Control is now hardcoded to always maintain bucket absolute angle

### 2. **Boom Angle Monitoring**
- Added dedicated `updateBoomAngleMonitoring()` function
- Continuously monitors AS5600 magnetic encoder for boom angle changes
- Detects boom movement with configurable threshold (0.005 rad default)
- Tracks both current and previous boom angles

### 3. **Automatic Boom Compensation**
- Added `compensateForBoomMovement()` function
- Automatically adjusts bucket actuator when boom moves
- Maintains target absolute bucket angle regardless of boom position
- Updates actuator target length in real-time

### 4. **Simplified Command Interface**
- Bucket commands now specify the desired absolute bucket angle
- `cmd.target_angle` = desired ground-relative bucket angle
- `cmd.control_mode` is fixed to 0 (boom compensation)
- No need to specify or change control modes

## Technical Implementation

### Boom Movement Detection
```cpp
// Check if boom angle has changed significantly
float boom_angle_delta = fabsf(new_boom_angle - _previous_boom_angle);
_boom_angle_changed = (boom_angle_delta > _boom_angle_threshold);
```

### Automatic Compensation
```cpp
// When boom moves, maintain same absolute bucket angle
float target_bucket_relative_angle = _target_absolute_bucket_angle - _current_boom_angle;
float new_target_actuator_length = bucketAngleToActuatorLength(target_bucket_relative_angle, _current_boom_angle);
```

### Control Flow
1. Monitor boom angle from AS5600 sensor
2. Detect boom movement (threshold-based)
3. When boom moves, recalculate required actuator length
4. Update actuator target to maintain absolute bucket angle
5. PID controller drives actuator to new target

## Benefits

1. **No Control Modes**: Removed the entire control mode system
2. **Automatic Compensation**: No manual intervention needed when boom moves
3. **Consistent Performance**: Bucket maintains precise angle regardless of boom position
4. **Real-time Response**: Immediate compensation for boom movement
5. **Reduced Complexity**: Eliminated complex multi-mode control logic

## Configuration Parameters

- `_boom_angle_threshold`: Sensitivity for detecting boom movement (default: 0.005 rad)
- All existing kinematic and PID parameters remain unchanged
- Removed parameters related to complex control modes

## Status Reporting

The status message now reports:
- `target_ground_angle`: Current target absolute bucket angle
- `machine_pitch`: Current boom angle (repurposed field)
- `anti_spill_active`: Boom movement detection flag (repurposed field)
- `control_mode`: Always reports 0 (boom compensation mode)

## Usage Example

```cpp
// Command bucket to 15 degrees absolute angle
bucket_command_s cmd{};
cmd.target_angle = math::radians(15.0f);  // 15 degrees absolute
// Bucket will automatically maintain this angle even if boom moves
```

This simplified approach provides robust boom compensation while reducing system complexity and improving operator experience.
