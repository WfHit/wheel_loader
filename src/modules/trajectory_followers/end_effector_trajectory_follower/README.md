# End Effector Trajectory Follower

## Overview

The End Effector Trajectory Follower is a key component in the wheel loader's autonomous control system. It serves as the bridge between high-level trajectory planning and low-level joint control, transforming end effector trajectory setpoints into coordinated boom and bucket control commands.

**Key Concept**: The "end effector" is the final working tool (bucket, fork, grapple, etc.) viewed from the system perspective, while "bucket" refers to the same physical object from the local joint control perspective.

## Architecture

### Control Hierarchy

```
┌─────────────────────────────────────┐
│        High-Level Planner          │ (VLA, Operation Mode, etc.)
│   (Position/Orientation Goals)     │
└─────────────────┬───────────────────┘
                  │ end_effector_trajectory_setpoint
                  ▼
┌─────────────────────────────────────┐
│   End Effector Trajectory Follower │ ◄── This Module
│      (Inverse Kinematics)          │
└─────────────┬───────────┬───────────┘
              │           │
              ▼           ▼
  boom_trajectory_setpoint  bucket_trajectory_setpoint
              │           │
              ▼           ▼
┌─────────────────┐ ┌─────────────────┐
│ Boom Controller │ │Bucket Controller│ (Joint-level control)
└─────────────────┘ └─────────────────┘
```

### Key Responsibilities

1. **Coordinate Transformation**: Convert end effector poses from chassis/world coordinates to joint space
2. **Inverse Kinematics**: Solve the kinematic chain to determine required boom and bucket angles
3. **Trajectory Generation**: Generate smooth, coordinated trajectories for both joints
4. **Safety Validation**: Ensure joint limits and workspace constraints are respected

**Control Philosophy**: This module thinks globally about end effector positioning while generating local joint commands for boom and bucket controllers.

## Coordinate Systems

### Reference Frames

The system operates with a clear hierarchy of coordinate frames:

```
World Frame
    ↓ (vehicle position/orientation)
Chassis Frame
    ↓ (boom joint)
Boom Frame
    ↓ (bucket joint)
Bucket Frame
```

### Frame Definitions

- **Chassis Frame**: Fixed to the vehicle chassis, origin at boom pivot point
- **Boom Frame**: Rotates with boom relative to chassis
- **Bucket Frame**: Rotates with bucket relative to boom
- **End Effector**: The final working tool (bucket/fork/grapple/etc.) viewed from the complete system perspective

### Key Distinction: Bucket vs End Effector

**Same Physical Object, Different Perspectives:**

- **Bucket**: Local joint control perspective - focuses on boom→bucket joint angle
- **End Effector**: System-wide perspective - focuses on the bucket as the final working element in the complete kinematic chain

**Why This Distinction Matters:**

1. **Control Perspective**:
   - **Bucket Controller**: Thinks locally → "Move bucket 30° relative to boom"
   - **End Effector Trajectory Follower**: Thinks globally → "Position end effector (bucket) at coordinates (x,y,z) in chassis frame"

2. **Terminology Clarity**:
   - **"Bucket"**: When discussing the physical attachment and local joint control
   - **"End Effector"**: When discussing the bucket's role as the final working element

3. **System Architecture**:
   - **Local Control**: Joint-level commands (boom angle, bucket angle)
   - **Global Control**: End effector positioning in world/chassis coordinates

**Multi-Tool Support:**
- **Bucket attachment** → End effector is the bucket
- **Fork attachment** → End effector is the fork
- **Grapple attachment** → End effector is the grapple
- **Blade attachment** → End effector is the blade

The end effector is always the final working tool, viewed from the system's perspective.

## Kinematic Model

### 2D Simplified Model

The current implementation uses a simplified 2D kinematic model in the vertical plane:

```
Parameters:
- BOOM_LENGTH = 3.0m
- BUCKET_LENGTH = 1.5m
- BOOM_PIVOT_HEIGHT = 1.0m (above chassis)

Constraints:
- Boom Angle: -20° to +60° (relative to chassis horizontal)
- Bucket Angle: -90° to +90° (relative to boom)
```

### Inverse Kinematics Algorithm

```cpp
Input: end_effector_position_chassis(x, y, z)
Output: boom_angle_chassis, bucket_angle_boom

1. Project to 2D vertical plane (x, z)
2. Check workspace reachability
3. Apply law of cosines to solve triangle
4. Compute boom angle relative to chassis
5. Compute bucket angle relative to boom
6. Validate joint limits

Control Philosophy:
- Input: Global end effector positioning goal
- Output: Local joint angles for boom and bucket controllers
- The system thinks globally but acts locally
```

### Forward Kinematics (Validation)

```cpp
Input: boom_angle_chassis, bucket_angle_boom
Output: end_effector_position_chassis

1. Compute boom tip position in chassis frame
2. Compute bucket angle in chassis frame
3. Compute final end effector (working tool) position and orientation
4. Return end effector state in chassis coordinates

Control Philosophy:
- Input: Local joint angles from controllers
- Output: Global end effector position for validation
- Verifies that local control achieves global positioning goals
```

## Control Flow

### Input Processing

1. **Subscription**: `end_effector_trajectory_setpoint`
   - Position: `(x, y, z)` in chassis coordinates
   - Orientation: `(qw, qx, qy, qz)` quaternion
   - Timestamp and validity flags

2. **State Updates**: Current boom and bucket status for feedback

### Processing Pipeline

1. **Coordinate Transformation**: World → Chassis (if needed)
2. **Inverse Kinematics**: End effector pose → Joint angles
3. **Safety Validation**: Apply joint limits and workspace constraints
4. **Velocity Computation**: Calculate joint velocities for smooth motion
5. **Trajectory Generation**: Create setpoints for controllers

### Output Generation

1. **Boom Trajectory Setpoint**:
   ```cpp
   boom_trajectory_setpoint_s {
       angle_setpoint: boom_angle_chassis
       angular_velocity_setpoint: boom_velocity
       coordinate_frame: 0  // Chassis frame
       control_mode: POSITION
   }
   ```

2. **Bucket Trajectory Setpoint**:
   ```cpp
   bucket_trajectory_setpoint_s {
       angle_setpoint: bucket_angle_boom
       angular_velocity_setpoint: bucket_velocity
       coordinate_frame: 1  // Boom frame
       control_mode: POSITION
   }
   ```

## Parameters

### Control Parameters
- `EE_FOLLOW_RATE`: Trajectory following update rate (Hz)
- `EE_POSITION_TOL`: Position tolerance for trajectory completion (m)
- `EE_MAX_BOOM_VEL`: Maximum boom angular velocity (rad/s)
- `EE_MAX_BUCKET_VEL`: Maximum bucket angular velocity (rad/s)
- `EE_SMOOTHING_FACTOR`: Trajectory smoothing factor
- `EE_IK_TOL`: Inverse kinematics solver tolerance

### Kinematic Parameters
- `BOOM_LENGTH`: Length of boom segment (m)
- `BUCKET_LENGTH`: Length of bucket segment (m)
- `BOOM_PIVOT_HEIGHT`: Height of boom pivot above chassis (m)

## Safety Features

### Workspace Limits
- **Reachability Check**: Ensures target is within kinematic workspace
- **Joint Limits**: Enforces mechanical joint angle constraints
- **Velocity Limits**: Prevents excessive joint speeds
- **Singularity Avoidance**: Detects and handles kinematic singularities

### Error Handling
- **Invalid Solutions**: Graceful handling of unreachable targets
- **Timeout Protection**: Stops trajectory following if no updates received
- **Status Reporting**: Comprehensive status and error reporting

## Performance Monitoring

### Performance Counters
- `_loop_perf`: Overall module execution time
- `_kinematics_perf`: Inverse kinematics computation time

### Status Information
- Trajectory active state
- Last end effector position
- Current joint angles
- Kinematic solution validity

## Usage

### Starting the Module
```bash
end_effector_trajectory_follower start
```

### Status Monitoring
```bash
end_effector_trajectory_follower status
```

### Parameter Configuration
Parameters can be set via QGroundControl or parameter files following the `EE_*` naming convention.

## Integration

### Dependencies
- `lib_mathlib`: Mathematical functions
- `lib_matrix`: Matrix operations
- `lib_perf`: Performance monitoring
- `px4_work_queue`: Scheduling framework

### uORB Topics

#### Subscriptions
- `end_effector_trajectory_setpoint`: High-level trajectory commands
- `boom_status`: Current boom joint state
- `bucket_status`: Current bucket joint state
- `vehicle_local_position`: Vehicle position for coordinate transforms

#### Publications
- `boom_trajectory_setpoint`: Commands for boom controller
- `bucket_trajectory_setpoint`: Commands for bucket controller

## Future Enhancements

### 3D Kinematics
- Full 3D kinematic model with side-to-side motion
- Support for articulated boom mechanisms
- Multi-DOF end effector orientation control

### Advanced Algorithms
- Redundancy resolution for over-constrained systems
- Optimal trajectory planning with obstacle avoidance
- Dynamic load compensation

### Tool Support
- Multi-tool attachment support (forks, grapples, blades)
- Tool-specific kinematic parameters
- Automatic tool detection and configuration
- **Perspective Consistency**: Each attached tool becomes the "end effector" from system perspective:
  - Bucket attachment → End effector is the bucket
  - Fork attachment → End effector is the fork
  - Grapple attachment → End effector is the grapple
  - Blade attachment → End effector is the blade

## Debugging

### Common Issues
1. **Unreachable Targets**: Check workspace limits and kinematic parameters
2. **Jerky Motion**: Adjust velocity limits and smoothing parameters
3. **Poor Tracking**: Verify coordinate frame definitions and transforms
4. **Perspective Confusion**: Remember end effector = working tool from system view, bucket = same tool from joint control view

### Debug Tools
- Enable `PX4_DEBUG` messages for detailed kinematic solver output
- Monitor performance counters for computational bottlenecks
- Use status command for real-time state inspection
- Verify control perspective consistency (global goals → local commands)

## References

- [Wheel Loader Robot Design](../../../design/wheel_loader_robot_design.md)
- [Boom Kinematics Design](../../../docs/BOOM_KINEMATICS_DESIGN.md)
- [Bucket Control Design](../../../docs/BUCKET_CONTROL_SUBSCRIPTION_PATTERN.md)
