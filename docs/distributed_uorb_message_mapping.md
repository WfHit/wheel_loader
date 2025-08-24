# Distributed uORB Message Mapping

## Overview

This document outlines the comprehensive message flow between the X7+ main board and NXT front/rear boards for the wheel loader system, including sensor data, control commands, and status feedback.

## Message Flow

### X7+ Main Board → NXT Boards (via uorb_uart_bridge)

| Message | Target | Purpose | Status |
|---------|--------|---------|---------|
| `wheel_loader_setpoint` | Both NXT boards | High-level wheel loader commands (speed setpoints, steering) | ✅ Implemented |
| `actuator_outputs` (front) | NXT Front | Front axle + bucket actuator commands | ✅ Implemented |
| `actuator_outputs` (rear) | NXT Rear | Rear axle + boom actuator commands | ✅ Implemented |
| `vehicle_status` | Both NXT boards | System status information | ✅ Implemented |
| `traction_control` | Both NXT boards | Traction control commands from centralized algorithm | ✅ Implemented |
| `boom_command` | NXT Rear | Boom electric motor control commands (velocity) | ✅ Implemented |
| `bucket_command` | NXT Front | Bucket electric motor control commands (angle) | ✅ Implemented |
| `steering_command` | NXT Rear | Steering control commands (ST2135 servo) | ✅ Implemented |
| `heartbeat` | Both NXT boards | Keep-alive and connectivity monitoring | ✅ Implemented |

### NXT Boards → X7+ Main Board (via uorb_uart_proxy)

| Message | Source | Purpose | Status |
|---------|--------|---------|---------|
| `slip_estimation` (front) | NXT Front | Front axle slip calculation results | ✅ Implemented |
| `slip_estimation` (rear) | NXT Rear | Rear axle slip calculation results | ✅ Implemented |
| `boom_status` | NXT Rear | Boom electric motor system status and feedback | ✅ Implemented |
| `bucket_status` | NXT Front | Bucket electric motor system status and feedback | ✅ Implemented |
| `steering_status` | NXT Rear | Steering system status and feedback | ✅ Implemented |
| `limit_sensor_bucket` | NXT Front | Bucket limit switch sensor data for zero calibration | ✅ Implemented |
| `heartbeat` | Both NXT boards | Keep-alive from NXT boards | ✅ Implemented |

## Hardware Configuration

### Front NXT Board
- **H-Bridge Device 1** (2 channels):
  - Channel 1: Front axle drive motor
  - Channel 2: Bucket electric motor control (angle-based)
- **Sensors**:
  - Quadrature encoder #1 (bucket drive motor position and speed) - **Local use only**
  - Quadrature encoder #2 (front axle drive motor speed) - **Local use only**
  - AS5600 magnetic angle sensor (boom angle monitoring for bucket compensation) - **Local use only**
  - Limit sensors (bucket position limits for zero calibration)
- **Control Systems**:
  - Front axle motor controller with PID and encoder feedback
  - Bucket angle controller with encoder position tracking and boom compensation
  - Slip estimation using encoder vs. local EKF speed comparison

### Rear NXT Board  
- **H-Bridge Device 2** (2 channels):
  - Channel 1: Rear axle drive motor
  - Channel 2: Boom electric motor control (velocity-based)
- **Servo Control**:
  - ST2135 servo for vehicle steering/articulation
- **Sensors**:
  - Quadrature encoder (rear axle drive motor speed) - **Local use only**
  - AS5600 angle sensor (boom position) - **Local use only**
- **Control Systems**:
  - Rear axle motor controller with PID and encoder feedback
  - Boom velocity controller with AS5600 feedback and motion planning
  - Steering servo controller for vehicle articulation
  - Slip estimation using encoder vs. local EKF speed comparison

## Control System Architecture

### Slip Estimation and Traction Control
- **Local Calculation**: Each NXT board calculates slip using local encoder data vs. local EKF speed
- **Central Distribution**: X7+ receives slip data from both boards and runs traction distribution function
- **Feedback Loop**: Traction control commands sent back to NXT boards for motor power adjustment

### Electric Motor Control Systems
- **Boom Control (NXT Rear)**:
  - Uses AS5600 sensor for absolute angle measurement
  - Motion planner generates trajectory commands
  - PID controller ensures adherence to motion plan
  - Velocity-based control (rate setpoints)

- **Bucket Control (NXT Front)**:
  - Uses motor encoder for relative position tracking
  - AS5600 sensor monitors boom angle for compensation
  - Requires zero action on init (limit switch calibration)
  - Angle-based control for precise positioning
  - Complex control modes (manual, auto-level, grading, transport)

### Motor Control Systems
- **Front and Rear Axles**:
  - Individual motor controllers for each axle
  - Quadrature encoders attached to motors for speed feedback (local use only)
  - PID speed control with encoder input
  - Real speed measurement from local EKF for slip calculation
  - Comparison between encoder speed and local EKF speed for slip detection

## Message Processing Verification

### Bridge (X7+ → NXT)
- ✅ `WHEEL_LOADER_SETPOINT` - Processed and forwarded to both boards (contains speed and steering commands)
- ✅ `ACTUATOR_OUTPUTS_FRONT` - Filtered to front board only
- ✅ `ACTUATOR_OUTPUTS_REAR` - Filtered to rear board only  
- ✅ `VEHICLE_STATUS` - Broadcast to both boards
- ✅ `TRACTION_CONTROL` - Broadcast to both boards for local motor control
- ✅ `BOOM_COMMAND` - Sent to rear board only (boom electric motor control)
- ✅ `BUCKET_COMMAND` - Sent to front board only (bucket electric motor control)
- ✅ `STEERING_COMMAND` - Sent to rear board only (steering servo)
- ✅ `HEARTBEAT` - Periodic keep-alive

### Proxy (NXT → X7+)
- ✅ `SLIP_ESTIMATION_FRONT` - Slip calculation results from front
- ✅ `SLIP_ESTIMATION_REAR` - Slip calculation results from rear
- ✅ `BOOM_STATUS` - Boom electric motor system status from rear board
- ✅ `BUCKET_STATUS` - Bucket electric motor system status from front board
- ✅ `STEERING_STATUS` - Steering system status from rear board
- ✅ `LIMIT_SENSOR_BUCKET` - Bucket limit switches from front board
- ✅ `HEARTBEAT` - From both boards

**Note**: Encoder data (quadrature, wheel, AS5600) is used locally on each board and not proxied. Each board has its own EKF for local slip calculation.

## Complete System Integration

The distributed uORB system now provides optimized coverage for:

1. **Command Distribution**: Essential control commands from X7+ reach appropriate NXT boards
2. **Status Collection**: Critical status data from NXT boards reaches X7+ main
3. **Slip-Aware Traction Control**: Distributed slip calculation with centralized traction control
4. **Electric Motor Control**: Differentiated boom (velocity) and bucket (angle) control
5. **Steering Integration**: Servo-based steering control with feedback
6. **Safety Monitoring**: Comprehensive heartbeat and status monitoring
7. **Local Processing**: Sensor data (encoders, AS5600) processed locally for optimal performance

The system supports the complete wheel loader operation cycle including manual RC control, autonomous operation, and advanced traction management with proper architectural separation between boards. Data that can be processed locally (encoders, EKF) stays local to reduce network overhead and improve real-time performance.