# Distributed uORB Message Mapping

## Overview

This document outlines the comprehensive message flow between the X7+ main board and NXT front/rear boards for the wheel loader system, including sensor data, control commands, and status feedback.

## Message Flow

### X7+ Main Board → NXT Boards (via uorb_uart_bridge)

| Message | Target | Purpose | Status |
|---------|--------|---------|---------|
| `wheel_loader_setpoint` | Both NXT boards | High-level wheel loader commands | ✅ Implemented |
| `actuator_outputs` (front) | NXT Front | Front axle + bucket actuator commands | ✅ Implemented |
| `actuator_outputs` (rear) | NXT Rear | Rear axle + boom actuator commands | ✅ Implemented |
| `vehicle_status` | Both NXT boards | System status information | ✅ Implemented |
| `traction_control` | Both NXT boards | Traction control commands from main board | ✅ Implemented |
| `boom_command` | NXT Rear | Boom hydraulic control commands (angle/height) | ✅ Implemented |
| `bucket_command` | NXT Front | Bucket hydraulic control commands (velocity) | ✅ Implemented |
| `steering_command` | NXT Rear | Steering control commands (ST2135 servo) | ✅ Implemented |
| `vehicle_local_position` | Both NXT boards | EKF position data for slip calculation | ✅ Implemented |
| `vehicle_attitude` | Both NXT boards | EKF attitude data for motion planning | ✅ Implemented |
| `vehicle_odometry` | Both NXT boards | EKF odometry data for slip estimation | ✅ Implemented |
| `heartbeat` | Both NXT boards | Keep-alive and connectivity monitoring | ✅ Implemented |

### NXT Boards → X7+ Main Board (via uorb_uart_proxy)

| Message | Source | Purpose | Status |
|---------|--------|---------|---------|
| `wheel_loader_status` (front) | NXT Front | Front axle and bucket status feedback | ✅ Implemented |
| `wheel_loader_status` (rear) | NXT Rear | Rear axle and boom status feedback | ✅ Implemented |
| `sensor_quad_encoder` (front) | NXT Front | Front motor quadrature encoder data | ✅ Implemented |
| `sensor_quad_encoder` (rear) | NXT Rear | Rear motor quadrature encoder data | ✅ Implemented |
| `sensor_as5600_boom` | NXT Rear | Boom angle sensor data (AS5600) | ✅ Implemented |
| `limit_sensor_bucket` | NXT Front | Bucket limit switch sensor data | ✅ Implemented |
| `wheel_encoders` (front) | NXT Front | Front wheel encoder speed data | ✅ Implemented |
| `wheel_encoders` (rear) | NXT Rear | Rear wheel encoder speed data | ✅ Implemented |
| `slip_estimation` (front) | NXT Front | Front axle slip calculation results | ✅ Implemented |
| `slip_estimation` (rear) | NXT Rear | Rear axle slip calculation results | ✅ Implemented |
| `boom_status` | NXT Rear | Boom hydraulic system status and feedback | ✅ Implemented |
| `bucket_status` | NXT Front | Bucket hydraulic system status and feedback | ✅ Implemented |
| `steering_status` | NXT Rear | Steering system status and feedback | ✅ Implemented |
| `heartbeat` | Both NXT boards | Keep-alive from NXT boards | ✅ Implemented |

## Hardware Configuration

### Front NXT Board
- **H-Bridge Device 1** (2 channels):
  - Channel 1: Front axle drive motor
  - Channel 2: Bucket hydraulic control (velocity-based)
- **Sensors**:
  - Quadrature encoder (front motor speed)
  - Wheel encoders (front axle speed measurement)
  - Limit sensors (bucket position limits)
- **Control Systems**:
  - Front axle motor controller with PID and encoder feedback
  - Bucket velocity controller with encoder position tracking
  - Slip estimation using encoder vs. EKF speed comparison

### Rear NXT Board  
- **H-Bridge Device 2** (2 channels):
  - Channel 1: Rear axle drive motor
  - Channel 2: Boom hydraulic control (angle/height-based)
- **Servo Control**:
  - ST2135 servo for vehicle steering/articulation
- **Sensors**:
  - Quadrature encoder (rear motor speed)
  - Wheel encoders (rear axle speed measurement)
  - AS5600 angle sensor (boom position)
- **Control Systems**:
  - Rear axle motor controller with PID and encoder feedback
  - Boom position controller with AS5600 feedback and motion planning
  - Steering servo controller for vehicle articulation
  - Slip estimation using encoder vs. EKF speed comparison

## Control System Architecture

### Slip Estimation and Traction Control
- **Local Calculation**: Each NXT board calculates slip using local encoder data vs. EKF speed from X7+
- **Central Distribution**: X7+ receives slip data from both boards and runs traction distribution function
- **Feedback Loop**: Traction control commands sent back to NXT boards for motor power adjustment

### Hydraulic Control Systems
- **Boom Control (NXT Rear)**:
  - Uses AS5600 sensor for absolute angle measurement
  - Motion planner generates trajectory commands
  - PID controller ensures adherence to motion plan
  - Position-based control (angle/height setpoints)

- **Bucket Control (NXT Front)**:
  - Uses motor encoder for relative position tracking
  - Requires zero action on init (limit switch calibration)
  - Velocity-based control for smooth operation
  - Complex control modes (manual, auto-level, grading, transport)

### Motor Control Systems
- **Front and Rear Axles**:
  - Individual motor controllers for each axle
  - Quadrature encoders attached to motors for speed feedback
  - PID speed control with encoder input
  - Real speed measurement from EKF for slip calculation
  - Comparison between encoder speed and EKF speed for slip detection

## Message Processing Verification

### Bridge (X7+ → NXT)
- ✅ `WHEEL_LOADER_SETPOINT` - Processed and forwarded to both boards
- ✅ `ACTUATOR_OUTPUTS_FRONT` - Filtered to front board only
- ✅ `ACTUATOR_OUTPUTS_REAR` - Filtered to rear board only  
- ✅ `VEHICLE_STATUS` - Broadcast to both boards
- ✅ `TRACTION_CONTROL` - Broadcast to both boards for local motor control
- ✅ `BOOM_COMMAND` - Sent to rear board only (boom control)
- ✅ `BUCKET_COMMAND` - Sent to front board only (bucket control)
- ✅ `STEERING_COMMAND` - Sent to rear board only (steering servo)
- ✅ `VEHICLE_LOCAL_POSITION` - Broadcast for slip calculation
- ✅ `VEHICLE_ATTITUDE` - Broadcast for motion planning
- ✅ `VEHICLE_ODOMETRY` - Broadcast for slip estimation
- ✅ `HEARTBEAT` - Periodic keep-alive

### Proxy (NXT → X7+)
- ✅ `WHEEL_LOADER_STATUS_FRONT` - From front board only
- ✅ `WHEEL_LOADER_STATUS_REAR` - From rear board only
- ✅ `SENSOR_QUAD_ENCODER_FRONT` - Motor encoder data from front
- ✅ `SENSOR_QUAD_ENCODER_REAR` - Motor encoder data from rear
- ✅ `SENSOR_AS5600_BOOM` - Boom angle sensor from rear board
- ✅ `LIMIT_SENSOR_BUCKET` - Bucket limit switches from front board
- ✅ `WHEEL_ENCODERS_FRONT` - Wheel speed data from front
- ✅ `WHEEL_ENCODERS_REAR` - Wheel speed data from rear
- ✅ `SLIP_ESTIMATION_FRONT` - Slip calculation results from front
- ✅ `SLIP_ESTIMATION_REAR` - Slip calculation results from rear
- ✅ `BOOM_STATUS` - Boom system status from rear board
- ✅ `BUCKET_STATUS` - Bucket system status from front board
- ✅ `STEERING_STATUS` - Steering system status from rear board
- ✅ `HEARTBEAT` - From both boards

## Complete System Integration

The distributed uORB system now provides complete coverage for:

1. **Command Distribution**: All control commands from X7+ reach appropriate NXT boards
2. **Sensor Data Collection**: All sensor data from NXT boards reaches X7+ main
3. **Slip-Aware Traction Control**: Distributed slip calculation with centralized traction control
4. **Hydraulic Control**: Differentiated boom (position) and bucket (velocity) control
5. **Steering Integration**: Servo-based steering control with feedback
6. **Safety Monitoring**: Comprehensive heartbeat and status monitoring
7. **Motion Planning**: EKF data available on NXT boards for local motion planning

The system supports the complete wheel loader operation cycle including manual RC control, autonomous operation, and advanced traction management with proper architectural separation between boards.