# Distributed uORB Message Mapping

## Overview

This document outlines the message flow between the X7+ main board and NXT front/rear boards for the wheel loader system.

## Message Flow

### X7+ Main Board → NXT Boards (via uorb_uart_bridge)

| Message | Target | Purpose | Status |
|---------|--------|---------|---------|
| `wheel_loader_setpoint` | Both NXT boards | High-level wheel loader commands | ✅ Implemented |
| `actuator_outputs` (front) | NXT Front | Front axle + bucket actuator commands | ✅ Implemented |
| `actuator_outputs` (rear) | NXT Rear | Rear axle + boom actuator commands | ✅ Implemented |
| `vehicle_status` | Both NXT boards | System status information | ✅ Implemented |
| `heartbeat` | Both NXT boards | Keep-alive and connectivity monitoring | ✅ Implemented |

### NXT Boards → X7+ Main Board (via uorb_uart_proxy)

| Message | Source | Purpose | Status |
|---------|--------|---------|---------|
| `wheel_loader_status` (front) | NXT Front | Front axle and bucket status feedback | ✅ Implemented |
| `wheel_loader_status` (rear) | NXT Rear | Rear axle and boom status feedback | ✅ Implemented |
| `heartbeat` | Both NXT boards | Keep-alive from NXT boards | ✅ Implemented |

## Hardware Configuration

### Front NXT Board
- **H-Bridge Device 1** (2 channels):
  - Channel 1: Front axle drive motor
  - Channel 2: Bucket hydraulic control (velocity-based)

### Rear NXT Board  
- **H-Bridge Device 2** (2 channels):
  - Channel 1: Rear axle drive motor
  - Channel 2: Boom hydraulic control (angle/height-based)

## Message Processing Verification

### Bridge (X7+ → NXT)
- ✅ `WHEEL_LOADER_SETPOINT` - Processed and forwarded
- ✅ `ACTUATOR_OUTPUTS_FRONT` - Filtered to front board only
- ✅ `ACTUATOR_OUTPUTS_REAR` - Filtered to rear board only  
- ✅ `VEHICLE_STATUS` - Broadcast to both boards
- ✅ `HEARTBEAT` - Periodic keep-alive

### Proxy (NXT → X7+)
- ✅ `WHEEL_LOADER_STATUS_FRONT` - From front board only
- ✅ `WHEEL_LOADER_STATUS_REAR` - From rear board only
- ✅ `HEARTBEAT` - From both boards

## Missing Messages Analysis

All required messages for the wheel loader system are properly proxied. The current implementation covers:

1. **Command Distribution**: Commands from X7+ reach appropriate NXT boards
2. **Status Feedback**: Status from NXT boards reaches X7+ main
3. **Safety**: Heartbeat monitoring ensures connectivity
4. **Hardware Mapping**: Proper separation of front/rear actuator outputs

No additional message proxying is required for the current wheel loader architecture.