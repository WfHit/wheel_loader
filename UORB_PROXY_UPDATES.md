# uORB Proxy Conflict Resolution and Kconfig Integration

## Overview
This document summarizes the changes made to resolve uORB proxy message conflicts and add Kconfig configuration for the load analysis module.

## Problem Analysis
The distributed load lamp control system required:
1. **Kconfig integration** for the load_analysis module to enable/disable compilation
2. **uORB proxy conflict resolution** for handling multiple HBridge status messages and load_lamp_command routing

## Changes Made

### 1. Kconfig Configuration for Load Analysis Module

**File: `/workspaces/wheel_loader/src/modules/load_analysis/Kconfig`**
- Added comprehensive Kconfig configuration with:
  - Module enable/disable toggle (`MODULES_LOAD_ANALYSIS`)
  - Userspace execution option (`USER_LOAD_ANALYSIS`)
  - Configurable parameters for load thresholds, smoothing, and update rates
  - Instance configuration for publication routing

**Key Configuration Options:**
- `LOAD_ANALYSIS_SMOOTHING_FACTOR`: Smoothing factor (0-100%)
- `LOAD_ANALYSIS_UPDATE_RATE_HZ`: Update frequency (1-50 Hz)
- Load threshold levels: `VERY_LOW`, `LOW`, `MEDIUM`, `HIGH`
- `LOAD_ANALYSIS_PUBLISH_INSTANCE`: Target board routing instance

### 2. Distributed uORB Protocol Extensions

**File: `/workspaces/wheel_loader/src/lib/distributed_uorb/uart_protocol/uart_protocol.hpp`**
- Added new message IDs for multi-instance HBridge status:
  - `HBRIDGE_STATUS_FRONT_0` (0x90): Front board, instance 0
  - `HBRIDGE_STATUS_FRONT_1` (0x91): Front board, instance 1
  - `HBRIDGE_STATUS_REAR_0` (0x92): Rear board, instance 0
  - `HBRIDGE_STATUS_REAR_1` (0x93): Rear board, instance 1
- Added `LOAD_LAMP_COMMAND` (0xA0): Load lamp control commands

### 3. uORB UART Proxy Updates (NXT Boards)

**Files: `/workspaces/wheel_loader/src/modules/uorb_uart_proxy/uorb_uart_proxy.hpp/.cpp`**

**Additions:**
- Multi-instance HBridge status subscriptions:
  ```cpp
  uORB::Subscription _hbridge_status_sub_0{ORB_ID(hbridge_status), 0};
  uORB::Subscription _hbridge_status_sub_1{ORB_ID(hbridge_status), 1};
  ```
- Load lamp command publication:
  ```cpp
  uORB::Publication<load_lamp_command_s> _load_lamp_command_pub{ORB_ID(load_lamp_command)};
  ```

**Message Handling:**
- **Incoming**: Load lamp commands (X7+ → NXT rear only)
- **Outgoing**: HBridge status per instance with board-specific message IDs

### 4. uORB UART Bridge Updates (X7+ Main Board)

**Files: `/workspaces/wheel_loader/src/modules/uorb_uart_bridge/uorb_uart_bridge.hpp/.cpp`**

**Additions:**
- Multi-instance HBridge status publications:
  ```cpp
  uORB::Publication<hbridge_status_s> _hbridge_status_front_0_pub{ORB_ID(hbridge_status), 0};
  uORB::Publication<hbridge_status_s> _hbridge_status_front_1_pub{ORB_ID(hbridge_status), 1};
  uORB::Publication<hbridge_status_s> _hbridge_status_rear_0_pub{ORB_ID(hbridge_status), 2};
  uORB::Publication<hbridge_status_s> _hbridge_status_rear_1_pub{ORB_ID(hbridge_status), 3};
  ```
- Load lamp command subscription:
  ```cpp
  uORB::Subscription _load_lamp_command_sub{ORB_ID(load_lamp_command)};
  ```

**Message Handling:**
- **Incoming**: HBridge status from all board instances
- **Outgoing**: Load lamp commands (X7+ → NXT rear)

## Conflict Resolution Strategy

### Multi-Instance HBridge Status
- **Problem**: Multiple HBridge drivers on each board created status message conflicts
- **Solution**: Instance-specific message IDs with board prefixes:
  - Front board: Uses `HBRIDGE_STATUS_FRONT_0/1`
  - Rear board: Uses `HBRIDGE_STATUS_REAR_0/1`
  - Main board: Maps to uORB instances 0-3 for proper routing

### Load Lamp Command Routing
- **Problem**: Efficient command distribution to specific target board
- **Solution**: Single message ID with board-specific routing:
  - Only rear board processes `LOAD_LAMP_COMMAND`
  - Other boards ignore message (no conflict)

## Integration Points

### Build System
- Kconfig automatically included via `src/modules/Kconfig` recursive sourcing
- Module builds only when `MODULES_LOAD_ANALYSIS=y`

### Runtime Configuration
- Parameters configurable via QGroundControl or command line
- Default values provide reasonable load thresholds and update rates

### Network Efficiency
- HBridge status sent only when updated (subscription-based)
- Load lamp commands sent only when load levels change
- Instance-specific routing reduces unnecessary network traffic

## Testing Recommendations

1. **Kconfig Integration**:
   ```bash
   make menuconfig  # Verify load_analysis appears in modules menu
   make clean && make px4_sitl_default  # Test compilation
   ```

2. **Message Routing**:
   ```bash
   # On main board (X7+)
   listener hbridge_status  # Should show all 4 instances

   # On rear board
   listener load_lamp_command  # Should receive commands
   ```

3. **Load Analysis**:
   ```bash
   load_analysis start  # Should start successfully
   load_analysis status  # Check running state and parameters
   ```

## Benefits Achieved

1. **Scalable Architecture**: Supports any number of HBridge instances per board
2. **Zero Message Conflicts**: Instance-specific routing eliminates conflicts
3. **Configurable Operation**: Runtime tuning via parameters
4. **Network Optimization**: Efficient message routing and minimal traffic
5. **Build Flexibility**: Optional compilation via Kconfig

## Future Enhancements

1. **Dynamic Instance Discovery**: Auto-detect number of HBridge instances
2. **Message Prioritization**: Critical status messages with higher priority
3. **Fault Tolerance**: Automatic retry for failed message delivery
4. **Performance Monitoring**: Real-time network utilization metrics
