# H-Bridge Module Updates for Multi-Instance Support

## Overview

Updated all modules### 2. Updated Modules

#### A. BucketControl (`src/modules/bucket_control/`)at interact with the H-Bridge driver to support the new multi-instance architecture. The key changes involve switching from single-instance publications/subscriptions to multi-instance arrays and updating message field names.

## Key Changes

### 1. Message Structure Updates

**HbridgeCommand.msg**:
- Changed `channel` field to `instance` for consistency
- Supports targeting specific H-bridge instances (0 or 1)

**HbridgeStatus.msg**:
- Uses `instance` field to identify which H-bridge is reporting
- Provides per-instance status information

### 2. New Messages

#### LoadLampCommand.msg - **NEW**

**Purpose**: Commands load indication lamps based on centralized motor load analysis.

**Structure**:
```
uint64 timestamp         # time since system start (microseconds)

# Load level constants
uint8 LOAD_VERY_LOW = 0  # 0-10% load: Very slow blink (0.5 Hz)
uint8 LOAD_LOW = 1       # 10-20% load: Slow blink (1 Hz)
uint8 LOAD_MED_LOW = 2   # 20-30% load: Medium-slow blink (2 Hz)
uint8 LOAD_MEDIUM = 3    # 30-60% load: Medium blink (5 Hz)
uint8 LOAD_HIGH = 4      # 60-80% load: Fast blink (10 Hz)
uint8 LOAD_VERY_HIGH = 5 # 80-100% load: Very fast blink (20 Hz)
uint8 LOAD_OFF = 255     # Turn off lamps

uint8 load_level         # Load level (LOAD_*)
float32 load_value       # Actual load value (0.0-1.0) for reference
uint32 blink_interval_us # Blink interval in microseconds
```

**Flow**: Main Board LoadAnalysis → uORB Proxy → Rear Board LoadLampController

### 3. Updated Modules

#### A. BucketControl (`src/modules/bucket_control/`)

**Changes Made**:
- Updated `bucket_control.hpp`:
  - Changed `uORB::Subscription _hbridge_status_sub` to `uORB::SubscriptionMultiArray _hbridge_status_sub`
  - Changed `uORB::Publication<hbridge_command_s>` to `uORB::PublicationMulti<hbridge_command_s>`
  - Added `#include <uORB/SubscriptionMultiArray.hpp>` and `#include <uORB/PublicationMulti.hpp>`
  - Added `bool checkHBridgeStatus()` method declaration

- Updated `bucket_control.cpp`:
  - Modified `setMotorCommand()` to use `cmd.instance` instead of `cmd.channel`
  - Updated to publish to specific message instance: `_hbridge_command_pub.publish(cmd, instance_to_publish)`
  - Implemented `checkHBridgeStatus()` method to monitor H-bridge health
  - Added H-bridge status check to main run loop

**Usage**:
```cpp
// Command publication (targets specific H-bridge instance)
cmd.instance = _motor_index;  // 0 or 1
_hbridge_command_pub.publish(cmd, _motor_index);

// Status monitoring (receives from all instances)
for (auto &sub : _hbridge_status_sub) {
    if (sub.update(&hbridge_status) && hbridge_status.instance == _motor_index) {
        // Process status for our motor
    }
}
```

#### B. BoomControl (`src/modules/boom_control/`)

**Changes Made**:
- Updated `boom_control.hpp`:
  - Changed subscription/publication types to multi-instance versions
  - Added required headers
  - Added `bool check_hbridge_status()` method declaration

- Updated `boom_control.cpp`:
  - Modified `publish_hbridge_command()` to use `cmd.instance` and publish to specific instance
  - Implemented `check_hbridge_status()` method for monitoring

**Usage**:
```cpp
// Command publication
uint8_t hbridge_instance = static_cast<uint8_t>(_param_hbridge_channel.get());
cmd.instance = hbridge_instance;
_hbridge_command_pub.publish(cmd, hbridge_instance);
```

#### C. WheelController (`src/modules/articulated_chassis/wheel_controller/`)

**Changes Made**:
- Updated `wheel_controller.hpp`:
  - Changed `uORB::Publication<hbridge_command_s>` to `uORB::PublicationMulti<hbridge_command_s>`

- Updated `wheel_controller.cpp`:
  - Modified command publication to use `cmd.instance` and publish to specific instance
  - Uses wheel instance (0 for front, 1 for rear) as H-bridge instance

**Usage**:
```cpp
// Command publication (wheel instance maps to H-bridge instance)
cmd.instance = _instance;  // 0 for front wheel, 1 for rear wheel
_hbridge_command_pub.publish(cmd, _instance);
```

### Updated Modules

### A. BucketControl (`src/modules/bucket_control/`)

**Changes Made**:
- Updated `bucket_control.hpp`:
  - Changed `uORB::Subscription _hbridge_status_sub` to `uORB::SubscriptionMultiArray _hbridge_status_sub`
  - Changed `uORB::Publication<hbridge_command_s>` to `uORB::PublicationMulti<hbridge_command_s>`
  - Added `#include <uORB/SubscriptionMultiArray.hpp>` and `#include <uORB/PublicationMulti.hpp>`
  - Added `bool checkHBridgeStatus()` method declaration

- Updated `bucket_control.cpp`:
  - Modified `setMotorCommand()` to use `cmd.instance` instead of `cmd.channel`
  - Updated to publish to specific message instance: `_hbridge_command_pub.publish(cmd, instance_to_publish)`
  - Implemented `checkHBridgeStatus()` method to monitor H-bridge health
  - Added H-bridge status check to main run loop

**Usage**:
```cpp
// Command publication (targets specific H-bridge instance)
cmd.instance = _motor_index;  // 0 or 1
_hbridge_command_pub.publish(cmd, _motor_index);

// Status monitoring (receives from all instances)
for (auto &sub : _hbridge_status_sub) {
    if (sub.update(&hbridge_status) && hbridge_status.instance == _motor_index) {
        // Process status for our motor
    }
}
```

### B. BoomControl (`src/modules/boom_control/`)

**Changes Made**:
- Updated `boom_control.hpp`:
  - Changed subscription/publication types to multi-instance versions
  - Added required headers
  - Added `bool check_hbridge_status()` method declaration

- Updated `boom_control.cpp`:
  - Modified `publish_hbridge_command()` to use `cmd.instance` and publish to specific instance
  - Implemented `check_hbridge_status()` method for monitoring

**Usage**:
```cpp
// Command publication
uint8_t hbridge_instance = static_cast<uint8_t>(_param_hbridge_channel.get());
cmd.instance = hbridge_instance;
_hbridge_command_pub.publish(cmd, hbridge_instance);
```

### C. WheelController (`src/modules/articulated_chassis/wheel_controller/`)

**Changes Made**:
- Updated `wheel_controller.hpp`:
  - Changed `uORB::Publication<hbridge_command_s>` to `uORB::PublicationMulti<hbridge_command_s>`

- Updated `wheel_controller.cpp`:
  - Modified command publication to use `cmd.instance` and publish to specific instance
  - Uses wheel instance (0 for front, 1 for rear) as H-bridge instance

**Usage**:
```cpp
// Command publication (wheel instance maps to H-bridge instance)
cmd.instance = _instance;  // 0 for front wheel, 1 for rear wheel
_hbridge_command_pub.publish(cmd, _instance);
```

### D. LoadLampController (`src/modules/load_lamp_controller/`)

**Changes Made**:
- **ARCHITECTURE CHANGE**: No longer directly subscribes to hbridge_status
- Updated `load_lamp_controller.h`:
  - Changed from `uORB::SubscriptionMultiArray _hbridge_status_sub` to `uORB::Subscription _load_lamp_command_sub`
  - Removed channel-specific load tracking variables
  - Added `#include <uORB/topics/load_lamp_command.h>`

- Updated `load_lamp_controller.cpp`:
  - Completely rewrote to respond to `LoadLampCommand` messages instead of calculating load
  - Simplified `update_load()` to process commands from main board
  - Removed load calculation and channel averaging logic
  - Updated documentation to reflect distributed architecture

**New Architecture**:
```
Main Board (X7+) → Load Analysis → load_lamp_command → Rear Board LoadLampController
```

**Usage**:
```cpp
// Command processing (receives from main board)
load_lamp_command_s cmd;
if (_load_lamp_command_sub.update(&cmd)) {
    _current_load = cmd.load_value;
    _blink_interval_us = cmd.blink_interval_us;
}
```

### E. LoadAnalysis (`src/modules/load_analysis/`) - **NEW MODULE**

**Purpose**: Runs on main board (X7+) to analyze motor load from all H-bridge instances and generate lamp commands.

**Features**:
- Subscribes to hbridge_status from all H-bridge instances via distributed uORB
- Calculates weighted average load across all active channels
- Applies exponential moving average smoothing
- Maps load values to discrete lamp command levels
- Publishes load_lamp_command at configurable rate

**Configuration**:
- `LOAD_ANALYSIS_ALPHA`: Smoothing factor for load data
- `LOAD_THRESHOLD_LOW/MED/HIGH`: Load thresholds for lamp levels
- `LOAD_CMD_RATE_HZ`: Command publication rate

**Usage**:
```bash
# Start on main board only
load_analysis start
```

## Message Instance Mapping

The new system uses parameter-based message instance routing:

- **HBRIDGE_MSG_INST0**: Message instance for H-bridge channel 0 commands (default: 0)
- **HBRIDGE_MSG_INST1**: Message instance for H-bridge channel 1 commands (default: 1)

This allows flexible routing where:
- Bucket motor (channel 0) can publish to message instance 0
- Boom motor (channel 0) can publish to message instance 1
- Front wheel (channel 0) can publish to message instance 0
- Rear wheel (channel 1) can publish to message instance 1

## Benefits

1. **Clean Architecture**: Clear separation between message routing and hardware instances
2. **Flexible Configuration**: Parameter-based routing allows system reconfiguration without code changes
3. **Better Monitoring**: Each module can monitor specific H-bridge status without filtering
4. **Fault Isolation**: Problems with one H-bridge instance don't affect others
5. **Scalability**: Easy to add more H-bridge channels in the future
6. **Distributed Processing**: Load analysis centralized on main board, lamp control on rear board
7. **Network Efficiency**: Only high-level commands sent across boards, not raw sensor data
8. **Centralized Intelligence**: All motor load data processed in one location for system-wide analysis

## Testing Recommendations

1. **Parameter Configuration**: Verify correct HBRIDGE_MSG_INST* parameter settings
2. **Command Routing**: Test that commands reach the correct H-bridge instances
3. **Status Feedback**: Verify modules receive status from their assigned H-bridge instances
4. **Load Monitoring**: Test load lamp controller with multiple active H-bridges
5. **Fault Scenarios**: Test behavior when one H-bridge instance fails

## Migration Notes

**For existing deployments**:
1. Update parameter file with correct HBRIDGE_MSG_INST* values
2. Verify motor instance assignments match hardware configuration
3. Test all control modules individually before full system integration
4. Monitor uORB topic traffic to ensure proper message routing

**For developers**:
- Use `PublicationMulti` for hbridge_command publication
- Use `SubscriptionMultiArray` for hbridge_status subscription
- Always specify target instance when publishing commands
- Filter status messages by instance when processing
