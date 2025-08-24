# Sensor Quad Encoder Module Updates

## Overview
Updated all modules that use the `sensor_quad_encoder` message to work with the new individual instance-based approach instead of the previous array-based approach.

## Changes Made

### 1. Message Structure Change
**Previous**: Array-based message with `count` field and indexed arrays
```cpp
sensor_quad_encoder_s encoder_data;
if (encoder_data.count > idx && encoder_data.valid[idx]) {
    position = encoder_data.position[idx];
    velocity = encoder_data.velocity[idx];
}
```

**New**: Individual instance-based messages
```cpp
sensor_quad_encoder_s encoder_data;
if (encoder_data.instance == expected_instance) {
    position = encoder_data.position * 1e-6f; // Convert from 1/million units
    velocity = encoder_data.velocity * 1e-6f; // Convert from 1/million units
}
```

### 2. Module Updates

#### BucketControl Module
**File**: `src/modules/bucket_control/bucket_control.hpp`, `bucket_control.cpp`

**Changes**:
- Updated subscription from single to multi-instance: `uORB::SubscriptionMultiArray<sensor_quad_encoder_s>`
- Modified `readEncoderFeedback()` to use instance-based access
- Added unit conversion from 1/million rad to encoder counts
- Added instance validation

#### WheelController Module
**File**: `src/modules/articulated_chassis/wheel_controller/wheel_controller.hpp`, `wheel_controller.cpp`

**Changes**:
- Updated subscription to use specific instance: `uORB::Subscription _sensor_quad_encoder_sub{ORB_ID(sensor_quad_encoder), _instance}`
- Modified `process_encoder_data()` to validate instance ID
- Added unit conversion from 1/million rad to radians
- Simplified logic by removing array indexing

#### SlipEstimator Module
**File**: `src/modules/articulated_chassis/slip_estimator/slip_estimator.hpp`, `slip_estimator.cpp`

**Changes**:
- Updated `updateVehicleState()` to read from specific front/rear instances
- Added instance validation for both front and rear encoders
- Added unit conversion from 1/million rad/s to rad/s
- Removed loop-based processing in favor of direct instance access

### 3. Distributed uORB Integration

#### uORB Proxy (NXT Boards)
**File**: `src/modules/uorb_uart_proxy/uorb_uart_proxy.hpp`, `uorb_uart_proxy.cpp`

**Additions**:
- Added sensor quad encoder subscriptions for instances 0 and 1
- Added outgoing message handlers for `SENSOR_QUAD_ENCODER_FRONT/REAR`
- Board-specific message ID routing

#### uORB Bridge (Main Board)
**File**: `src/modules/uorb_uart_bridge/uorb_uart_bridge.hpp`, `uorb_uart_bridge.cpp`

**Additions**:
- Added sensor quad encoder publications for all 4 instances (front 0/1, rear 0/1)
- Added incoming message handlers for front/rear encoder data
- Instance remapping for global consistency:
  - Front board: instances 0,1 → global instances 0,1
  - Rear board: instances 0,1 → global instances 2,3

### 4. Instance Mapping Strategy

**Global Instance Map**:
- Instance 0: Front board, local instance 0
- Instance 1: Front board, local instance 1
- Instance 2: Rear board, local instance 0 (remapped from 0→2)
- Instance 3: Rear board, local instance 1 (remapped from 1→3)

**Parameter Mapping**:
- `SE_FRONT_ENCODER_IDX`: Points to global instance (0 or 1 for front)
- `SE_REAR_ENCODER_IDX`: Points to global instance (2 or 3 for rear)
- `BC_ENCODER_INDEX`: Points to appropriate bucket encoder instance

### 5. Network Protocol
**Message IDs**:
- `SENSOR_QUAD_ENCODER_FRONT` (0x40): Front board encoder data
- `SENSOR_QUAD_ENCODER_REAR` (0x41): Rear board encoder data

**Data Flow**:
```
NXT Front Board → UART → X7+ Main Board
[Instance 0,1]            [Republish as 0,1]

NXT Rear Board → UART → X7+ Main Board
[Instance 0,1]           [Republish as 2,3]
```

### 6. Unit Conversions
The new message format uses micro-precision units:
- **Position**: 1/million radians (for rotary) or mm (for linear)
- **Velocity**: 1/million rad/s (for rotary) or mm/s (for linear)

**Conversion Examples**:
```cpp
// Position from message to radians
float position_rad = encoder_data.position * 1e-6f;

// Velocity from message to rad/s
float velocity_rad_s = encoder_data.velocity * 1e-6f;

// Position to encoder counts (bucket control example)
int32_t counts = static_cast<int32_t>(position_rad / encoder_scale);
```

### 7. Error Handling
- Instance validation: Check `encoder_data.instance == expected_instance`
- Bounds checking: Verify instance < max_instances before subscription access
- Unit validation: Handle micro-precision conversion carefully
- Timeout handling: Maintained existing timeout logic for encoder data freshness

## Testing Verification

### Module-Level Testing
```bash
# Test bucket control
bucket_control start
listener bucket_status

# Test wheel controller
wheel_controller start 0  # Front wheels
wheel_controller start 1  # Rear wheels
listener wheel_controller_status

# Test slip estimator
slip_estimator start
listener slip_estimation
```

### Distributed System Testing
```bash
# On main board (X7+)
listener sensor_quad_encoder  # Should show instances 0,1,2,3

# On front board
uorb_uart_proxy start
# Should transmit local instances 0,1

# On rear board
uorb_uart_proxy start
# Should transmit local instances 0,1 (remapped to 2,3)
```

### Parameter Configuration
```bash
# Set encoder indices for slip estimator
param set SE_FRONT_ENCODER_IDX 0  # Use front board instance 0
param set SE_REAR_ENCODER_IDX 2   # Use rear board instance 0 (global 2)

# Set bucket encoder index
param set BC_ENCODER_INDEX 1     # Use appropriate encoder instance
```

## Benefits Achieved
1. **Scalable Architecture**: Supports any number of encoder instances per board
2. **Instance Isolation**: Each encoder publishes independently
3. **Network Efficiency**: Only active encoders transmit data
4. **Type Safety**: Eliminates array bounds errors
5. **Distributed Ready**: Seamless integration with multi-board systems
6. **Parameter Flexibility**: Runtime configuration of encoder assignments

## Migration Path
Existing configurations will need parameter updates:
- Review `SE_FRONT_ENCODER_IDX` and `SE_REAR_ENCODER_IDX` settings
- Verify `BC_ENCODER_INDEX` points to correct encoder
- Test encoder assignments after parameter changes
- Validate distributed system encoder routing
