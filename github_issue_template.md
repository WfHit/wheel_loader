# GitHub Issue: Update Message Types in uORB UART Bridge and Proxy Modules

## 🎯 Issue Title
**Add New Messages and Remove Deprecated Types in uORB UART Communication System**

## 📋 Description
The `uorb_uart_bridge` and `uorb_uart_proxy` modules need to be updated to support the comprehensive set of wheel loader messages available in the system while removing deprecated legacy message types. Currently, the UART communication only supports a minimal set of messages, limiting the distributed system's capabilities.

## 🔍 Current State Analysis

### Currently Supported Messages (Limited Set):
```cpp
enum class UartMessageId : uint8_t {
    HEARTBEAT = 0x01,
    WHEEL_LOADER_SETPOINT = 0x10,           // ⚠️ DEPRECATED - replaced by WheelLoaderCommand
    WHEEL_LOADER_STATUS_FRONT = 0x11,
    WHEEL_LOADER_STATUS_REAR = 0x12,
    ACTUATOR_OUTPUTS_FRONT = 0x20,
    ACTUATOR_OUTPUTS_REAR = 0x21,
    VEHICLE_STATUS = 0x30
};
```

### Legacy Message Types (TO BE REMOVED):
```cpp
enum class MessageType : uint8_t {
    DATA = 0x01,        // ❌ DEPRECATED - no longer used
    SUBSCRIBE = 0x02,   // ❌ DEPRECATED - no longer used
    ADVERTISE = 0x03,   // ❌ DEPRECATED - no longer used
    HEARTBEAT = 0x04,   // ❌ DEPRECATED - conflicts with UartMessageId
    TIME_SYNC = 0x05,   // ❌ DEPRECATED - no longer used
    ACK = 0x06,         // ❌ DEPRECATED - no longer used
    NACK = 0x07         // ❌ DEPRECATED - no longer used
};
```

## 🎯 New Messages to Add

### High Priority Messages (Critical for wheel loader operation):
```cpp
// Hydraulic Control Messages
WHEEL_LOADER_COMMAND = 0x13,      // Replaces WHEEL_LOADER_SETPOINT
BOOM_COMMAND = 0x14,              // Boom hydraulic control
BOOM_STATUS = 0x15,               // Boom status feedback
BUCKET_COMMAND = 0x16,            // Bucket control commands
BUCKET_STATUS = 0x17,             // Bucket status feedback

// Drivetrain and Mobility
WHEEL_SPEEDS_SETPOINT = 0x22,     // Individual wheel speed control
WHEEL_ENCODERS = 0x23,            // Wheel encoder feedback
STEERING_COMMAND = 0x24,          // Steering control
STEERING_STATUS = 0x25,           // Steering feedback

// Traction and Stability Control
TRACTION_CONTROL = 0x26,          // Traction control commands
TRACTION_CONTROL_STATUS = 0x27,   // Traction control feedback
PREDICTIVE_TRACTION = 0x28,       // MPC-based traction control

// Load Management
LOAD_SENSING = 0x29,              // Load sensing data
LOAD_AWARE_TORQUE = 0x2A,         // Load-aware torque control
```

### Medium Priority Messages (Enhanced functionality):
```cpp
// Task Execution and Autonomy
TASK_EXECUTION_COMMAND = 0x40,    // Autonomous task commands
TERRAIN_ADAPTATION = 0x41,        // Terrain adaptation data

// System Monitoring
MODULE_STATUS = 0x42,             // Individual module status
SYSTEM_SAFETY = 0x43,             // Safety system status
LIMIT_SENSOR = 0x44,              // Limit switch status

// Advanced Control
WHEEL_DRIVE_COMMAND = 0x45,       // Advanced wheel drive control
SLIP_ESTIMATION = 0x46,           // Slip estimation data
```

## 🔧 Implementation Tasks

### Phase 1: Remove Deprecated Messages
1. **Clean up legacy enums**
   - [ ] Remove `MessageType` enum entirely
   - [ ] Remove `NodeId` enum (use board IDs instead)
   - [ ] Remove any code references to deprecated message types

2. **Update protocol documentation**
   - [ ] Remove references to legacy protocol in comments
   - [ ] Update architecture documentation

### Phase 2: Add New Message Support
1. **Update protocol definition**
   - [ ] Add new message IDs to `UartMessageId` enum
   - [ ] Ensure no ID conflicts with existing messages
   - [ ] Update maximum payload size if needed

2. **Update Bridge Module (`uorb_uart_bridge`)**
   - [ ] Add new uORB subscriptions for outgoing messages:
     ```cpp
     uORB::Subscription _wheel_loader_command_sub{ORB_ID(wheel_loader_command)};
     uORB::Subscription _boom_command_sub{ORB_ID(boom_command)};
     uORB::Subscription _bucket_command_sub{ORB_ID(bucket_command)};
     uORB::Subscription _traction_control_sub{ORB_ID(traction_control)};
     // ... additional subscriptions
     ```
   - [ ] Add new uORB publications for incoming messages:
     ```cpp
     uORB::Publication<boom_status_s> _boom_status_pub{ORB_ID(boom_status)};
     uORB::Publication<bucket_status_s> _bucket_status_pub{ORB_ID(bucket_status)};
     uORB::Publication<wheel_encoders_s> _wheel_encoders_pub{ORB_ID(wheel_encoders)};
     // ... additional publications
     ```
   - [ ] Update `processOutgoingMessages()` to handle new message types
   - [ ] Update `processIncomingMessages()` to handle new message types

3. **Update Proxy Module (`uorb_uart_proxy`)**
   - [ ] Mirror bridge subscriptions/publications in reverse
   - [ ] Update message processing functions
   - [ ] Ensure board-specific filtering (front vs rear)

### Phase 3: Migration Support
1. **Backward compatibility**
   - [ ] Support both old and new message formats during transition
   - [ ] Add protocol version negotiation
   - [ ] Graceful handling of unknown message types

2. **Parameter updates**
   - [ ] Add parameters to enable/disable specific message types
   - [ ] Add message rate limiting parameters
   - [ ] Update documentation for new parameters

## 📋 Detailed File Changes Required

### Core Protocol Files:
- **`/src/lib/distributed_uorb/uart_protocol/uart_protocol.hpp`**
  - Remove `MessageType` and `NodeId` enums
  - Add new message IDs to `UartMessageId`
  - Update comments and documentation

### Bridge Module Files:
- **`/src/modules/uorb_uart_bridge/uorb_uart_bridge.hpp`**
  - Add new uORB includes
  - Add new subscription/publication member variables

- **`/src/modules/uorb_uart_bridge/uorb_uart_bridge.cpp`**
  - Add cases for new message types in processing functions
  - Remove any legacy message handling code

### Proxy Module Files:
- **`/src/modules/uorb_uart_proxy/uorb_uart_proxy.hpp`**
  - Mirror bridge changes for proxy module

- **`/src/modules/uorb_uart_proxy/uorb_uart_proxy.cpp`**
  - Add message handling for new types
  - Remove legacy code

### Supporting Files:
- **`/src/lib/distributed_uorb/topic_registry/topic_registry.hpp`**
  - Add topic ID mappings for new messages

## 🧪 Testing Requirements

### Unit Tests:
- [ ] Test message serialization/deserialization for all new types
- [ ] Test backward compatibility during migration
- [ ] Test error handling for unknown message types
- [ ] Verify memory usage doesn't exceed limits

### Integration Tests:
- [ ] End-to-end message flow tests for all new message types
- [ ] Performance tests with full message load
- [ ] Cross-board communication tests (X7+ ↔ NXT)
- [ ] Fault tolerance tests (message corruption, timeouts)

### Hardware Tests:
- [ ] Real UART communication with full message set
- [ ] Load testing with high message rates
- [ ] Power consumption impact assessment

## 📊 Success Metrics

### Functional Metrics:
- [ ] All 15+ new message types successfully transmitted
- [ ] Zero data corruption in normal operation
- [ ] Backward compatibility maintained during transition
- [ ] <1% packet loss under normal conditions

### Performance Metrics:
- [ ] Message latency <5ms for high-priority messages
- [ ] Support for 200+ messages/second throughput
- [ ] Memory overhead <100KB total
- [ ] CPU overhead <3% additional load

### Quality Metrics:
- [ ] Unit test coverage >95% for new code
- [ ] Zero memory leaks in continuous operation
- [ ] Code review approval from 2+ developers
- [ ] Documentation updated for all changes

## 📋 Acceptance Criteria

### Phase 1 Complete:
- [ ] All legacy message types removed
- [ ] No compilation errors or warnings
- [ ] Existing functionality preserved

### Phase 2 Complete:
- [ ] All new message types implemented
- [ ] Bridge and proxy modules handle new messages
- [ ] Integration tests pass

### Phase 3 Complete:
- [ ] Migration path validated
- [ ] Performance requirements met
- [ ] Hardware testing successful

## 🔗 Related Files and Resources

### Message Definitions (Reference):
- `/msg/WheelLoaderCommand.msg` - New comprehensive wheel loader control
- `/msg/BoomCommand.msg` / `/msg/BoomStatus.msg` - Boom control
- `/msg/BucketCommand.msg` / `/msg/BucketStatus.msg` - Bucket control
- `/msg/TractionControl.msg` / `/msg/TractionControlStatus.msg` - Traction control
- `/msg/WheelEncoders.msg` - Wheel feedback
- `/msg/LoadSensing.msg` - Load management

### Architecture Documentation:
- `/design/uorb_proxy_architecture.md` - System architecture
- `/src/lib/distributed_uorb/README.md` - Implementation details

### Current Implementation:
- `/src/modules/uorb_uart_bridge/` - Bridge module
- `/src/modules/uorb_uart_proxy/` - Proxy module
- `/src/lib/distributed_uorb/uart_protocol/` - Protocol implementation

## 🏷️ Labels
`enhancement`, `communication`, `uorb`, `wheel-loader`, `protocol-update`, `message-handling`, `breaking-change`

## 👤 Agent Assignment Requirements

**Skills Required:**
- Strong C/C++ embedded systems experience
- Understanding of publish/subscribe messaging patterns
- Knowledge of UART communication protocols
- Experience with PX4/ArduPilot or similar systems (preferred)

**Estimated Effort:** 5-7 days for experienced developer

**Prerequisites:**
- Access to wheel loader system documentation
- Understanding of hydraulic system control
- Familiarity with distributed embedded systems

**Deliverables:**
1. Updated protocol with new message types
2. Modified bridge and proxy modules
3. Comprehensive test suite
4. Updated documentation
5. Migration guide for existing deployments
