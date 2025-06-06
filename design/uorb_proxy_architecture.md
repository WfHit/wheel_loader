# Distributed uORB System Architecture

## Overview

The Distributed uORB System enables control of a wheel loader with multiple controller boards connected via UART. It extends PX4's uORB messaging system across multiple boards, creating a transparent distributed messaging architecture where remote boards appear as local uORB nodes.

## System Architecture

### Hardware Configuration

```
┌────────────────────────────────────────────────────────────────────┐
│                        CUAV X7+ (Main Board)                       │
│  ┌─────────────────┐  ┌──────────────────┐  ┌─────────────────┐  │
│  │  Wheel Loader   │  │ Traction Control │  │   uORB UART     │  │
│  │   Controller    │◄─┤    Algorithm     │◄─┤     Bridge      │  │
│  └─────────────────┘  └──────────────────┘  └────┬────────┬───┘  │
│                            uORB Bus               │        │      │
└───────────────────────────────────────────────────┼────────┼──────┘
                                              UART1 │        │ UART2
                                                    ▼        ▼
┌─────────────────────────────┐            ┌─────────────────────────────┐
│    Front NXT-Dual Board     │            │     Rear NXT-Dual Board     │
│  ┌───────────────────────┐  │            │  ┌───────────────────────┐  │
│  │   uORB UART Proxy     │  │            │  │   uORB UART Proxy     │  │
│  └───────────┬───────────┘  │            │  └───────────┬───────────┘  │
│              │ Virtual uORB  │            │              │ Virtual uORB  │
│  ┌───────────▼───────────┐  │            │  ┌───────────▼───────────┐  │
│  │   Front Actuator      │  │            │  │    Rear Actuator      │  │
│  │   Control Module      │  │            │  │   Control Module      │  │
│  └───────────────────────┘  │            │  └───────────────────────┘  │
│  • Front Left/Right Wheel   │            │  • Rear Left/Right Wheel    │
│  • Bucket Control           │            │  • Boom Control             │
│                             │            │  • Steering Servo           │
└─────────────────────────────┘            └─────────────────────────────┘
```

### Key Components

#### 1. **CUAV X7+ Main Board**
- Runs the main wheel loader controller
- Implements traction control algorithms
- Hosts the uORB UART Bridge module
- Manages bidirectional communication with NXT boards

#### 2. **NXT-Dual Boards**
- **Front Board**: Controls front wheels and bucket
- **Rear Board**: Controls rear wheels, boom, and steering
- Each runs a uORB UART Proxy module
- Provides transparent uORB API for local modules

#### 3. **Communication Protocol**
- UART-based communication (921600 baud)
- Custom binary protocol with CRC16 error checking
- Support for multiple message types (data, subscribe, advertise, heartbeat, time sync)

## Software Architecture

### Directory Structure

```
/workspaces/wheel_loader/
├── src/
│   ├── modules/
│   │   ├── wheel_loader_control/        # Main wheel loader controller
│   │   ├── traction_control/            # Traction control algorithms
│   │   ├── uorb_uart_bridge/           # uORB UART bridge (X7+)
│   │   └── uorb_uart_proxy/            # uORB UART proxy (NXT boards)
│   │
│   ├── drivers/
│   │   └── nxt_dual/                    # NXT-Dual board drivers
│   │       ├── front_actuator/          # Front board actuator control
│   │       ├── rear_actuator/           # Rear board actuator control
│   │       └── nxt_driver/              # Hardware abstraction layer
│   │
│   └── lib/
│       └── distributed_uorb/            # Common protocol/utilities
│           ├── uart_protocol/           # UART frame protocol
│           └── topic_registry/          # Topic definitions
```

### Core Components

#### 1. **uORB UART Bridge (X7+)**
- **Purpose**: Bridges local uORB messages to/from remote boards via UART
- **Key Features**:
  - Manages multiple UART connections to remote nodes
  - Topic-based message routing
  - Automatic discovery of remote publishers/subscribers
  - Performance monitoring and statistics
  - Heartbeat monitoring for fault detection

#### 2. **uORB UART Proxy (NXT Boards)**
- **Purpose**: Provides transparent uORB API on remote boards
- **Key Features**:
  - Implements standard uORB API locally
  - Forwards publish/subscribe requests via UART
  - Local caching of subscribed topics
  - Transparent to user modules
  - Automatic reconnection on link failure

#### 3. **UART Protocol**
- **Frame Structure**:
  ```cpp
  struct UARTFrame {
      uint8_t  sync[2];        // 0xEB, 0x90
      uint8_t  version;        // Protocol version
      uint8_t  msg_type;       // Message type enum
      uint16_t topic_id;       // Topic identifier
      uint16_t payload_len;    // Data length
      uint32_t timestamp;      // Microseconds
      uint8_t  sequence;       // Sequence number
      uint8_t  source_node;    // Node identifier
      uint8_t  payload[256];   // Message data
      uint16_t crc16;          // CRC16 checksum
  };

  enum MessageType {
      MSG_DATA = 0x01,         // Topic data
      MSG_SUBSCRIBE = 0x02,    // Subscribe request
      MSG_ADVERTISE = 0x03,    // Advertise topic
      MSG_HEARTBEAT = 0x04,    // Keep-alive
      MSG_TIME_SYNC = 0x05     // Time synchronization
  };
  ```

### Message Flow

#### 1. **Command Flow (X7+ → NXT)**
```
Controller → orb_publish() → uORB → Bridge → UART → Proxy → orb_publish() → Actuator
```

#### 2. **Feedback Flow (NXT → X7+)**
```
Sensor → orb_publish() → Proxy → UART → Bridge → uORB → orb_copy() → Controller
```

### Topic Mapping

| Topic Name | ID | Direction | Description |
|------------|-----|-----------|-------------|
| wheel_loader_cmd_front | 0x1001 | X7+ → Front | Front wheel and bucket commands |
| wheel_loader_cmd_rear | 0x1003 | X7+ → Rear | Rear wheel, boom, steering commands |
| wheel_encoder_front | 0x2001 | Front → X7+ | Front wheel encoder feedback |
| wheel_encoder_rear | 0x2004 | Rear → X7+ | Rear wheel encoder feedback |
| bucket_feedback | 0x2002 | Front → X7+ | Bucket position feedback |
| boom_feedback | 0x2005 | Rear → X7+ | Boom position feedback |
| steering_feedback | 0x2006 | Rear → X7+ | Steering angle feedback |
| wheel_slip_status | 0x2003/7 | NXT → X7+ | Traction control feedback |

## Key Features

### 1. **Transparent Integration**
- NXT boards appear as local uORB publishers/subscribers to X7+
- No changes required to existing PX4 modules
- Standard uORB API on all boards

### 2. **Traction Control Support**
- Real-time wheel slip detection on NXT boards
- High-level control algorithms on X7+
- Low-latency feedback loop (<5ms)

### 3. **Fault Tolerance**
- Independent UART links for each NXT board
- Heartbeat monitoring
- Graceful degradation on board failure

### 4. **Performance Optimization**
- Efficient binary protocol
- Minimal overhead (<5% CPU usage)
- Configurable update rates
- Message prioritization

### 5. **Future Enhancement: External Communication**
- **Optional Zenoh Gateway**: Bridge uORB messages to external IP networks
- **Use Cases**:
  - Fleet management systems
  - Remote diagnostics and monitoring
  - Cloud-based data analytics
  - Multi-machine coordination
- **Note**: Current system uses UART only. Zenoh gateway would be an additional module.

## Usage

### Starting the System

**On X7+ (Main Board):**
```bash
# Start uORB UART bridge
uorb_uart_bridge start

# Start wheel loader controller with traction control
wheel_loader_control start --enable-traction

# Monitor status
uorb_uart_bridge status
```

**On NXT Boards:**
```bash
# Front NXT - Start proxy and actuator module
uorb_uart_proxy start --node front --uart /dev/ttyUSB0
front_actuator_control start

# Rear NXT - Start proxy and actuator module
uorb_uart_proxy start --node rear --uart /dev/ttyUSB0
rear_actuator_control start
```

### Configuration

**X7+ Parameters:**
- `UUB_ENABLED`: Enable/disable bridge (default: 1)
- `UUB_FRONT_DEV`: Front UART device (default: /dev/ttyS1)
- `UUB_REAR_DEV`: Rear UART device (default: /dev/ttyS2)
- `UUB_BAUD`: UART baud rate (default: 921600)
- `UUB_HB_TIMEOUT`: Heartbeat timeout in ms (default: 1000)

**NXT Proxy Parameters:**
- `UUP_CACHE_SIZE`: Local topic cache size (default: 10)
- `UUP_RETRY_INT`: Reconnection interval in ms (default: 500)

### Example Module Implementation

**Front Actuator Control (NXT Board):**
```cpp
// This module runs on NXT board and uses standard uORB API
// The uorb_uart_proxy handles all communication transparently

#include <px4_platform_common/module.h>
#include <uORB/uORB.h>
#include <uORB/topics/wheel_loader_cmd_front.h>
#include <uORB/topics/wheel_encoder_front.h>

class FrontActuatorControl : public ModuleBase<FrontActuatorControl>
{
public:
    FrontActuatorControl() = default;
    ~FrontActuatorControl() = default;

    static int task_spawn(int argc, char *argv[]);
    static int custom_command(int argc, char *argv[]);
    static int print_usage(const char *reason = nullptr);

    void run() override;

private:
    int _cmd_sub{-1};
    orb_advert_t _encoder_pub{nullptr};

    void process_commands();
    void publish_feedback();
};

void FrontActuatorControl::run()
{
    // Subscribe to commands - proxy handles UART transparently
    _cmd_sub = orb_subscribe(ORB_ID(wheel_loader_cmd_front));

    // Advertise encoder feedback
    wheel_encoder_front_s encoder{};
    _encoder_pub = orb_advertise(ORB_ID(wheel_encoder_front), &encoder);

    while (!should_exit()) {
        // Check for commands
        bool updated;
        orb_check(_cmd_sub, &updated);

        if (updated) {
            wheel_loader_cmd_front_s cmd{};
            orb_copy(ORB_ID(wheel_loader_cmd_front), _cmd_sub, &cmd);

            // Apply to hardware
            set_wheel_speed(WHEEL_LEFT, cmd.wheel_speed_left);
            set_wheel_speed(WHEEL_RIGHT, cmd.wheel_speed_right);
            set_bucket_position(cmd.bucket_position);
        }

        // Publish encoder feedback
        encoder.timestamp = hrt_absolute_time();
        encoder.wheel_speed_left = read_encoder(WHEEL_LEFT);
        encoder.wheel_speed_right = read_encoder(WHEEL_RIGHT);
        encoder.slip_ratio_left = calculate_slip(WHEEL_LEFT);
        encoder.slip_ratio_right = calculate_slip(WHEEL_RIGHT);

        orb_publish(ORB_ID(wheel_encoder_front), _encoder_pub, &encoder);

        px4_usleep(10000); // 100Hz
    }
}
```

## Performance Characteristics

- **Latency**: <5ms round-trip (command → actuator → feedback)
- **Throughput**: Up to 92KB/s per UART link @ 921600 baud
- **CPU Usage**: <5% on X7+ bridge, <10% on NXT proxy
- **Memory**: ~50KB on X7+, ~30KB on NXT boards
- **Reliability**: >99.9% message delivery with CRC16

## Future Enhancements

### Optional Zenoh Gateway
For external monitoring and fleet management:
```
X7+ [uORB] <--UART--> NXT Boards (Real-time control)
 |
 +--[Zenoh Gateway]--> Cloud/Monitoring Systems
```

Benefits:
- Remote diagnostics and monitoring
- Fleet management integration
- Cloud-based data analytics
- Multi-machine coordination

### Other Enhancements
1. **CAN Bus Support**: Alternative to UART for higher reliability
2. **Message Compression**: For bandwidth-limited scenarios
3. **Encryption**: For secure communication
4. **Dynamic Discovery**: Automatic board detection
5. **Multi-hop Routing**: Support for daisy-chained boards

## Troubleshooting

### Common Issues

1. **No Communication**
   - Check UART connections and termination
   - Verify baud rate settings match
   - Monitor heartbeat timeout logs
   - Check CRC error counters

2. **High Latency**
   - Reduce topic update rates
   - Check for UART buffer overflows
   - Monitor CPU usage on both boards
   - Verify no other processes using UART

3. **Message Loss**
   - Check CRC error statistics
   - Verify UART cable shielding
   - Monitor sequence number gaps
   - Check for electromagnetic interference

### Debug Commands
```bash
# View bridge statistics
uorb_uart_bridge status

# Monitor topic flow
uorb top

# Check proxy status on NXT
uorb_uart_proxy status

# View error counters
uorb_uart_bridge diag
```

## References

- [PX4 uORB Documentation](https://docs.px4.io/main/en/middleware/uorb.html)
- [Distributed Systems in PX4](https://docs.px4.io/main/en/advanced/distributed_systems.html)
- [Wheel Loader Control Architecture](./wheel_loader_architecture.md)
