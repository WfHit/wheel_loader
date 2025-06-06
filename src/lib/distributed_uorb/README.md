# Distributed uORB System for Wheel Loader

This directory contains the distributed uORB implementation for the wheel loader project, enabling transparent uORB messaging between multiple controller boards via UART.

## System Architecture

The system consists of:

### Hardware Configuration
- **X7+ Main Board**: Central control unit running the bridge module
- **NXT-Dual Front Board**: Front wheel controller running the proxy module
- **NXT-Dual Rear Board**: Rear wheel controller running the proxy module

### Software Components

#### 1. uORB UART Bridge (`uorb_uart_bridge`)
- Runs on X7+ main board
- Forwards uORB messages between local topics and UART
- Manages communication with multiple NXT boards
- Provides heartbeat monitoring and statistics

**Topics:**
- **Outgoing** (X7+ → NXT): `wheel_loader_setpoint`, `actuator_outputs`, `vehicle_status`
- **Incoming** (NXT → X7+): `wheel_loader_status_front`, `wheel_loader_status_rear`

#### 2. uORB UART Proxy (`uorb_uart_proxy`)
- Runs on NXT controller boards
- Provides transparent uORB interface for distributed topics
- Automatically forwards appropriate messages based on board type
- Board-specific filtering (front vs rear commands)

**Topics:**
- **Incoming** (X7+ → NXT): `wheel_loader_setpoint`, `actuator_outputs_*`, `vehicle_status`
- **Outgoing** (NXT → X7+): `wheel_loader_status`

#### 3. Supporting Libraries

##### UART Protocol Library (`uart_protocol`)
- Defines frame structure and message format
- CRC16 validation for data integrity
- Board and message ID definitions

##### UART Transport Library (`uart_transport`)
- Low-level UART communication
- Frame parsing and validation
- Non-blocking I/O with poll-based operation

##### Topic Registry Library (`topic_registry`)
- Topic name to ID mapping
- Publisher/subscriber validation
- Distributed topic metadata

## Configuration

### Parameters

#### Bridge Parameters (X7+ Board)
```
UART_BRIDGE_EN=1                    # Enable bridge module
UART_BRIDGE_DEV="/dev/ttyS2"        # UART device path
UART_BRIDGE_BAUD=115200             # Baudrate
UART_BRIDGE_STAT=10                 # Statistics interval (seconds)
UART_BRIDGE_HB=1000                 # Heartbeat interval (ms)
UART_BRIDGE_TO=3000                 # Timeout for offline detection (ms)
```

#### Proxy Parameters (NXT Boards)
```
UART_PROXY_EN=1                     # Enable proxy module
UART_PROXY_DEV="/dev/ttyS1"         # UART device path
UART_PROXY_BAUD=115200              # Baudrate
UART_PROXY_TYPE=0                   # Board type (0=Front, 1=Rear)
UART_PROXY_STAT=10                  # Statistics interval (seconds)
UART_PROXY_HB=1000                  # Heartbeat interval (ms)
UART_PROXY_TO=3000                  # Timeout for main board (ms)
```

### Build Configuration

Enable the modules in your board configuration:

```cmake
CONFIG_MODULES_UORB_UART_BRIDGE=y     # For X7+ board
CONFIG_MODULES_UORB_UART_PROXY=y      # For NXT boards
CONFIG_MODULES_WHEEL_LOADER=y         # Main wheel loader controller
```

## Usage

### Starting the Bridge (X7+ Board)
```bash
# Start with default parameters
uorb_uart_bridge start

# Start with custom UART device and baudrate
uorb_uart_bridge start -d /dev/ttyS3 -b 230400

# Check status
uorb_uart_bridge status

# Stop
uorb_uart_bridge stop
```

### Starting the Proxy (NXT Boards)
```bash
# Start front wheel controller proxy
uorb_uart_proxy start

# Start with custom parameters
uorb_uart_proxy start -d /dev/ttyS2 -b 230400

# Check status
uorb_uart_proxy status

# Stop
uorb_uart_proxy stop
```

### System Monitoring
```bash
# Monitor topics on main board
uorb top wheel_loader_setpoint
uorb top wheel_loader_status_front
uorb top wheel_loader_status_rear

# Monitor topics on NXT boards
uorb top wheel_loader_setpoint
uorb top actuator_outputs
uorb top wheel_loader_status
```

## Protocol Details

### Frame Format
```
| Sync (2B) | MsgID (1B) | BoardID (1B) | Length (2B) | Seq (2B) | Timestamp (4B) | Payload (0-256B) | CRC (2B) |
```

### Message Flow
1. **Command Distribution**: X7+ publishes setpoints → Bridge forwards to both NXT boards
2. **Status Collection**: NXT boards publish status → Proxy forwards to X7+ bridge
3. **Heartbeat Monitoring**: All modules exchange periodic heartbeats
4. **Error Handling**: CRC validation, timeout detection, automatic reconnection

## Troubleshooting

### Common Issues

1. **No Communication**
   - Check UART device paths and permissions
   - Verify baudrate settings match on both ends
   - Check cable connections

2. **High Error Rates**
   - Reduce baudrate
   - Check for electrical interference
   - Verify CRC calculation consistency

3. **Board Not Detected**
   - Check heartbeat intervals and timeouts
   - Verify board ID configuration
   - Monitor statistics for dropped messages

### Debug Commands
```bash
# Check UART bridge statistics
uorb_uart_bridge status

# Monitor proxy status
uorb_uart_proxy status

# Low-level UART debugging
cat /dev/ttyS2 | hexdump -C
```

## Development

### Adding New Topics
1. Update `topic_registry.hpp` with new topic mapping
2. Add message handling in bridge/proxy modules
3. Update protocol message IDs if needed

### Testing
Use the provided test configuration in `distributed_uorb_test.conf` for validation.

## Files Structure
```
src/lib/distributed_uorb/
├── uart_protocol/          # Protocol definitions
├── uart_transport/         # UART communication layer
└── topic_registry/         # Topic management

src/modules/
├── uorb_uart_bridge/       # Main board bridge module
└── uorb_uart_proxy/        # NXT board proxy module

msg/
├── WheelLoaderSetpoint.msg # Command messages
└── WheelLoaderStatus.msg   # Status messages
```
