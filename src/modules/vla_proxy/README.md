# VLA Proxy Module

The VLA (Vision-Language-Action) Proxy module acts as a bridge between PX4 and a VLA model, enabling autonomous control through natural language commands and visual understanding.

## Features

- **Bidirectional Communication**: Sends robot status to VLA and receives trajectory commands
- **Real-time Status Updates**: Position, velocity, attitude, and system status
- **Trajectory Control**: Receives and executes complex trajectory commands from VLA
- **Command Interface**: Start/stop VLA control via MAVLink commands
- **Safety Features**: Trajectory timeout detection and error handling

## Architecture

```
┌─────────────────┐    UART Serial   ┌─────────────────┐
│   VLA Model     │ ◄────────────► │   VLA Proxy     │
│                 │   (RS232/TTL)    │    Module       │
└─────────────────┘                 └─────────────────┘
                                             │
                                             │ uORB
                                             ▼
                                    ┌─────────────────┐
                                    │   PX4 System    │
                                    │  (Navigator,    │
                                    │   Controller)   │
                                    └─────────────────┘
```

## Configuration Parameters

Parameters are now defined in `module.yaml` following modern PX4 conventions.

> **Note**: Parameter definitions have been migrated from `params.c` to `module.yaml` for better maintainability and modern PX4 compliance.

| Parameter | Description | Default | Range |
|-----------|-------------|---------|-------|
| `VLA_PROXY_EN` | Enable VLA proxy module | 0 | 0/1 |
| `VLA_UART_EN` | Enable UART communication | 1 | 0/1 |
| `VLA_UART_BAUD` | UART baud rate | 115200 | 9600-921600 |
| `VLA_UART_DEV` | UART device selection | 0 | 0-10 |
| `VLA_UPDATE_RATE` | Status update rate (Hz) | 20.0 | 1.0-100.0 |
| `VLA_TIMEOUT_MS` | Trajectory timeout (ms) | 1000 | 100-10000 |

## Usage

### 1. Enable the Module

Set the parameters via QGroundControl or command line:
```bash
param set VLA_PROXY_EN 1
param set VLA_UART_EN 1
param set VLA_UART_BAUD 115200
param set VLA_UPDATE_RATE 20.0
param set VLA_TIMEOUT_MS 1000
```

### 2. Start the Module

```bash
vla_proxy start
```

### 3. Control Commands

Start VLA control:
```bash
# Via MAVLink command (MAV_CMD_USER_1 with param1=1)
commander takeoff
# Send custom command to start VLA
```

Stop VLA control:
```bash
# Via MAVLink command (MAV_CMD_USER_1 with param1=0)
```

### 4. Check Status

```bash
vla_proxy status
```

## Data Structures

### Robot Status (Sent to VLA)

```cpp
struct RobotStatus {
    uint64_t timestamp;         // Microseconds since boot
    float position[3];          // x, y, z (m)
    float velocity[3];          // vx, vy, vz (m/s)
    float quaternion[4];        // w, x, y, z
    float angular_vel[3];       // roll, pitch, yaw rates (rad/s)
    uint8_t armed;              // 0=disarmed, 1=armed
    uint8_t flight_mode;        // Navigation state
    float battery_percentage;   // Battery remaining (0.0-1.0)
};
```

### VLA Waypoint (Received from VLA)

```cpp
struct VLAWaypoint {
    float position[3];          // x, y, z (m)
    float velocity[3];          // vx, vy, vz (m/s)
    float acceleration[3];      // ax, ay, az (m/s²)
    float yaw;                  // Yaw angle (rad)
    float yaw_rate;             // Yaw rate (rad/s)
    uint64_t timestamp_us;      // Execution time (microseconds)
};
```

## Testing

## UART Communication Protocol

The VLA proxy uses a simple framed protocol over UART:

### Frame Format
```
[HEADER] [TYPE] [LENGTH] [DATA] [CHECKSUM]
   1B      1B      1B     N B      1B
```

- **HEADER**: Always 0xAA
- **TYPE**: Frame type (0x01=Status, 0x02=Trajectory)
- **LENGTH**: Data payload length
- **DATA**: Payload data (varies by type)
- **CHECKSUM**: XOR checksum of DATA bytes

### Status Frame (PX4 → VLA)
- **TYPE**: 0x01
- **DATA**: RobotStatus structure (66 bytes)

### Trajectory Frame (VLA → PX4)
- **TYPE**: 0x02
- **DATA**: VLAWaypoint structure (52 bytes)

## Testing

Use the provided UART test client to simulate VLA communication:

```bash
# Install required Python packages
pip install pyserial numpy

# Monitor robot status only
python3 vla_uart_test_client.py --port /dev/ttyUSB0 --trajectory monitor

# Send circular trajectory
python3 vla_uart_test_client.py --port /dev/ttyUSB0 --trajectory circle --duration 30

# Send figure-8 trajectory
python3 vla_uart_test_client.py --port /dev/ttyUSB0 --trajectory figure8 --duration 45

# Use different UART settings
python3 vla_uart_test_client.py --port /dev/ttyS3 --baudrate 57600
```

## Integration with VLA Models

### Protocol Implementation

The module uses a simple framed binary protocol over UART. You can adapt the communication layer for your specific VLA implementation:

1. **Modify Communication**: Replace UART with your preferred protocol (CAN, SPI, I2C, etc.)
2. **Update Serialization**: Implement JSON, Protocol Buffers, or custom serialization
3. **Add Authentication**: Implement security features as needed

### Example VLA Integration

```python
import serial
import struct
import time

class VLAModel:
    def __init__(self, uart_port="/dev/ttyUSB0", baudrate=115200):
        self.ser = serial.Serial(uart_port, baudrate, timeout=0.1)

    def get_robot_status(self):
        # Look for status frame (0xAA, 0x01)
        while True:
            if self.ser.read(1) == b'\xAA':
                frame_info = self.ser.read(2)
                if len(frame_info) == 2 and frame_info[0] == 0x01:
                    data_len = frame_info[1]
                    data = self.ser.read(data_len + 1)  # +1 for checksum
                    if len(data) == data_len + 1:
                        return struct.unpack('<Q3f3f4f3fBBf', data[:-1])

    def send_trajectory(self, waypoints):
        for wp in waypoints:
            # Create trajectory frame
            data = struct.pack('<3f3f3fffQ', *wp)
            checksum = 0
            for byte in data:
                checksum ^= byte
            frame = b'\xAA\x02' + len(data).to_bytes(1, 'little') + data + checksum.to_bytes(1, 'little')
            self.ser.write(frame)
```## Safety Considerations

- **Timeout Protection**: Module automatically stops if no trajectory is received within timeout period
- **Bounds Checking**: Validate trajectory waypoints before execution
- **Fallback Mode**: Ensure manual override capability
- **Connection Monitoring**: Automatic reconnection on communication failure

## Troubleshooting

### Common Issues

1. **Connection Failed**: Check UART device permissions and cable connections
2. **No Status Updates**: Verify VLA_PROXY_EN and VLA_UART_EN parameters are set to 1
3. **Trajectory Timeout**: Increase VLA_TIMEOUT_MS or check VLA model response time
4. **Frame Errors**: Check UART settings (baud rate, parity, stop bits) match on both ends
5. **Permission Denied**: Add user to dialout group: `sudo usermod -a -G dialout $USER`

### Debug Information

Check module status and logs:
```bash
vla_proxy status
dmesg | grep vla_proxy

# Check UART device
ls -la /dev/ttyUSB* /dev/ttyS*

# Test UART connectivity
stty -F /dev/ttyUSB0 115200
echo "test" > /dev/ttyUSB0
```

## Future Enhancements

- Support for multiple VLA models
- Advanced trajectory interpolation
- Real-time obstacle avoidance integration
- Mission planning interface
- ROS2 bridge implementation
