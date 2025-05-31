# ST3125 Smart Servo Driver

This driver provides support for SCServo ST3125 smart servo motors in PX4.

## Overview

The ST3125 driver implements a serial communication protocol compatible with SCServo smart servos, providing:

- Position control with feedback
- Velocity and current monitoring
- Temperature monitoring
- Multi-servo synchronous control
- Auto-discovery of connected servos
- Parameter-based configuration

## Hardware Setup

### Wiring

Connect the ST3125 servos in a daisy-chain configuration:

```
PX4 UART ←→ Servo 1 ←→ Servo 2 ←→ ... ←→ Servo N
```

- **VCC**: 6-12V DC power supply
- **GND**: Common ground
- **DATA**: Serial data line (TTL level)

### Servo Configuration

Each servo must have a unique ID (1-253). Use the SCServo configuration software to:

1. Set unique servo IDs
2. Configure baudrate (default: 115200)
3. Set position limits if needed
4. Configure PID parameters

## Usage

### Starting the Driver

```bash
# Start with default settings
st3125_servo start

# Start on specific port with custom baudrate
st3125_servo start -d /dev/ttyS3 -b 115200

# Start with debug output
st3125_servo start -v
```

### Parameter Configuration

Key parameters:

- `ST3125_BAUDRATE`: Serial communication baudrate
- `ST3125_UPDATE_RATE`: Control loop frequency (Hz)
- `ST3125_POS_MIN/MAX`: Position limits for all servos
- `ST3125_TORQUE_EN`: Enable torque on startup
- `ST3125_AUTO_DISC`: Auto-discover connected servos

### Mixer Configuration

Add servo outputs to your mixer file:

```
# ST3125 servo outputs
M: 1
S: 1 0  10000  10000      0 -10000  10000
```

### QGroundControl Integration

The driver publishes servo feedback via uORB topics:
- Position, velocity, current, voltage, temperature
- Error status and diagnostics
- Load and goal position information

## Protocol Details

The driver implements the SCServo protocol:

- **Packet format**: Header + Length + ID + Instruction + Data + Checksum
- **Baudrate**: 9600 - 1M baud
- **Addressing**: 1-253 servo IDs
- **Instructions**: Read, write, sync write, ping, reset

### Communication Features

- Automatic timeout and retry handling
- Checksum verification
- Multi-servo synchronous writes
- Status feedback reading
- Error detection and reporting

## Troubleshooting

### Common Issues

1. **No servos detected**
   - Check wiring and power supply
   - Verify baudrate matches servo configuration
   - Enable debug output: `param set ST3125_DEBUG 4`

2. **Communication errors**
   - Check for loose connections
   - Reduce update rate: `param set ST3125_UPDATE_RATE 50`
   - Verify servo IDs are unique

3. **Position errors**
   - Check position limits: `ST3125_POS_MIN/MAX`
   - Verify mixer configuration
   - Check servo calibration

### Debug Commands

```bash
# Show driver status
st3125_servo status

# Show detailed servo information
st3125_servo info

# Stop the driver
st3125_servo stop
```

### Log Analysis

Enable logging of servo feedback:

```bash
logger start -t -b 200 -e servo_feedback
```

## Performance

- **Update rate**: Up to 400 Hz
- **Latency**: < 5ms typical
- **Servo count**: Up to 12 servos
- **Position resolution**: 12-bit (4096 steps)
- **Velocity resolution**: 10-bit (1024 steps)

## Compatibility

- **Servo models**: ST3125, ST3215 (compatible protocol)
- **PX4 versions**: v1.14+
- **Hardware**: Any PX4 flight controller with UART
- **Power**: 6-12V DC (servo dependent)
