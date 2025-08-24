# Holybro Pixhawk 6X Pro UWB

This is a specialized board configuration based on the PX4 FMU-v6x hardware, specifically configured for UWB (Ultra-Wideband) positioning applications.

## Key Features

- **Base Hardware**: STM32H753IIK6 processor (same as FMU-v6x)
- **UWB Integration**: TELEM1 port dedicated to UWB communication
- **NoopLoop LinkTrack Support**: Built-in driver for LinkTrack UWB modules
- **Enhanced EKF2**: Optimized for UWB fusion and multipath mitigation

## UWB Configuration

### Hardware Setup
- **UWB Port**: TELEM1 (/dev/ttyS6) is dedicated for UWB communication
- **Baud Rate**: 921600 bps (configurable)
- **Protocol**: NoopLoop LinkTrack compatible

### Default Parameters
- `UWB_EN`: 1 (UWB enabled)
- `UWB_PORT`: 101 (TELEM1)
- `UWB_BAUD`: 921600
- `EKF2_UWB_EN`: 1 (UWB fusion enabled)

### Usage
The UWB driver starts automatically on boot. To manually control:

```bash
# Start UWB driver
nooploop_linktrack start -d /dev/ttyS6

# Stop UWB driver
nooploop_linktrack stop

# Check status
nooploop_linktrack status

# Monitor UWB data
listener sensor_uwb
```

### Anchor Configuration
Create an anchor configuration file at `/fs/microsd/uwb_anchors.conf`:

```
# Format: ID,Name,X,Y,Z
0,Anchor_NW,0.0,0.0,2.5
1,Anchor_NE,10.0,0.0,2.5
2,Anchor_SE,10.0,8.0,2.5
3,Anchor_SW,0.0,8.0,2.5
```

## Hardware Differences from Standard FMU-v6x

- TELEM1 port is pre-configured for UWB instead of MAVLink telemetry
- UWB drivers are enabled by default
- EKF2 UWB fusion is pre-configured

## Compatible UWB Hardware

- NoopLoop LinkTrack series
- NXP UWB SR150 (with uwb_sr150 driver)
- Other LinkTrack-compatible UWB modules

## Build Instructions

```bash
make holybro_pixhawk-6x-pro-uwb_default
```

## Support

For UWB-specific issues, refer to:
- `/docs/UWB_LINKTRACK_INTEGRATION.md`
- UWB parameter documentation (`UWB_*` and `EKF2_UWB_*`)
