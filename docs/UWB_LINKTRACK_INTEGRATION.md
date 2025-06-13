# UWB LinkTrack Indoor Positioning System

This document describes the integration of the Nooploop LinkTrack UWB positioning system with PX4 for robust indoor navigation.

## Overview

The UWB LinkTrack system provides centimeter-level positioning accuracy in indoor environments where GPS is unavailable. The implementation includes:

- **Nooploop LinkTrack driver** for serial communication with UWB modules
- **Enhanced EKF2 fusion** with multipath detection and mitigation
- **Robust estimation** techniques for handling NLOS and multipath conditions
- **Adaptive measurement weighting** based on signal quality

## Hardware Setup

### Anchor Placement
- Minimum 4 anchors required for 3D positioning
- Place anchors at different heights (2-3 meters recommended)
- Avoid obstacles between anchors and expected vehicle paths
- Ensure good geometric diversity (avoid coplanar arrangements)

### Connection
- Connect LinkTrack tag to UART port (e.g., TELEM2 - /dev/ttyS3)
- Configure baudrate (default: 921600)
- Ensure proper power supply (5V recommended)

## Software Configuration

### 1. Enable UWB Driver

```bash
# Enable UWB driver
param set SENS_UWB_EN 1

# Set UART port (adjust for your setup)
param set SENS_UWB_PORT 3  # for /dev/ttyS3

# Set baudrate
param set SENS_UWB_BAUD 921600

# Set tag ID to track (0 for all tags)
param set SENS_UWB_TAG_ID 0
```

### 2. Configure Anchor Positions

Create anchor configuration file at `/fs/microsd/uwb_anchors.conf`:

```
# Format: ID,Name,X,Y,Z
0,Anchor_NW,0.0,0.0,2.5
1,Anchor_NE,10.0,0.0,2.5
2,Anchor_SE,10.0,8.0,2.5
3,Anchor_SW,0.0,8.0,2.5
```

### 3. Enable EKF2 UWB Fusion

```bash
# Enable UWB fusion in EKF2
param set EKF2_UWB_EN 1

# Configure measurement noise (adjust based on environment)
param set EKF2_UWB_NOISE 0.1

# Set innovation gate
param set EKF2_UWB_GATE 5.0

# Configure multipath detection thresholds
param set EKF2_UWB_NLOS_THR -85.0
param set EKF2_UWB_LOS_THR 70
```

### 4. Start the System

```bash
# Start UWB driver
nooploop_linktrack start -d /dev/ttyS3 -a /fs/microsd/uwb_anchors.conf

# Check status
nooploop_linktrack status

# Monitor UWB data
listener sensor_uwb
```

## Advanced Configuration

### Multipath Mitigation Parameters

```bash
# RSSI threshold for NLOS detection
param set EKF2_UWB_NLOS_THR -85.0

# LOS confidence threshold (0-100)
param set EKF2_UWB_LOS_THR 70

# Power ratio threshold for multipath detection
param set EKF2_UWB_PWR_THR 0.6

# Maximum range rate for outlier detection
param set EKF2_UWB_MAX_RR 20.0

# Huber loss threshold for robust estimation
param set EKF2_UWB_HUBER 2.0
```

### Vehicle Constraints

```bash
# Maximum velocity constraint
param set EKF2_UWB_MAX_VEL 50.0

# Maximum acceleration constraint
param set EKF2_UWB_MAX_ACC 20.0

# Minimum RSSI for measurement acceptance
param set EKF2_UWB_MIN_RSSI -90.0
```

## Troubleshooting

### No UWB Data
1. Check UART connection and baudrate
2. Verify LinkTrack tag is powered and configured
3. Check parameter `SENS_UWB_EN` is set to 1

### Poor Positioning Accuracy
1. Verify anchor positions are correct
2. Check for multipath conditions (metal objects, walls)
3. Increase measurement noise: `param set EKF2_UWB_NOISE 0.2`
4. Monitor RSSI and LOS confidence values

### High Rejection Rate
1. Lower innovation gate: `param set EKF2_UWB_GATE 3.0`
2. Adjust RSSI threshold: `param set EKF2_UWB_MIN_RSSI -95.0`
3. Check anchor geometry (GDOP)

### Frequent NLOS Detection
1. Relocate anchors to improve line-of-sight
2. Adjust NLOS threshold: `param set EKF2_UWB_NLOS_THR -90.0`
3. Increase measurement noise for NLOS conditions

## Monitoring and Diagnostics

### Real-time Monitoring
```bash
# Monitor UWB measurements
listener sensor_uwb

# Monitor EKF2 innovations
listener estimator_innovations

# Monitor position estimates
listener vehicle_local_position

# Check EKF2 status
listener estimator_status
```

### Performance Analysis
```bash
# UWB driver statistics
nooploop_linktrack status

# EKF2 performance
listener ekf2_timestamps

# System performance
top
perf
```

## Best Practices

### Anchor Deployment
- Use at least 4 anchors for 3D positioning
- Place anchors at different heights
- Avoid symmetrical arrangements
- Ensure good coverage of operating area
- Consider redundant anchors for reliability

### Environment Considerations
- Minimize reflective surfaces near anchors
- Account for moving objects (people, vehicles)
- Consider temperature effects on anchor positions
- Plan for anchor calibration procedures

### System Integration
- Always test in actual operating environment
- Implement fallback positioning methods
- Monitor system health continuously
- Plan for anchor failure scenarios

## API Reference

### uORB Topics

**sensor_uwb** - UWB measurement data
- `range` - Distance measurement in meters
- `rssi` - Signal strength in dBm
- `los_confidence` - Line-of-sight confidence (0-100)
- `anchor_x/y/z` - Anchor position
- `multipath_count` - Detected multipath signals
- `range_bias` - Estimated range bias

### Parameters

See parameter documentation for complete list of configurable options:
- `SENS_UWB_*` - Driver parameters
- `EKF2_UWB_*` - Fusion parameters
