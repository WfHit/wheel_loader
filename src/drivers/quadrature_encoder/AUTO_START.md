# Quadrature Encoder Auto-Start Configuration

## Overview

The quadrature encoder driver now supports automatic startup during system boot through the `QE_AUTO_START` parameter. This eliminates the need for manual driver startup and provides a more robust initialization process.

## Configuration

### Parameter: QE_AUTO_START

- **Type**: Integer (bitmask)
- **Range**: 0-15
- **Default**: 0 (disabled)
- **Description**: Controls which quadrature encoder instances are automatically started during boot

### Bitmask Values

The `QE_AUTO_START` parameter uses a bitmask to specify which instances to start:

- **Bit 0 (value 1)**: Enable auto-start for instance 0
- **Bit 1 (value 2)**: Enable auto-start for instance 1
- **Bit 2 (value 4)**: Enable auto-start for instance 2
- **Bit 3 (value 8)**: Enable auto-start for instance 3

### Common Configurations

| Value | Instances Started | Description |
|-------|------------------|-------------|
| 0     | None            | Auto-start disabled |
| 1     | 0               | Instance 0 only |
| 2     | 1               | Instance 1 only |
| 3     | 0, 1            | Instances 0 and 1 |
| 4     | 2               | Instance 2 only |
| 15    | 0, 1, 2, 3      | All instances |

## Board-Specific Defaults

### Front Wheel Loader Board (nxt-dual-wl-front)
- **Default**: `QE_AUTO_START = 3` (instances 0 and 1)
- **Instance 0**: Front wheel encoder
- **Instance 1**: Bucket position encoder

### Rear Wheel Loader Board (nxt-dual-wl-rear)
- **Default**: `QE_AUTO_START = 1` (instance 0 only)
- **Instance 0**: Rear wheel encoder

## Manual Configuration

To change the auto-start configuration:

```bash
# Enable auto-start for instances 0 and 1
param set QE_AUTO_START 3

# Enable auto-start for instance 0 only
param set QE_AUTO_START 1

# Disable auto-start
param set QE_AUTO_START 0

# Save parameters
param save
```

## Legacy Support

The system maintains backward compatibility with the legacy `QUADENC_ENABLE` parameter. If `QE_AUTO_START` is set to 0 but `QUADENC_ENABLE` is set to 1, the system will fall back to legacy mode (though no instances will be automatically started).

## Manual Driver Control

Even with auto-start enabled, you can still manually control the drivers:

```bash
# Start a specific instance
quadrature_encoder start -i 0

# Stop a specific instance
quadrature_encoder stop -i 0

# Check status
quadrature_encoder status

# Reset encoder position
quadrature_encoder reset -i 0
```

## Troubleshooting

1. **Auto-start not working**: Check that `QE_AUTO_START` is set to a non-zero value
2. **Wrong instances starting**: Verify the bitmask calculation for your desired configuration
3. **Driver conflicts**: Ensure no manual starts conflict with auto-start instances

## Implementation Details

The auto-start functionality is implemented in the board-specific sensor initialization scripts:
- `/boards/hkust/nxt-dual-wl-front/init/rc.board_sensors`
- `/boards/hkust/nxt-dual-wl-rear/init/rc.board_sensors`

These scripts check the `QE_AUTO_START` parameter during boot and start the appropriate encoder instances automatically.
