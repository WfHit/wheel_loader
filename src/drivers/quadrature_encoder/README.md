# Quadrature Encoder Driver

A high-performance quadrature encoder driver for PX4 using GPIO interrupts.

## Overview

This driver provides direct hardware access for reading quadrature encoders without requiring hardware timer peripherals. It uses GPIO interrupts to decode quadrature signals, making it suitable for applications where timer resources are limited or when maximum flexibility is needed.

## Key Improvements Over Previous Implementation

1. **Cleaner Architecture**
   - Removed redundant "gpio" naming throughout the codebase
   - Better separation of concerns with dedicated `SignalFilter` class
   - More modular design with clear responsibilities

2. **Enhanced Code Quality**
   - Proper use of C++11 features (atomic operations, constexpr, delete for copy/move)
   - Better error handling and health monitoring
   - Simplified parameter handling
   - Cleaner interrupt handler implementation

3. **Improved Performance**
   - Optimized state machine using lookup table
   - Efficient atomic operations for thread safety
   - Dedicated performance counters per instance
   - Better velocity calculation with low-pass filtering

4. **Better Documentation**
   - Comprehensive Doxygen comments
   - Clear class and method documentation
   - Detailed usage examples in help text

## Features

- **Multi-instance support**: Up to 4 encoders simultaneously
- **Digital filtering**: Configurable noise reduction
- **Real-time velocity calculation**: With low-pass filtering
- **Comprehensive error detection**: Invalid transitions, high error rates
- **Thread-safe operation**: Using atomic operations
- **Flexible configuration**: Per-instance parameters
- **Automatic startup**: Configurable auto-start on boot

## Configuration

### Auto-Start Configuration

The driver supports automatic startup during system boot through the `QE_AUTO_START` parameter:

- `QE_AUTO_START`: Bitmask for auto-starting instances (0-15, default: 0)
  - Bit 0 (value 1): Auto-start instance 0
  - Bit 1 (value 2): Auto-start instance 1
  - Bit 2 (value 4): Auto-start instance 2
  - Bit 3 (value 8): Auto-start instance 3

For detailed auto-start configuration, see [AUTO_START.md](AUTO_START.md).

### Parameters

- `QE_UPDATE_RATE`: Update rate in Hz (1-1000, default: 50)
- `QE_FILTER_EN`: Enable digital filtering (default: true)
- `QE_MAX_ERR_RT`: Maximum error rate threshold in % (0.1-50.0, default: 5.0)
- `QE_PPR_x`: Pulses per revolution for instance x (1-100000, default: 1024)
- `QE_INVERT_x`: Invert direction for instance x (default: false)

### Board Configuration

Board-specific configuration should define:
```cpp
const struct EncoderConfig g_quadrature_encoder_config[] = {
    {
        .gpio_a = GPIO_PIN_A,
        .gpio_b = GPIO_PIN_B,
        .pulses_per_revolution = 1024,
        .invert_direction = false,
        .enable_filtering = true,
        .filter_window = 3
    },
    // Additional instances...
};
const unsigned int g_quadrature_encoder_count = 1;
```

## Usage

### Starting the Driver
```bash
# Start encoder on instance 0
quadrature_encoder start -i 0

# Start multiple instances
quadrature_encoder start -i 0
quadrature_encoder start -i 1
```

### Reset Position
```bash
# Reset encoder position to zero
quadrature_encoder reset -i 0
```

### Check Status
```bash
# Show all running instances
quadrature_encoder status
```

## Implementation Details

### Quadrature Decoding

The driver uses a state machine approach with a lookup table for efficient quadrature decoding:
- Monitors phase A signal via GPIO interrupt
- Reads phase B signal on each interrupt
- Uses 2-bit Gray code state transitions
- Detects invalid transitions for error reporting

### Signal Filtering

Optional digital filtering using majority voting:
- Configurable window size (default: 3 samples)
- Requires stable signal for minimum samples before accepting transition
- Helps reduce noise-induced counting errors

### Thread Safety

All critical data is protected using atomic operations:
- Position counter
- Velocity calculations
- Health status
- Error counters

## Building

The driver is built as part of the PX4 firmware when `DRIVERS_QUADRATURE_ENCODER` is enabled in Kconfig and the board defines `BOARD_HAS_QUADRATURE_ENCODER`.

## Troubleshooting

1. **High Error Rate**
   - Check wiring and connections
   - Enable filtering if not already enabled
   - Verify encoder specifications match configuration
   - Check for electromagnetic interference

2. **No Updates**
   - Verify GPIO pins are correctly configured
   - Check encoder power supply
   - Confirm interrupt is attached (check status output)

3. **Incorrect Velocity**
   - Verify PPR (pulses per revolution) setting
   - Check update rate is appropriate for encoder speed
   - Ensure no mechanical slippage
