# Quadrature Encoder Architecture

## Overview

The PX4 quadrature encoder subsystem follows a layered architecture similar to the IO timer system, with clear separation between board-specific configuration and platform-generic implementation.

## Architecture Layers

### 1. Board-Specific Layer
**Location**: `boards/{vendor}/{board}/src/board_qencoder.{h,cpp}`

**Purpose**: Contains only board-specific quadrature encoder configurations.

**Responsibilities**:
- Define encoder hardware configuration (GPIO pins, resolution, etc.)
- Provide board-specific encoder count
- Expose simple initialization interface

**Example Structure**:
```cpp
// Board-specific encoder configuration array
static const struct nuttx_qe_config_s g_motor_encoders[] = {
  {
    .gpio = {
      .phase_a = QENCODER_A_GPIO_RAW,
      .phase_b = QENCODER_B_GPIO_RAW,
      .index = 0,
    },
    .resolution = 1024,
    .use_index = false,
    .x4_mode = true,
    .invert_dir = false,
  },
};

// Simple initialization wrapper
int board_qencoder_initialize(void) {
  return px4_arch_qencoder_initialize(g_motor_encoders, NUM_ENCODERS);
}
```

### 2. Platform Interface Layer
**Location**: `platforms/nuttx/src/px4/stm/stm32_common/include/px4_arch/qencoder.h`

**Purpose**: Defines the platform-level API for quadrature encoder operations.

**Key Functions**:
- `px4_arch_qencoder_initialize()` - Initialize encoders from board config
- `px4_arch_qencoder_uninitialize_all()` - Cleanup all encoder instances

### 3. Platform Implementation Layer
**Location**: `platforms/nuttx/src/px4/stm/stm32_common/qencoder/qencoder.c`

**Purpose**: Implements platform-specific logic that is common across boards.

**Responsibilities**:
- GPIO configuration for index signals
- Device path generation (`/dev/qe0`, `/dev/qe1`, etc.)
- Logging and error handling
- Iteration over encoder arrays
- Calls to low-level driver functions

### 4. Low-Level Driver Layer
**Location**: `platforms/nuttx/src/px4/stm/stm32_common/qencoder/nuttx_qencoder.c`

**Purpose**: Hardware abstraction layer interfacing with NuttX kernel.

**Responsibilities**:
- Direct hardware register manipulation
- NuttX device driver interface
- Hardware-specific encoder implementations (GPIO, Timer, etc.)

### 5. Architecture-Specific Headers
**Location**: `platforms/nuttx/src/px4/stm/{arch}/include/px4_arch/qencoder.h`

**Purpose**: Include appropriate common headers for specific architectures.

## Data Flow

```
Board Config Array → Platform Interface → Platform Implementation → Low-Level Driver → Hardware
     ↓                     ↓                      ↓                       ↓
g_motor_encoders[]  px4_arch_qencoder_    GPIO config +           nuttx_qencoder_      STM32
                    initialize()          Device creation         initialize()         Hardware
```

## File Structure

```
boards/{vendor}/{board}/src/
├── board_qencoder.h          # Board-specific encoder declarations
└── board_qencoder.cpp        # Board-specific encoder configurations

platforms/nuttx/src/px4/stm/
├── stm32_common/
│   ├── include/px4_arch/
│   │   └── qencoder.h        # Platform interface declarations
│   └── qencoder/
│       ├── CMakeLists.txt    # Build configuration
│       ├── qencoder.c        # Platform implementation
│       └── nuttx_qencoder.c  # Low-level driver
└── {arch}/include/px4_arch/
    └── qencoder.h            # Architecture-specific includes
```

## Benefits of This Architecture

### 1. **Separation of Concerns**
- Board files contain only hardware-specific configuration
- Platform files contain reusable logic
- Clear boundaries between layers

### 2. **Code Reuse**
- Platform implementation shared across all boards
- Consistent behavior across different hardware
- Reduced duplication

### 3. **Maintainability**
- Changes to platform logic don't require board file modifications
- Board-specific changes isolated to board directory
- Clear interfaces between layers

### 4. **Consistency**
- Follows established PX4 patterns (similar to IO timer system)
- Predictable structure for developers
- Standard naming conventions

### 5. **Testability**
- Platform layer can be unit tested independently
- Board configurations can be validated separately
- Clear dependency injection points

## Implementation Guidelines

### For Board Developers

1. **Keep board files minimal** - Only configuration data
2. **Use standardized naming** - Follow GPIO pin naming conventions
3. **Document hardware connections** - Comment encoder wiring details
4. **Define constants in board_config.h** - Centralize pin definitions

### For Platform Developers

1. **Handle errors gracefully** - Provide clear error messages
2. **Log important events** - Use appropriate log levels
3. **Validate inputs** - Check configuration parameters
4. **Follow coding standards** - Match existing PX4 style

## Migration Pattern

When converting existing encoder code to this architecture:

1. **Extract board-specific data** to configuration arrays
2. **Move GPIO configuration** to platform layer
3. **Centralize logging** in platform implementation
4. **Replace direct driver calls** with platform interface
5. **Update build files** to include new platform sources

## Future Extensions

This architecture supports future enhancements:

- **Multiple encoder types** (GPIO, Timer, SPI, I2C)
- **Runtime configuration** changes
- **Encoder calibration** features
- **Performance monitoring** and diagnostics
- **Hot-plug support** for modular systems

## Related Systems

- **IO Timer System** - Similar layered architecture pattern
- **SPI/I2C Drivers** - Hardware abstraction examples
- **Sensor Framework** - Device driver patterns
