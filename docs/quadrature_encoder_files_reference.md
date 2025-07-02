# Quadrature Encoder Files Reference

## Architecture Files Created/Modified

### Board-Specific Configuration Files

#### NXT-Dual-WL-Rear Board
- **`boards/hkust/nxt-dual-wl-rear/src/board_qencoder.h`**
  - Board-specific quadrature encoder interface
  - Defines `BOARD_NUM_QENCODERS = 1`
  - Function declarations for board encoder operations

- **`boards/hkust/nxt-dual-wl-rear/src/board_qencoder.cpp`**
  - Contains `g_motor_encoders[]` configuration array
  - Single encoder configuration for motor feedback
  - GPIO pins: `QENCODER_A_GPIO_RAW`, `QENCODER_B_GPIO_RAW`
  - Simple wrapper: `board_qencoder_initialize()`

#### NXT-Dual-WL-Front Board
- **`boards/hkust/nxt-dual-wl-front/src/board_qencoder.h`**
  - Board-specific quadrature encoder interface
  - Defines `BOARD_NUM_QENCODERS = 2`
  - Function declarations for board encoder operations

- **`boards/hkust/nxt-dual-wl-front/src/board_qencoder.cpp`**
  - Contains `g_motor_encoders[]` configuration array
  - Two encoder configurations for dual motor setup
  - GPIO pins: `QENCODER1_A_GPIO_RAW`, `QENCODER1_B_GPIO_RAW`, etc.
  - Simple wrapper: `board_qencoder_initialize()`

### Platform Interface Layer

- **`platforms/nuttx/src/px4/stm/stm32_common/include/px4_arch/qencoder.h`**
  - Platform-level API definitions
  - Key functions:
    - `px4_arch_qencoder_initialize(configs, count)`
    - `px4_arch_qencoder_uninitialize_all()`
  - Used by board files to initialize encoders

### Platform Implementation Layer

- **`platforms/nuttx/src/px4/stm/stm32_common/qencoder/qencoder.c`**
  - Platform implementation of encoder operations
  - GPIO configuration for index signals
  - Device path generation (`/dev/qe0`, `/dev/qe1`)
  - Logging and error handling
  - Calls low-level driver functions

### Architecture-Specific Headers

- **`platforms/nuttx/src/px4/stm/stm32h7/include/px4_arch/qencoder.h`**
  - STM32H7-specific header
  - Includes common platform interface
  - Architecture abstraction layer

### Build Configuration

- **`platforms/nuttx/src/px4/stm/stm32_common/qencoder/CMakeLists.txt`**
  - Updated to include both:
    - `nuttx_qencoder.c` (low-level driver)
    - `qencoder.c` (platform implementation)

### Existing Low-Level Driver (Not Modified)

- **`platforms/nuttx/src/px4/stm/stm32_common/qencoder/nuttx_qencoder.c`**
  - Hardware abstraction layer
  - NuttX device driver interface
  - GPIO and timer-based encoder implementations

- **`platforms/nuttx/src/px4/stm/stm32_common/include/px4_arch/nuttx_qencoder.h`**
  - Low-level driver interface
  - Data structures and function prototypes
  - Hardware configuration structures

## Key Changes Made

### 1. Board Files Simplified
- **Before**: Complex initialization logic, GPIO config, logging, device management
- **After**: Only board-specific configuration arrays and simple wrappers

### 2. Platform Layer Added
- **New**: `qencoder.h` - Platform interface
- **New**: `qencoder.c` - Platform implementation
- **New**: Architecture-specific headers

### 3. Clear Separation of Concerns
- **Board Layer**: Hardware pin assignments and encoder specifications
- **Platform Layer**: Common initialization logic and device management
- **Driver Layer**: Hardware register manipulation and NuttX interface

## Usage Pattern

```cpp
// In board init code:
#include "board_qencoder.h"

int board_init(void) {
    // Initialize board-specific encoders
    ret = board_qencoder_initialize();
    return ret;
}
```

## Configuration Pattern

```cpp
// In board_qencoder.cpp:
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
```

## Benefits Achieved

1. **Reduced Code Duplication**: Common logic moved to platform layer
2. **Cleaner Board Files**: Only configuration data remains
3. **Consistent Interface**: All boards use same platform API
4. **Maintainable**: Changes to platform don't affect board files
5. **Extensible**: Easy to add new boards or encoder types
