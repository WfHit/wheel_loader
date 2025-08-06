# Bucket Control Module Refactoring

This document explains the refactoring of the bucket control module into separate files for better code organization.

## File Structure

### Core Files

#### `bucket_control.hpp`
- Class definition and member variables
- Function declarations for all functionality
- Module parameters and enums

#### `bucket_control.cpp`
- Main implementation with core functionality:
  - Constructor and initialization
  - Main Run() loop
  - State machine and motion control
  - Hardware interface (sensors, actuators)
  - AHRS integration and advanced control modes
  - Kinematic calculations
  - Status publishing

### System Interface Files

#### `bucket_control_main.cpp`
- Module system interface and command handling:
  - `task_spawn()` - Module startup
  - `custom_command()` - CLI command processing
  - `print_usage()` - Help text
  - `bucket_control_main()` - Entry point
  - Test commands (mode, angle, status)

### Calibration Files

#### `bucket_calibration.cpp`
- AS5600 magnetic encoder calibration routines:
  - `startAutoCalibration()` - Initialize calibration process
  - `updateCalibration()` - State machine for calibration steps
  - `completeCalibration()` - Finalize and validate calibration
  - `abortCalibration()` - Error handling and cleanup
  - `getAS5600Angle()` - Hardware interface (placeholder)
  - Angle translation functions
  - Calibration state management

## Benefits of Refactoring

1. **Better Code Organization**: Related functionality is grouped together
2. **Separation of Concerns**:
   - Core control logic in main file
   - System interface separate from business logic
   - Calibration isolated for specialized functionality
3. **Maintainability**: Easier to find and modify specific features
4. **Modularity**: Each file has a clear purpose and responsibility
5. **Testing**: Individual components can be tested separately

## Building

The CMakeLists.txt has been updated to include all three source files:
```cmake
SRCS
    bucket_control.cpp         # Core implementation
    bucket_calibration.cpp     # Calibration routines
    bucket_control_main.cpp    # System interface
```

## Usage

The module interface remains the same:
```bash
bucket_control start
bucket_control test status
bucket_control test mode 1
bucket_control test angle 45
bucket_control calibrate
bucket_control stop
```

## Code Dependencies

- `bucket_control_main.cpp` depends on `bucket_control.hpp` and calls methods from the main class
- `bucket_calibration.cpp` implements methods declared in `bucket_control.hpp`
- All files share the same class instance and member variables
- Public member variables are accessible for the CLI interface in `bucket_control_main.cpp`

## Future Enhancements

This structure makes it easy to add:
- Additional calibration methods in `bucket_calibration.cpp`
- New CLI commands in `bucket_control_main.cpp`
- Core control features in `bucket_control.cpp`
- Each without affecting the others
