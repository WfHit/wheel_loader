# PX4 Wheel Loader Coding Style Guide

## File Naming Conventions

### Source Files
- **C++ implementation files**: `module_name_function.cpp`
  - Example: `load_aware_torque_distribution.cpp`
  - Example: `load_aware_torque_main.cpp`
- **C++ header files**: `module_name_function.hpp`
  - Example: `load_aware_torque_distribution.hpp`
- **C parameter files**: `module_name_params.c`
  - Example: `src/modules/articulated_chassis/load_aware_torque/load_aware_torque_params.c`
  - **Note**: Parameter files are NOT included in CMakeLists.txt SRCS
- **All lowercase with underscores** (snake_case)

### Build Files
- `CMakeLists.txt` - Standard CMake naming
- `Kconfig` - Module configuration for menuconfig
- **DO NOT** create `module.yaml`

## uORB Message Guidelines

### Check Existing Messages First
Before creating a new uORB message, always check if an existing message can be used:
```bash
# Search for existing messages
find /workspaces/wheel_loader/msg -name "*.msg" | grep -i torque
find /workspaces/wheel_loader/msg -name "*.msg" | grep -i load
```

### Review Message Content
When creating or modifying uORB messages:
1. **Check field necessity** - Don't add fields "just in case"
2. **Use appropriate data types** - uint8 for flags, float for continuous values
3. **Include units in comments**
4. **Add timestamp if needed**
5. **Group related fields together**
6. **Check for similar existing messages** - If similar messages exist, ask for confirmation on which to use
7. **After changing message content** - Update ALL modules using this message

### Example Message Review
```msg
# BAD - Redundant fields, unclear units
uint64 timestamp
float32 torque_front
float32 torque_rear
float32 front_torque_percentage  # Redundant with torque values
float32 rear_torque_percentage   # Redundant with torque values
int32 mode                       # Should be uint8
float32 speed                    # Missing units

# GOOD - Clear, minimal, well-documented
uint64 timestamp                 # time since system start (microseconds)
float32 front_torque_ratio       # Front axle torque ratio [0.0-1.0]
float32 total_torque_nm          # Total requested torque (Nm)
uint8 distribution_mode          # 0=normal, 1=efficiency, 2=traction
float32 vehicle_speed_mps        # Vehicle speed (m/s)
bool is_slipping                 # Traction control active flag
```

## Class and Structure Naming

### Classes
- **CamelCase** with descriptive names
- Example: `LoadAwareTorqueDistribution`
- Inherit from appropriate PX4 base classes:
  ```cpp
  class LoadAwareTorqueDistribution : public ModuleBase<LoadAwareTorqueDistribution>,
                                      public ModuleParams,
                                      public px4::ScheduledWorkItem
  ```

### Structures
- **snake_case** with `_s` suffix for uORB messages
- Example: `load_aware_torque_s`
- Example: `module_status_s`

## Variable Naming

### Member Variables
- **Private members**: underscore prefix
  ```cpp
  float _min_front_ratio;
  LoadState _load_state;
  uORB::Subscription _wheel_speeds_sub;
  ```

### Constants
- **Static constexpr**: ALL_CAPS with underscores
  ```cpp
  static constexpr float CONTROL_RATE_HZ = 50.0f;
  static constexpr float MAX_PAYLOAD_KG = 8000.0f;
  ```

### Local Variables
- **snake_case** for all local variables
  ```cpp
  float front_weight_ratio = 0.5f;
  bool is_valid = true;
  ```

## Parameter Naming

### Parameter Name Requirements
- **Maximum 16 characters** (including prefix)
- **Format**: `PREFIX_NAME`
- **Prefix**: 2-4 letter module identifier

### Parameter Types
- **Float parameters**: `PARAM_DEFINE_FLOAT(NAME, value)`
- **Integer parameters**: `PARAM_DEFINE_INT32(NAME, value)` (Note: Use `ParamInt` in class definitions, not `ParamInt32`)
- **Boolean parameters**: Use `PARAM_DEFINE_INT32` with 0/1 values

### Examples
```c
// GOOD - 16 characters or less
PARAM_DEFINE_FLOAT(LAT_MIN_FRONT, 0.2f);      // 13 chars ✓
PARAM_DEFINE_FLOAT(LAT_RATE_LIM, 1.0f);       // 12 chars ✓
PARAM_DEFINE_INT32(LAT_ADAPT_EN, 1);          // 12 chars ✓

// BAD - Too long (>16 chars)
PARAM_DEFINE_FLOAT(LAT_MIN_FRONT_RATIO, 0.2f);    // 19 chars ✗
PARAM_DEFINE_FLOAT(LAT_EFFICIENCY_WEIGHT, 0.4f);  // 21 chars ✗
PARAM_DEFINE_FLOAT(LAT_TRACTION_WEIGHT, 0.6f);    // 19 chars ✗
```

### Parameter Usage in Class Definitions
```cpp
// Use ParamInt (not ParamInt32) for integer parameters
DEFINE_PARAMETERS(
    (ParamFloat<px4::params::LAT_MIN_FRONT>) _min_front,
    (ParamInt<px4::params::LAT_ADAPT_EN>) _adaptive_enable,  // NOT ParamInt32
    (ParamBool<px4::params::LAT_BOOL_PARAM>) _bool_param
)
```

### Parameter Units
Always specify appropriate units in parameter documentation:
```c
/**
 * Parameter description
 *
 * @unit m      // meters
 * @unit rad    // radians
 * @unit m/s    // meters per second
 * @unit deg    // degrees
 * @unit s      // seconds
 * @unit Hz     // hertz
 * @unit kg     // kilograms
 * @unit N      // newtons
 * @unit Nm     // newton-meters
 * @unit 1/s    // per second (for rates)
 */
```

### Parameter Documentation Template
```c
/**
 * Brief description (keep short)
 */
PARAM_DEFINE_FLOAT(LAT_PARAM, 0.5f);
```

## uORB Topic Naming

### Topic Names
- **Always lowercase with underscores**
- Match the message file name (without .msg)

```cpp
// GOOD - lowercase
uORB::Publication<load_aware_torque_s> _torque_pub{ORB_ID(load_aware_torque)};
uORB::Subscription _wheel_speeds_sub{ORB_ID(wheel_speeds_setpoint)};

// BAD - mixed case
uORB::Publication<LoadAwareTorque_s> _torque_pub{ORB_ID(LoadAwareTorque)};
```

### Before Creating New Topics
1. **Check existing topics** that might serve your purpose:
   ```bash
   grep -r "torque" /workspaces/wheel_loader/msg/
   grep -r "wheel" /workspaces/wheel_loader/msg/
   ```

2. **Consider extending existing messages** rather than creating new ones

3. **Review similar modules** to see what topics they use

## Header File Structure

```cpp
#pragma once  // Always use pragma once

// System includes first
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>

// Library includes
#include <lib/mathlib/mathlib.h>
#include <matrix/matrix.hpp>

// uORB includes (use lowercase topic names)
#include <uORB/topics/load_aware_torque.h>
#include <uORB/topics/wheel_speeds_setpoint.h>

// Using declarations
using namespace time_literals;
using namespace matrix;

/**
 * @brief Module description
 *
 * Detailed description
 */
class ClassName : public BaseClasses
{
public:
	// Public methods

private:
	// Private methods

	// Private members (underscore prefix)
};
```

## Code Organization

### Module Structure
```
module_name/
├── CMakeLists.txt              # Build configuration
├── Kconfig                     # Module configuration
├── module_name_main.cpp        # Entry point
├── module_name_class.hpp       # Main class header
├── module_name_class.cpp       # Main class implementation
└── module_name_params.c        # Parameters (NOT in CMakeLists.txt)
```

### CMakeLists.txt Format
```cmake
px4_add_module(
	MODULE modules__category__module_name
	MAIN module_name
	STACK_MAIN 2048
	SRCS
		module_name_class.cpp
		module_name_main.cpp
		# DO NOT include module_name_params.c here
	DEPENDS
		mathlib
		# other dependencies
)
```

### Kconfig Format
```kconfig
menuconfig ARTICULATED_CHASSIS_LOAD_AWARE_TORQUE
	bool "load aware torque"
	default n
	---help---
		Enable support for load aware torque distribution

if ARTICULATED_CHASSIS_LOAD_AWARE_TORQUE

config ARTICULATED_CHASSIS_LOAD_AWARE_TORQUE_DEBUG
	bool "Enable debug output"
	default n
	---help---
		Enable additional debug output

endif # ARTICULATED_CHASSIS_LOAD_AWARE_TORQUE
```

## Formatting Rules

### Indentation
- **Tabs** for indentation
- **Tab width: 4 spaces** (not 8)
- Configure your editor to display tabs as 4 spaces

### Braces
- Opening brace on same line for functions and classes
- New line for control structures
```cpp
class MyClass {
	void my_function() {
		// code indented with tab (shown as 4 spaces)
	}

	if (condition) {
		// code indented with tab
	}
};
```

### Spacing
- Space after control keywords: `if (`, `for (`, `while (`
- No space after function names: `function_name(`
- Spaces around operators: `a + b`, `x = y`

## uORB Message Best Practices

### Message Design Checklist
- [ ] Check if existing message can be reused
- [ ] All fields have clear purpose and documentation
- [ ] Units are specified in comments
- [ ] Timestamp included if message timing matters
- [ ] No redundant fields
- [ ] Appropriate data types used
- [ ] Related fields grouped together
- [ ] Message name reflects its purpose

### Example Well-Designed Message
```msg
# Load aware torque distribution command
# Distributes torque between front and rear axles based on load conditions

uint64 timestamp                    # time since system start (microseconds)

# Torque distribution
float32 front_torque_ratio         # Front axle torque ratio [0.0-1.0]
float32 requested_total_torque_nm  # Total torque request (Nm)

# Vehicle state
float32 estimated_payload_kg       # Estimated payload mass (kg)
float32 cog_position_x_m          # Center of gravity X position (m)
float32 cog_position_z_m          # Center of gravity Z height (m)

# Control modes
uint8 MODE_NORMAL = 0
uint8 MODE_EFFICIENCY = 1
uint8 MODE_TRACTION = 2
uint8 distribution_mode           # Current distribution mode

# Status flags
bool is_adaptive_enabled          # Adaptive control active
bool is_slipping                  # Traction control intervention
bool is_stable                    # Stability within limits
```

## Common Parameter Abbreviations

To keep parameter names under 16 characters, use these abbreviations:
- `MIN` instead of `MINIMUM`
- `MAX` instead of `MAXIMUM`
- `LIM` instead of `LIMIT`
- `EN` instead of `ENABLE`
- `OPT` instead of `OPTIMIZE`
- `EFF` instead of `EFFICIENCY`
- `TRAC` instead of `TRACTION`
- `STAB` instead of `STABILITY`
- `DYN` instead of `DYNAMIC`
- `TERR` instead of `TERRAIN`
- `OFF` instead of `OFFSET`
- `TC` for `TIME_CONSTANT`

## Quick Reference Card

### Critical Rules:
- File names: snake_case (`load_aware_torque_main.cpp`)
- Class names: CamelCase (`LoadAwareTorqueDistribution`)
- Parameter names: ≤16 chars (`LAT_MIN_FRONT` not `LAT_MINIMUM_FRONT_RATIO`)
- uORB topics: lowercase (`load_aware_torque` not `LoadAwareTorque`)
- Tab width: 4 spaces
- NO `module.yaml`, YES `Kconfig`
- `params.c` NOT in CMakeLists.txt SRCS
- Check existing uORB messages before creating new ones
- Always specify units in parameter documentation

### Pre-Commit Checklist
- [ ] Parameter names ≤16 characters?
- [ ] File names use snake_case?
- [ ] uORB topics are lowercase?
- [ ] Checked for existing uORB messages?
- [ ] Units specified in parameter docs?
- [ ] Kconfig file created?
- [ ] params.c NOT in CMakeLists.txt?
- [ ] Tab indentation (4 spaces)?

This style guide ensures consistency with PX4 standards while adhering to the specific requirements for the wheel loader project.
