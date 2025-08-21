# Redesigned Wheel Loader Control Architecture

This directory contains the redesigned wheel loader robot module and the new chassis control module that implements advanced control algorithms with battery power management.

## Architecture Overview

The new architecture separates concerns into two main modules:

### 1. Wheel Loader Robot Module (Redesigned)
- **Location**: `src/modules/wheel_loader_robot/wheel_loader_robot.hpp/cpp`
- **Purpose**: High-level coordinator with integrated power management
- **Features**:
  - Command arbitration between manual and autonomous control
  - Battery-aware power management and optimization
  - Subsystem coordination (chassis, boom, bucket)
  - Safety monitoring and emergency response
  - Power mode management (Normal, ECO, Boost, Critical)

### 2. Chassis Control Module (New)
- **Location**: `src/modules/chassis_control/`
- **Purpose**: Advanced mobility control with MPC and slip-aware traction
- **Features**:
  - Model Predictive Control (MPC) for trajectory tracking
  - Slip estimation using existing `articulated_chassis/slip_estimator`
  - Load-aware torque distribution using `articulated_chassis/load_aware_torque`
  - Anti-slip regulation (ASR) and electronic stability control (ESP)
  - Power-aware torque distribution
  - Articulated steering coordination

## Key Improvements

### Power Management
- **Battery State Monitoring**: Real-time battery voltage, current, and capacity tracking
- **Power Budget Calculation**: Dynamic power allocation based on battery state
- **Efficiency Optimization**: Adaptive control based on battery voltage and load
- **Power Modes**:
  - `NORMAL`: Standard operation
  - `ECO`: Reduced power consumption for extended runtime
  - `BOOST`: High performance when battery is full
  - `CRITICAL`: Minimum operation to preserve battery

### Advanced Chassis Control
- **MPC Controller**: Predictive control for optimal trajectory tracking
- **Slip Estimation**: Real-time wheel slip detection and friction estimation
- **Traction Control**:
  - Anti-slip regulation to prevent wheel spinning
  - Electronic stability control for safe cornering
  - Load-aware torque distribution for optimal traction
- **Articulated Steering**: Coordinated control for articulated chassis vehicles

### Safety Features
- **Emergency Stop**: Immediate halt of all operations
- **Health Monitoring**: Continuous subsystem health assessment
- **Fault Detection**: Automatic fault detection and safe mode activation
- **Command Validation**: Input sanitization and limit enforcement

## Message Definitions

### New uORB Messages
- `chassis_command.msg`: Commands for chassis control module
- `chassis_status.msg`: Status feedback from chassis control

### Enhanced/Existing Messages
- Enhanced `wheel_loader_status.msg` with power information
- `vla_command.msg`: VLA bucket end effector pose and position commands (existing)
- Battery and power monitoring integration

## VLA Integration

The VLA (Vision-Language-Action) system provides bucket end effector control:
- **Input**: Natural language commands and visual perception
- **Output**: Target bucket pose and position in world coordinates via `vla_command.msg`
- **Processing**: Inverse kinematics converts bucket targets to vehicle motion and joint commands
- **Safety**: Includes confidence scoring and emergency stop capabilities

## Usage

### Starting the Redesigned System
```bash
# Start the redesigned wheel loader robot coordinator
wheel_loader_robot start

# Start the advanced chassis control
chassis_control start
```

### Configuration Parameters

#### Wheel Loader Robot Parameters
- `WLR_MAX_PWR`: Maximum power consumption limit
- `WLR_ECO_PWR`: Power limit in ECO mode
- `WLR_CRIT_BAT`: Critical battery threshold
- `WLR_LOW_BAT`: Low battery threshold
- `WLR_PWR_OPT`: Enable power optimization

#### Chassis Control Parameters
- `CC_MPC_EN`: Enable Model Predictive Control
- `CC_SLIP_EN`: Enable slip estimation and control
- `CC_ASR_EN`: Enable Anti-Slip Regulation
- `CC_ESP_EN`: Enable Electronic Stability Program
- `CC_MASS`: Vehicle mass for dynamics calculations
- `CC_WHLBASE`: Wheelbase for kinematic calculations

## Integration with Existing Modules

The new architecture leverages existing components:

### Articulated Chassis Components
- **slip_estimator**: Used for real-time slip detection
- **load_aware_torque**: Used for optimal torque distribution
- **steering_controller**: Integrated for articulated steering
- **wheel_controller**: Enhanced with MPC control

### Boom and Bucket Control
- Existing boom and bucket control modules remain unchanged
- Enhanced with power-aware command limiting
- Integrated safety shutdown capabilities

## Development Notes

### Coding Style Compliance
- Follows PX4 coding standards with snake_case naming
- Proper parameter naming conventions
- Consistent module structure and documentation

### Performance Considerations
- 100Hz main control loop for chassis control
- 20Hz MPC updates for computational efficiency
- 50Hz slip estimation updates for safety
- Optimized power calculations at 10Hz

### Future Enhancements
- Machine learning-based terrain adaptation
- Advanced battery degradation modeling
- Predictive maintenance integration
- Enhanced VLA (Visual Language Action) integration

## Testing and Validation

### Unit Tests
- Power management logic validation
- MPC controller stability tests
- Slip estimation accuracy tests
- Safety system response tests

### Integration Tests
- Full system coordination tests
- Power limit compliance validation
- Emergency response verification
- Multi-subsystem coordination tests
