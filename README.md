# Wheel Loader Robot Autopilot

[![Build Status](https://img.shields.io/badge/build-passing-brightgreen.svg)]() [![PX4 Base](https://img.shields.io/badge/PX4%20Base-v1.16.0-blue.svg)]()

An independent autonomous wheel loader robot control system based on PX4 v1.16.0, specifically designed for construction and material handling applications.

## Overview

The Wheel Loader Robot project provides a comprehensive autonomous control system for wheel loader vehicles, featuring advanced hydraulic control, articulated chassis management, and intelligent material handling capabilities. This project has diverged significantly from the main PX4 codebase to specialize in ground-based construction equipment automation.

## Key Features

### Hydraulic Control Systems
- **Boom Control**: Precise hydraulic boom positioning and force control
- **Bucket Control**: Advanced bucket manipulation with anti-spill and grading capabilities
- **Load-Aware Operation**: Adaptive control based on payload weight and stability

### Chassis Control
- **Articulated Steering**: Front axle steering with frame articulation support
- **Independent Wheel Control**: Dual-axle wheel control with differential capabilities
- **Traction Management**: Advanced slip detection and torque distribution

### Safety & Autonomy
- **Multi-Level Safety Systems**: Emergency stops, stability monitoring, load limiting
- **Autonomous Task Execution**: Coordinated material handling operations
- **Manual Override**: Seamless transition between autonomous and manual control

## Architecture

### Core Modules

The system is built around specialized control modules:

```
src/modules/
├── boom_control/              # Hydraulic boom control
├── bucket_control/            # Bucket manipulation control  
├── articulated_chassis/       # Chassis and mobility control
│   ├── wheel_controller/      # Individual wheel control
│   ├── steering_controller/   # Articulated steering
│   └── traction_control/      # Slip and traction management
└── (standard PX4 modules)     # Core PX4 infrastructure
```

### Message System

Custom uORB messages for wheel loader operations:
- `BoomCommand/Status` - Boom hydraulic control
- `BucketCommand/Status` - Bucket control and trajectory
- `ChassisCommand/Status` - Chassis coordination and control

See [Design Documentation](design/wheel_loader_robot_design.md) for detailed architecture information.

## Getting Started

### Prerequisites

- **Supported Platforms**: Linux (Ubuntu 20.04+), NuttX for embedded targets
- **Dependencies**: Standard PX4 build dependencies
- **Hardware**: Wheel loader chassis with hydraulic actuators and CAN/PWM interfaces

### Building

```bash
# Clone the repository
git clone https://github.com/WfHit/wheel_loader_robot.git
cd wheel_loader_robot

# Initialize submodules  
git submodule update --init --recursive

# Build for simulation
make px4_sitl

# Build for specific hardware target (example)
make px4_fmu-v5
```

### Configuration

The wheel loader robot uses PX4's parameter system with `WLR_` prefixed parameters:

```bash
# Example key parameters
WLR_BOOM_MAX_LIFT     # Maximum boom lift rate
WLR_BUCKET_MAX_TILT   # Maximum bucket tilt rate  
WLR_CHASSIS_MAX_SPD   # Maximum chassis speed
WLR_SAFETY_ENABLE     # Enable safety systems
```

## Hardware Integration

### Supported Configurations

- **Hydraulic Systems**: Proportional valve control via PWM/CAN
- **Wheel Drive**: Independent motor control (electric/hydraulic)
- **Sensors**: Position encoders, pressure sensors, IMU, GNSS
- **Safety**: Emergency stop circuits, stability monitoring

### Communication Interfaces

- **uORB**: Internal message passing
- **MAVLink**: Ground control station communication
- **CAN**: Hydraulic valve and motor control
- **Serial/Ethernet**: External system integration

## Divergence from PX4

This project has evolved independently from PX4 to focus on wheel loader applications:

### Major Differences

1. **Vehicle Type**: Ground-based construction equipment vs. aerial vehicles
2. **Control Systems**: Hydraulic actuators vs. flight control surfaces  
3. **Navigation**: 2D ground navigation vs. 3D flight navigation
4. **Safety Systems**: Construction site safety vs. aviation safety

### PX4 Heritage

- **Core Infrastructure**: uORB messaging, parameter system, logging
- **Build System**: CMake-based build with PX4 toolchain
- **Communication**: MAVLink protocol and ground station compatibility
- **Hardware Abstraction**: Driver framework and board support

See [PX4_VERSION_TRACKING.md](PX4_VERSION_TRACKING.md) for detailed version history and cherry-pick tracking.

## Documentation

- **[Design Document](design/wheel_loader_robot_design.md)**: Comprehensive system architecture
- **[Module Architecture](src/modules/REDESIGNED_ARCHITECTURE.md)**: Detailed module design
- **[PX4 Version Tracking](PX4_VERSION_TRACKING.md)**: Version history and cherry-pick log

## Development

### Coding Standards

The project follows PX4 coding standards with wheel loader specific adaptations:
- Snake_case file naming (`wheel_loader_robot.hpp`)  
- CamelCase class names (`WheelLoaderRobot`)
- `WLR_` parameter prefix for wheel loader parameters
- Comprehensive safety checks and error handling

### Contributing

1. **Fork** the repository
2. **Create** a feature branch (`git checkout -b feature/amazing-feature`)
3. **Commit** your changes (`git commit -m 'Add amazing feature'`)
4. **Push** to the branch (`git push origin feature/amazing-feature`)  
5. **Open** a Pull Request

### Testing

```bash
# Run unit tests
make tests

# Run integration tests  
make integration_tests

# Hardware-in-the-loop testing
make wheel_loader_hitl
```

## Support

- **Issues**: [GitHub Issues](https://github.com/WfHit/wheel_loader_robot/issues)
- **Discussions**: [GitHub Discussions](https://github.com/WfHit/wheel_loader_robot/discussions)
- **Documentation**: [Project Wiki](https://github.com/WfHit/wheel_loader_robot/wiki)

## License

This project is licensed under the BSD 3-Clause License - see the [LICENSE](LICENSE) file for details.

## Acknowledgments

- **PX4 Development Team**: For the foundational autopilot framework
- **Dronecode Foundation**: For the open-source ecosystem
- **Construction Robotics Community**: For domain expertise and requirements

---

> **Note**: This is an independent project based on PX4 v1.16.0. It is not affiliated with or endorsed by the PX4 Development Team or Dronecode Foundation.