# Wheel Loader Control System

[![Build Status](https://github.com/WfHit/wheel_loader/actions/workflows/build.yml/badge.svg)](https://github.com/WfHit/wheel_loader/actions/workflows/build.yml)

This repository contains an **independent wheel loader control system** based on the [PX4](http://px4.io) autopilot framework. The project has diverged significantly from the main PX4 codebase to provide specialized autonomous control capabilities for wheel loader vehicles.

## Project Overview

This control system is specifically designed for wheel loader operations and includes:
- **Articulated chassis control** with advanced traction management
- **Boom and bucket hydraulic control** for material handling
- **Battery-aware power management** for extended operation
- **Safety systems** designed for construction environments
- **Multi-modal control** supporting manual, semi-autonomous, and fully autonomous operation

The system is built on the robust, real-time PX4 middleware platform but has been extensively modified to meet the unique requirements of wheel loader applications.

## Relationship to PX4

This project was **separated from the main PX4 codebase** due to significant architectural divergence required for wheel loader applications. While it maintains the robust foundation of PX4's real-time operating system and middleware, the control algorithms, safety systems, and operational modes have been completely redesigned for ground-based construction vehicles.

**Base Version**: PX4 v1.16.0  
**Separation Reason**: The wheel loader modifications were too extensive and specialized to be merged back into the main PX4 project, which focuses on aerial vehicles.

### Key Modifications from PX4
- **Ground vehicle dynamics** instead of flight dynamics
- **Hydraulic system control** for boom and bucket operations  
- **Articulated chassis control** with slip estimation and traction management
- **Construction-specific safety systems** and operational modes
- **Power management** optimized for battery-powered heavy equipment

## Installation and Usage

### Prerequisites
- Ubuntu 20.04 or later (tested on Ubuntu 22.04)
- GCC 9+ or Clang 12+
- CMake 3.16+
- Python 3.8+

### Building the System
```bash
git clone https://github.com/WfHit/wheel_loader.git --recursive
cd wheel_loader
make px4_sitl default
```

### Running SITL Simulation
```bash
make px4_sitl gazebo_wheel_loader
```

### Hardware Deployment
Refer to the [deployment documentation](docs/deployment.md) for hardware-specific build targets and configuration.

## Architecture

The control system consists of several specialized modules:

- **Wheel Loader Robot** (`src/modules/wheel_loader_robot/`) - Main coordination and safety management
- **Articulated Chassis** (`src/modules/articulated_chassis/`) - Advanced chassis control with MPC
- **Boom Control** (`src/modules/boom_control/`) - Hydraulic boom actuation
- **Bucket Control** (`src/modules/bucket_control/`) - Hydraulic bucket control
- **Load Monitoring** (`src/modules/load_mon/`) - Payload and stability monitoring
- **Safety Manager** (`src/modules/safety_manager/`) - Construction-specific safety systems

For detailed architecture information, see [design/wheel_loader_robot_design.md](design/wheel_loader_robot_design.md).

## Version Tracking

This project maintains independent versioning while tracking its relationship to upstream PX4 changes. See [PX4_VERSION_TRACKING.md](PX4_VERSION_TRACKING.md) for detailed version history and cherry-picked changes.

## Building a Wheel Loader Control System

This system is designed for integration with wheel loader hardware platforms. Supported configurations include:
- Battery-powered compact wheel loaders
- Hydraulic system integration via CAN/PWM interfaces  
- Articulated chassis with front and rear axles
- Various sensor packages (IMU, encoders, load cells, etc.)

For hardware integration guides, see the [hardware documentation](docs/hardware/).

## Contributing to Wheel Loader Development

This project welcomes contributions specifically related to wheel loader control systems. Please read our [wheel loader contribution guidelines](CONTRIBUTING.md) before submitting changes.

**Focus areas for contributions:**
- Autonomous digging and loading algorithms
- Improved hydraulic control strategies  
- Enhanced safety systems for construction environments
- Power optimization for battery operations
- Integration with fleet management systems

See our [project roadmap](docs/roadmap.md) for planned features and development priorities.


## Maintenance Team

See the latest list of maintainers on [MAINTAINERS](MAINTAINERS.md) file at the root of the project.

## License and Acknowledgments

This project is licensed under the BSD 3-clause license - see the [LICENSE](LICENSE) file for details.

**Acknowledgments:**
- Based on the [PX4 Autopilot](https://github.com/PX4/PX4-Autopilot) framework v1.16.0
- PX4 Development Team for the foundational middleware and real-time systems
- [Dronecode Foundation](https://www.dronecode.org/) for supporting open-source autopilot development

<a href="https://www.dronecode.org/" style="padding:20px" ><img src="https://dronecode.org/wp-content/uploads/sites/24/2020/08/dronecode_logo_default-1.png" alt="Dronecode Logo" width="110px"/></a>
<div style="padding:10px">&nbsp;</div>
