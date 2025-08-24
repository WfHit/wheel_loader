# Contributing to Wheel Loader Control System

We welcome contributions to the wheel loader control system! This project follows GitHub flow development model with wheel loader specific considerations.

## Development Overview

This project is an **independent wheel loader control system** based on PX4 v1.16.0. Contributions should focus on:
- Autonomous wheel loader operations
- Construction industry safety requirements
- Hydraulic system control improvements  
- Battery power management optimization
- Articulated chassis control enhancements

## Prerequisites

Before contributing, please:
1. Read the [wheel loader design documentation](design/wheel_loader_robot_design.md)
2. Understand the [project architecture](src/modules/REDESIGNED_ARCHITECTURE.md)
3. Review [PX4 version tracking](PX4_VERSION_TRACKING.md) for upstream relationship

## Contribution Process

### 1. Fork and Clone
First [fork and clone](https://help.github.com/articles/fork-a-repo) the wheel loader repository.

### 2. Create a Feature Branch
Always branch off the main branch for new features:
```bash
git checkout -b feature/descriptive-wheel-loader-feature-name
```

### 3. Development Environment
Set up your development environment following the [installation instructions](README.md#installation-and-usage). 

**Wheel Loader Specific Setup:**
- Ensure hydraulic simulation plugins are available
- Configure for articulated chassis if testing chassis control
- Set up wheel loader specific simulation environments

### 4. Coding Standards
Follow PX4 coding conventions with wheel loader specific considerations:
- **Snake_case** for module files (`wheel_loader_robot.cpp`)
- **CamelCase** for class names (`WheelLoaderRobot`)
- **Underscore prefix** for private members (`_control_state`)
- **WLR_** prefix for wheel loader parameters
- Document safety-critical code thoroughly

### 5. Testing Requirements
All contributions must include appropriate testing:

**Simulation Testing (Required):**
```bash
# Basic functionality test
make px4_sitl gazebo_wheel_loader

# Safety system test
make px4_sitl test_wheel_loader_safety
```

**Hardware Testing (When Available):**
- Test on actual wheel loader hardware when possible
- Provide equipment logs from testing sessions
- Include hydraulic system response data

### 6. Commit Guidelines
Write descriptive commit messages with wheel loader context:

**Example:**
```
wheel_loader: improve hydraulic response time

- Optimize boom control PID parameters
- Add safety timeout for bucket operations  
- Reduce power consumption during idle

Fixes issue #123
```

**Commit Categories:**
- `wheel_loader:` - Core wheel loader functionality
- `chassis:` - Articulated chassis control
- `hydraulics:` - Boom/bucket control systems
- `safety:` - Safety system improvements
- `power:` - Battery/power management
- `docs:` - Documentation updates

### 7. Pull Request Process
1. **Test thoroughly** - Include simulation and hardware test results
2. **Update documentation** - Update relevant design docs if needed
3. **Safety review** - Ensure changes don't compromise safety systems
4. **Performance impact** - Consider impact on real-time performance

## Contribution Areas

### High Priority Areas
- **Autonomous digging algorithms** - Improve material handling efficiency
- **Safety system hardening** - Enhanced protection for construction environments  
- **Power optimization** - Extend battery life and improve efficiency
- **Load estimation** - Better payload detection and handling
- **Slip prevention** - Advanced traction control

### Medium Priority Areas  
- **Fleet coordination** - Multi-vehicle operation support
- **Predictive maintenance** - Component wear monitoring
- **UI/UX improvements** - Operator interface enhancements
- **Simulation fidelity** - More realistic wheel loader physics

### Areas Requiring Special Consideration
- **Real-time safety systems** - Requires extensive testing and review
- **Hydraulic control** - Must maintain precise safety margins
- **Power management** - Critical for operational safety
- **Core middleware changes** - May affect PX4 compatibility

## Code Review Process

All submissions go through code review focusing on:
1. **Safety implications** - Construction equipment safety is paramount
2. **Real-time performance** - Maintain deterministic behavior  
3. **Code quality** - Follow established patterns and standards
4. **Testing coverage** - Verify functionality across configurations
5. **Documentation** - Ensure maintainability

## Questions or Issues?

- **General questions**: Open a GitHub discussion
- **Bug reports**: Use GitHub issues with wheel loader templates
- **Security concerns**: Contact maintainers privately
- **Feature requests**: Propose via GitHub issues with detailed use cases

## Wheel Loader Specific Resources

- [Wheel Loader Robot Design](design/wheel_loader_robot_design.md)
- [Architecture Overview](src/modules/REDESIGNED_ARCHITECTURE.md)  
- [PX4 Integration Guide](PX4_VERSION_TRACKING.md)
- [Safety Systems Documentation](docs/safety/README.md)
- [Testing Procedures](docs/testing/README.md)
