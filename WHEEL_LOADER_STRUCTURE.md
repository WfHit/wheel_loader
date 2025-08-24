# Wheel Loader Project Structure

This document explains the branching strategy and project structure for the independent wheel loader control system.

## Project Overview

This project is a **standalone wheel loader control system** that was separated from the main PX4 project due to extensive architectural modifications required for ground-based construction vehicles.

## Branching Strategy

### Base Branch
- **Branch Name**: `px4-v1.16.0-clean-cherry-pick`
- **Purpose**: Clean starting point based on PX4 v1.16.0
- **Status**: Base for all wheel loader development

### Development Workflow
1. **Feature Development**: Create feature branches from the base branch
2. **Integration**: Merge features back to main development branch
3. **Cherry-picking**: Selectively integrate upstream PX4 fixes when relevant

### Branch Types
- `main` / `wheel_loader` - Primary development branch
- `feature/*` - Feature development branches
- `hotfix/*` - Critical bug fixes
- `upstream/*` - Branches for testing upstream PX4 changes

## Directory Structure

### Core Wheel Loader Modules
```
src/modules/
├── wheel_loader_robot/       # Main coordination module
├── articulated_chassis/       # Advanced chassis control
├── boom_control/             # Hydraulic boom control
├── bucket_control/           # Hydraulic bucket control
├── load_mon/                 # Load monitoring and safety
├── safety_manager/           # Construction-specific safety
└── ...                       # Standard PX4 modules (modified)
```

### Design Documentation
```
design/
├── wheel_loader_robot_design.md    # Main design document
├── uorb_proxy_architecture.md      # System architecture
└── ...                            # Additional design docs
```

### Configuration
```
boards/                       # Hardware board configurations
msg/                         # uORB message definitions
ROMFS/                       # Runtime file system
launch/                      # Launch configurations
```

## Future PX4 Integration Strategy

### Principles
1. **Selective Integration**: Only cherry-pick relevant upstream changes
2. **Compatibility Testing**: Validate all upstream changes against wheel loader functionality
3. **Documentation**: Track all cherry-picked changes in [PX4_VERSION_TRACKING.md](PX4_VERSION_TRACKING.md)

### Cherry-Pick Process
1. **Evaluation Phase**
   - Assess upstream change for wheel loader relevance
   - Check for conflicts with wheel loader modifications
   - Evaluate safety implications

2. **Integration Phase**
   - Create test branch for integration
   - Resolve any conflicts
   - Test thoroughly with wheel loader simulation

3. **Documentation Phase**
   - Update version tracking file
   - Document any modifications needed
   - Record testing results

### Priority Areas for Upstream Integration
- **High Priority**: Security fixes, core middleware stability
- **Medium Priority**: Hardware driver updates, build system improvements
- **Low Priority**: Flight-specific features (typically not applicable)

## Maintenance Guidelines

### Code Style
- Follow PX4 coding standards where applicable
- Use wheel loader specific prefixes for new parameters (`WLR_`)
- Maintain compatibility with existing wheel loader interfaces

### Testing Requirements
- Simulation testing for all changes
- Hardware testing when available
- Safety system validation for control changes

### Release Management
- Independent versioning (separate from PX4)
- Release notes focused on wheel loader functionality
- Compatibility documentation for different wheel loader configurations

## Build Targets

### Simulation
```bash
make px4_sitl                    # Standard SITL simulation
make px4_sitl_wheel_loader      # Wheel loader specific simulation
```

### Hardware Targets
Refer to individual board configurations in `boards/` directory for supported hardware platforms.

## Contributing

See [CONTRIBUTING.md](CONTRIBUTING.md) for detailed contribution guidelines specific to the wheel loader project.