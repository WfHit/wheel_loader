# PX4 Version Tracking for Wheel Loader Robot

This file tracks the PX4 version alignment and cherry-picked changes for the independent wheel_loader_robot project.

## Base PX4 Version

**Base Version**: PX4 v1.16.0  
**Branch Used**: px4-v1.16.0-clean-cherry-pick  
**Date**: 2024-08-24  
**Reason**: Established as independent wheel loader robot project with diverged functionality

## Project Divergence Summary

The wheel_loader_robot project has diverged significantly from the main PX4 codebase with the following major additions:

### Custom Modules Added:
- `src/modules/boom_control/` - Hydraulic boom control system
- `src/modules/bucket_control/` - Hydraulic bucket control system  
- `src/modules/articulated_chassis/` - Chassis control including:
  - `wheel_controller/` - Independent wheel control
  - `steering_controller/` - Articulated steering control
  - `traction_control/` - Advanced traction management

### Custom Message Definitions:
- `BoomCommand.msg` / `BoomStatus.msg` - Boom hydraulic control
- `BucketCommand.msg` / `BucketStatus.msg` / `BucketControlCommand.msg` / `BucketTrajectorySetpoint.msg` - Bucket control
- `ChassisCommand.msg` / `ChassisStatus.msg` / `ChassisControlCommand.msg` / `ChassisTrajectorySetpoint.msg` - Chassis control

### Design Documentation:
- `design/wheel_loader_robot_design.md` - Comprehensive system architecture

### Architecture Documents:
- `src/modules/REDESIGNED_ARCHITECTURE.md` - Detailed wheel loader control architecture
- Various module-specific documentation files

## Cherry-Pick Log

This section will track future cherry-picks from PX4 main branch to keep the wheel loader robot updated with relevant PX4 improvements.

### Template for Future Entries:
```
### [Date] - PX4 Commit [hash]
**PX4 Version**: [version/tag]
**Commit**: [commit_hash]
**Subject**: [commit_subject]
**Files Modified**: [list_of_files]
**Reason**: [why_this_commit_was_cherry_picked]
**Conflicts**: [any_conflicts_and_resolutions]
**Testing**: [testing_performed]
```

## Cherry-Pick Guidelines

When cherry-picking commits from PX4:

1. **Focus Areas**: Prioritize commits related to:
   - Core PX4 infrastructure improvements
   - uORB message system updates
   - Build system enhancements
   - Security fixes
   - Performance improvements
   - Hardware support updates relevant to wheel loader platforms

2. **Avoid**: 
   - Vehicle-specific changes for multicopters, fixed-wing, VTOL
   - Flight-specific navigation and control algorithms
   - Airspace/aviation-specific features

3. **Test Requirements**:
   - Build verification for all wheel loader modules
   - Regression testing of boom, bucket, and chassis controls
   - Hardware-in-the-loop testing when possible

4. **Documentation**:
   - Update this tracking file for each cherry-pick
   - Note any conflicts and their resolution
   - Update wheel loader design docs if interfaces change

## Maintenance Schedule

- **Monthly**: Review PX4 main branch for relevant commits
- **Quarterly**: Evaluate need for major PX4 version alignment
- **As Needed**: Cherry-pick critical security fixes immediately

## Contact

For questions about PX4 version tracking and cherry-pick decisions, contact the wheel loader robot project maintainers.