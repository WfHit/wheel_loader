# PX4 Version Tracking

This document tracks the PX4 version this wheel loader control system is based on and documents any cherry-picked changes from upstream PX4.

## Base Version

**PX4 Version**: v1.16.0  
**Base Branch**: `px4-v1.16.0-clean-cherry-pick`  
**Separation Date**: 2024  
**Last Sync**: PX4 v1.16.0

## Version History

### v1.16.0 (Base Version)
- **Source**: [PX4-Autopilot v1.16.0](https://github.com/PX4/PX4-Autopilot/releases/tag/v1.16.0)
- **Commit**: Base commit for wheel loader modifications
- **Status**: Full fork - extensive modifications made this unsuitable for upstream merge

## Cherry-Pick Tracking Framework

This section documents selective integration of upstream PX4 changes. Each entry should include:

### Template for Future Cherry-Picks
```
### [Date] - [Feature/Fix Description]
- **Upstream Commit**: [commit hash]
- **Upstream PR**: [PX4-Autopilot#XXXXX](link)
- **Local Commit**: [commit hash]
- **Conflicts**: [Description of any conflicts resolved]
- **Testing**: [Description of testing performed]
- **Notes**: [Any wheel loader specific adaptations made]
```

## Integration Strategy

### Cherry-Pick Process
1. **Evaluation**: Assess upstream changes for relevance to wheel loader operations
2. **Compatibility**: Verify compatibility with wheel loader modifications
3. **Testing**: Comprehensive testing in wheel loader simulation and hardware
4. **Documentation**: Update this tracking file

### Priority Areas for Upstream Integration
- **Core middleware improvements** (uORB, workqueue, etc.)
- **Safety system enhancements** 
- **Build system and toolchain updates**
- **Hardware driver updates** (where applicable)
- **Performance optimizations**

### Areas Generally NOT Cherry-Picked
- **Flight dynamics and control** (not applicable to ground vehicles)
- **Airframe configurations** (replaced with wheel loader configs)
- **Flight-specific safety logic** (replaced with construction-specific safety)
- **Aerodynamic estimators** (replaced with ground vehicle dynamics)

## Maintenance Notes

### When to Cherry-Pick
- **Security fixes**: High priority for integration
- **Core system stability**: Important for reliable operation
- **Hardware support**: When adding new supported hardware
- **Build system**: To maintain compatibility with development tools

### When NOT to Cherry-Pick
- **Flight-specific features**: Not applicable to wheel loaders
- **Breaking changes to core APIs**: May conflict with wheel loader modifications
- **Large architectural changes**: Require careful evaluation and potential major integration work

## Future Version Planning

### Planned Upstream Tracking
- Monitor PX4 releases for relevant improvements
- Quarterly evaluation of upstream changes
- Annual major version consideration (when wheel loader code allows)

### Major Version Upgrade Considerations
Future major PX4 version upgrades will require:
1. **Compatibility assessment** of wheel loader modifications
2. **Migration planning** for modified subsystems  
3. **Extensive testing** across all wheel loader configurations
4. **Documentation updates** for changed APIs

## Contact

For questions about PX4 integration or cherry-pick decisions, contact the wheel loader maintenance team.