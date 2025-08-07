uint64 timestamp                # time since system start (microseconds)
float32 target_angle           # target bucket angle in radians (ground reference)
float32 max_velocity           # maximum angular velocity (rad/s)
uint8 command_mode             # 0: position, 1: velocity
uint8 coordinate_frame         # 0: ground reference, 1: boom reference

# Control mode selection
uint8 control_mode             # 0: manual, 1: auto-level, 2: slope-comp, 3: grading, 4: transport
bool enable_stability_limit    # enable stability-based movement limiting
bool enable_anti_spill         # enable anti-spill control in transport mode

# Advanced control parameters
float32 grading_angle          # override for grading angle (rad) - NaN to use parameter
float32 transport_angle        # override for transport angle (rad) - NaN to use parameter
float32 stability_threshold    # override for stability threshold (rad) - NaN to use parameter

# Control modes (only modes actually used in code)
uint8 MODE_CALIBRATE = 0       # Calibration mode
uint8 MODE_POSITION = 1        # Position control mode
uint8 MODE_DIRECT = 2          # Direct actuator control
uint8 MODE_STOP = 3            # Stop all movement
uint8 MODE_EMERGENCY_STOP = 4  # Emergency stop
