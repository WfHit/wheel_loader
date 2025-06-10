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

# Control mode constants
uint8 MODE_MANUAL = 0
uint8 MODE_AUTO_LEVEL = 1
uint8 MODE_SLOPE_COMPENSATION = 2
uint8 MODE_GRADING = 3
uint8 MODE_TRANSPORT = 4
