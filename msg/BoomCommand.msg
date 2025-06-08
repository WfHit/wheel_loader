uint64 timestamp                # time since system start (microseconds)
float32 target_angle           # target boom angle in radians (relative to horizontal)
float32 max_velocity           # maximum angular velocity (rad/s)
float32 max_load               # maximum load limit (normalized 0-1)
uint8 command_mode             # 0: position, 1: velocity, 2: force
uint8 priority                 # command priority (0: low, 1: normal, 2: high, 3: emergency)
bool emergency_stop            # emergency stop command
