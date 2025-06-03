uint64 timestamp               # time since system start (microseconds)
float32 current_angle          # current bucket angle in radians (ground reference)
float32 bucket_relative_angle  # current bucket angle relative to boom (radians)
float32 current_velocity       # current angular velocity (rad/s)
float32 actuator_length        # current actuator length in mm
bool limit_switch_min          # coarse limit switch state (down position)
bool limit_switch_max          # fine limit switch state (up position)
uint8 state                    # 0: uninitialized, 1: zeroing, 2: ready, 3: moving, 4: error
