# VLA command interface for autonomous wheel loader control
# Simplified interface for Vision-Language-Action model integration

uint64 timestamp                    # microseconds since boot

# Basic movement commands
float32 forward_speed               # Forward/backward speed (-1 to 1)
float32 steering_angle              # Steering angle (-1 to 1, normalized)

# Bucket control
float32 bucket_tilt                 # Bucket tilt command (-1 to 1, normalized)
float32 boom_lift                   # Boom lift command (-1 to 1, normalized)

# Operation mode
uint8 OPERATION_IDLE = 0            # No active operation
uint8 OPERATION_LOAD = 1            # Auto load sequence
uint8 OPERATION_DUMP = 2            # Auto dump sequence
uint8 operation_mode                # Current operation mode

# Control flags
bool valid_command                  # Whether command is valid
bool emergency_stop                 # Emergency stop from VLA