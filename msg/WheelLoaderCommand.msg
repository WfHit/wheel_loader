# Wheel loader control command with traction control support

uint64 timestamp                    # microseconds since boot

# Wheel speed commands (rad/s)
float32 front_left_speed
float32 front_right_speed
float32 rear_left_speed
float32 rear_right_speed

# Actuator commands (rad)
float32 bucket_angle
float32 boom_angle
float32 steering_angle

# Traction control
bool traction_control_enabled
float32 max_slip_ratio             # Maximum allowed slip before intervention
float32 traction_gain              # How aggressively to reduce power

# Control modes
uint8 CONTROL_MODE_MANUAL = 0
uint8 CONTROL_MODE_ASSISTED = 1
uint8 CONTROL_MODE_AUTO = 2
uint8 control_mode

# Emergency stop
bool emergency_stop
