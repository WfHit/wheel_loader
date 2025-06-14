# Steering system control commands

uint64 timestamp                # time since system start (microseconds)

# Front axle steering
float32 front_steering_angle_cmd # Front axle steering angle (rad)
float32 front_steering_rate_cmd  # Front axle steering rate (rad/s)

# Frame articulation (for articulated loaders)
float32 articulation_angle_cmd   # Frame articulation angle (rad)
float32 articulation_rate_cmd    # Frame articulation rate (rad/s)

# Rear axle steering (if equipped)
float32 rear_steering_angle_cmd  # Rear axle steering angle (rad)
float32 rear_steering_rate_cmd   # Rear axle steering rate (rad/s)

# Control modes
uint8 MODE_FRONT_ONLY = 0       # Front axle steering only
uint8 MODE_ARTICULATED = 1      # Frame articulation only
uint8 MODE_FOUR_WHEEL = 2       # Front and rear axle steering
uint8 MODE_COORDINATED = 3      # Coordinated front, rear, and articulation
uint8 steering_mode

# Steering control type
uint8 CONTROL_POSITION = 0      # Position control
uint8 CONTROL_RATE = 1          # Rate control
uint8 control_type

# Operational limits
float32 max_steering_rate       # Maximum steering rate (rad/s)
float32 max_articulation_rate   # Maximum articulation rate (rad/s)
bool enable_rate_limiting       # Enable rate limiting
bool enable_stability_assist    # Enable stability-based steering assist

# Safety
bool emergency_center           # Emergency return to center position
bool power_steering_enabled     # Enable power steering assistance

# Priority
uint8 PRIORITY_LOW = 0
uint8 PRIORITY_NORMAL = 1
uint8 PRIORITY_HIGH = 2
uint8 PRIORITY_EMERGENCY = 3
uint8 command_priority
