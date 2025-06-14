# Boom hydraulic control command

uint64 timestamp                # time since system start (microseconds)

# Position control
float32 lift_angle_cmd          # Boom lift angle command (rad, relative to horizontal)
float32 extend_position_cmd     # Boom extension command (m or normalized 0-1)

# Velocity control
float32 lift_velocity_cmd       # Boom lift angular velocity (rad/s)
float32 extend_velocity_cmd     # Boom extension velocity (m/s or normalized)

# Control modes
uint8 MODE_POSITION = 0         # Position control mode
uint8 MODE_VELOCITY = 1         # Velocity control mode
uint8 MODE_FORCE = 2            # Force/pressure control mode
uint8 MODE_COORDINATED = 3      # Coordinated with bucket operation
uint8 control_mode

# Operational limits and safety
float32 max_lift_velocity       # Maximum lift angular velocity (rad/s)
float32 max_extend_velocity     # Maximum extension velocity (m/s)
float32 max_load_factor         # Maximum load factor (0-1)
bool enable_load_limiting       # Enable load-based limiting
bool emergency_stop             # Emergency stop command

# Priority and arbitration
uint8 PRIORITY_LOW = 0
uint8 PRIORITY_NORMAL = 1
uint8 PRIORITY_HIGH = 2
uint8 PRIORITY_EMERGENCY = 3
uint8 command_priority
