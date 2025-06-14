# Wheel loader control command with input source tracking and comprehensive actuator control

uint64 timestamp                    # microseconds since boot

# Command source tracking
uint8 SOURCE_MANUAL_CONTROL = 0     # Commands from manual RC/joystick control
uint8 SOURCE_TASK_EXECUTION = 1     # Commands from autonomous task execution module
uint8 SOURCE_EXTERNAL = 2           # Commands from external system (MAVLink, etc.)
uint8 command_source                 # Source of this command

# Wheel speed commands (rad/s) - separate control for each wheel
float32 front_left_wheel_speed
float32 front_right_wheel_speed
float32 rear_left_wheel_speed
float32 rear_right_wheel_speed

# Hydraulic actuator commands
float32 bucket_angle_cmd            # Bucket tilt angle command (rad)
float32 boom_lift_cmd               # Boom lift angle command (rad)
float32 boom_extend_cmd             # Boom extension command (m or %)

# Steering control
float32 steering_angle_cmd          # Front axle steering angle (rad)
float32 articulation_angle_cmd      # Frame articulation angle (rad)

# Drivetrain control modes
uint8 DRIVE_MODE_MANUAL = 0         # Direct wheel speed control
uint8 DRIVE_MODE_VELOCITY = 1       # Vehicle velocity and heading control
uint8 DRIVE_MODE_POSITION = 2       # Position-based control
uint8 drive_mode

# Hydraulic control modes
uint8 HYDRAULIC_MODE_MANUAL = 0     # Direct actuator control
uint8 HYDRAULIC_MODE_COORDINATED = 1 # Coordinated bucket/boom operation
uint8 HYDRAULIC_MODE_AUTO = 2       # Automated operation sequences
uint8 hydraulic_mode

# Traction and stability control
bool traction_control_enabled
float32 max_slip_ratio              # Maximum allowed slip before intervention
float32 traction_gain               # How aggressively to reduce power
bool stability_control_enabled      # Rollover prevention system

# Safety and operational limits
bool emergency_stop                 # Immediate stop all operations
bool enable_hydraulics              # Enable/disable hydraulic system
bool enable_drivetrain              # Enable/disable drivetrain
float32 max_vehicle_speed           # Maximum vehicle speed limit (m/s)
float32 max_hydraulic_speed         # Maximum hydraulic operation speed

# Task execution context (when SOURCE_TASK_EXECUTION)
uint32 task_id                      # Current task identifier
uint8 task_phase                    # Current phase of the task
