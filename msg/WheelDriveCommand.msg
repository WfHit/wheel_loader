# Individual wheel drive control commands

uint64 timestamp                # time since system start (microseconds)

# Individual wheel speed commands (rad/s)
float32 front_left_speed_cmd
float32 front_right_speed_cmd
float32 rear_left_speed_cmd
float32 rear_right_speed_cmd

# Individual wheel torque commands (Nm) - alternative to speed control
float32 front_left_torque_cmd
float32 front_right_torque_cmd
float32 rear_left_torque_cmd
float32 rear_right_torque_cmd

# Control modes per wheel
uint8 MODE_SPEED = 0            # Speed control mode
uint8 MODE_TORQUE = 1           # Torque control mode
uint8 MODE_DISABLED = 2         # Wheel disabled/freewheeling
uint8 front_left_mode
uint8 front_right_mode
uint8 rear_left_mode
uint8 rear_right_mode

# Limits and safety
float32 max_acceleration        # Maximum acceleration (rad/s²)
float32 max_deceleration        # Maximum deceleration (rad/s²)
bool enable_slip_control        # Enable individual wheel slip control
bool enable_differential_lock   # Enable differential locking
bool emergency_stop             # Emergency stop all wheels

# Synchronized operation
bool synchronized_front         # Synchronize front wheels
bool synchronized_rear          # Synchronize rear wheels
bool synchronized_left          # Synchronize left side wheels
bool synchronized_right         # Synchronize right side wheels
bool synchronized_all           # Synchronize all wheels

# Priority
uint8 PRIORITY_LOW = 0
uint8 PRIORITY_NORMAL = 1
uint8 PRIORITY_HIGH = 2
uint8 PRIORITY_EMERGENCY = 3
uint8 command_priority
