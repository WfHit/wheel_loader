# Feedback message for smart servos like ST3125

uint64 timestamp          # time since system start (microseconds)
uint8 id                  # Servo ID
float32 position          # Current position (radians)
float32 velocity          # Current velocity (rad/s)
float32 current           # Current draw (Amperes)
float32 voltage           # Supply voltage (Volts)
float32 temperature       # Temperature (Celsius)
uint16 error_flags        # Error status flags
bool torque_enabled       # Torque enable status
float32 goal_position     # Target position (radians)
float32 load              # Current load (-1.0 to 1.0)
