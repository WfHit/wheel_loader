# Steering system status

uint64 timestamp                # Time since system start (microseconds)

float32 actual_angle_rad        # Current steering angle in radians
float32 actual_rate_rad_s       # Current steering rate in rad/s
float32 servo_position_rad      # Servo position in radians
float32 servo_torque_nm         # Applied servo torque in Nm
float32 power_steering_pressure # Power steering pressure in Pa
float32 steering_temperature_c  # Servo controller temperature in Celsius

bool servo_healthy              # Servo controller health status
bool power_steering_active      # Power steering system active
bool position_valid             # Position sensor valid
bool emergency_stop_active      # Emergency stop engaged

uint8 error_flags               # Error flags bitfield
float32 max_torque_available   # Maximum available torque in Nm
