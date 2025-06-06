# Wheel loader setpoint message
# Commands for wheel loader operation

uint64 timestamp		# time since system start (microseconds)

# Wheel control commands
float32 front_wheel_speed	# Front wheel speed setpoint (m/s)
float32 rear_wheel_speed	# Rear wheel speed setpoint (m/s)
float32 articulation_angle_setpoint # Articulation angle setpoint (rad)

# Control modes
uint8 CONTROL_MODE_MANUAL = 0
uint8 CONTROL_MODE_AUTO = 1
uint8 CONTROL_MODE_ASSIST = 2

uint8 control_mode		# Current control mode

# Safety and limits
bool emergency_stop		# Emergency stop flag
float32 max_speed		# Maximum allowed speed (m/s)
