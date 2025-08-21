# Chassis command message for wheel loader mobility control

uint64 timestamp		# time since system start (microseconds)

# Motion commands
float32 linear_velocity		# Desired forward/backward velocity [m/s]
float32 angular_velocity	# Desired turning rate [rad/s]
float32 lateral_velocity	# Desired lateral velocity (for special maneuvers) [m/s]

# Position commands (for position control mode)
float32 x_position		# Target X position [m]
float32 y_position		# Target Y position [m]
float32 heading			# Target heading [rad]

# Control mode
uint8 MODE_IDLE = 0
uint8 MODE_VELOCITY = 1
uint8 MODE_POSITION = 2
uint8 MODE_TRAJECTORY = 3
uint8 mode

# Control flags
bool emergency_stop
bool traction_control_override
bool stability_control_override
