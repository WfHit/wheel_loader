# Chassis status message for wheel loader mobility system

uint64 timestamp		# time since system start (microseconds)

# Motion state
float32 linear_velocity		# Current forward/backward velocity [m/s]
float32 angular_velocity	# Current turning rate [rad/s]
float32 lateral_velocity	# Current lateral velocity [m/s]
float32 steering_angle		# Current articulation angle [rad]

# Wheel state (FL, FR, RL, RR)
float32[4] wheel_speeds		# Individual wheel speeds [rad/s]
float32[4] wheel_slip		# Wheel slip ratios
float32[4] wheel_torques	# Applied wheel torques [Nm]

# Traction information
float32 traction_mu		# Average friction coefficient estimate
bool asr_active			# Anti-slip regulation active
bool esp_active			# Electronic stability program active

# System state
uint8 mode			# Current control mode
uint8 state			# Current chassis state

# Health
uint8 HEALTH_OK = 0
uint8 HEALTH_WARNING = 1
uint8 HEALTH_ERROR = 2
uint8 health
