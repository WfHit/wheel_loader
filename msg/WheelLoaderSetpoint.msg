# Wheel loader setpoint with velocity and force commands

uint64 timestamp		# time since system start (microseconds)

# Velocity commands (m/s)
float32 front_wheel_velocity
float32 rear_wheel_velocity

# Force/torque commands (N)
float32 front_wheel_force
float32 rear_wheel_force

# Control mode flags
bool velocity_control		# Use velocity control
bool force_control		# Use force/torque control
bool traction_control_active	# Traction control is active
