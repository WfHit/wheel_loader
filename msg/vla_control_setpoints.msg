# VLA (Vision-Language-Action) control setpoints
# Contains bucket end effector pose and position commands from VLA system

uint64 timestamp		# time since system start (microseconds)

# Bucket end effector position setpoints
float32 x_position		# Target bucket tip X position [m] (world frame)
float32 y_position		# Target bucket tip Y position [m] (world frame)
float32 z_position		# Target bucket tip Z position [m] (height above ground)

# Bucket end effector orientation setpoints
float32 roll			# Target bucket roll angle [rad]
float32 pitch			# Target bucket pitch angle [rad]
float32 yaw			# Target bucket yaw angle [rad]

# Control mode
uint8 MODE_IDLE = 0
uint8 MODE_POSITION = 1
uint8 MODE_TRAJECTORY = 2
uint8 mode

# Command validity and priority
bool valid			# True if command is valid
uint8 priority			# Command priority (0 = highest)
