# Wheel loader status message
# Feedback from wheel loader subsystems

uint64 timestamp		# time since system start (microseconds)

# Wheel status
float32 front_wheel_speed	# Actual front wheel speed (m/s)
float32 rear_wheel_speed	# Actual rear wheel speed (m/s)
float32 articulation_angle_feedback # Articulation angle feedback (rad)

# Wheel motor currents
float32 front_motor_current	# Front motor current (A)
float32 rear_motor_current	# Rear motor current (A)

# System health
uint8 HEALTH_OK = 0
uint8 HEALTH_WARNING = 1
uint8 HEALTH_ERROR = 2
uint8 HEALTH_CRITICAL = 3

uint8 system_health		# Overall system health status
bool motor_fault		# Motor fault indicator
bool communication_fault	# Communication fault

# Temperature monitoring
float32 motor_temperature	# Motor temperature (°C)
float32 controller_temperature	# Controller board temperature (°C)

# Power status
float32 supply_voltage		# Supply voltage (V)
float32 power_consumption	# Power consumption (W)

# Operational counters
uint32 operating_hours		# Total operating hours
uint32 cycle_count		# Work cycle count
