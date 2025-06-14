# Load sensing data for wheel loader
# This message contains information about the current load/payload

uint64 timestamp				# time since system start (microseconds)

float32 payload_mass_kg			# Payload mass in kilograms
float32 load_percentage			# Load as percentage of maximum capacity (0-100%)
float32 center_of_gravity_x		# Center of gravity X offset from nominal (meters)
float32 center_of_gravity_y		# Center of gravity Y offset from nominal (meters)
float32 center_of_gravity_z		# Center of gravity Z offset from nominal (meters)

uint8 load_status				# Load status flags
uint8 LOAD_STATUS_EMPTY = 0
uint8 LOAD_STATUS_PARTIAL = 1
uint8 LOAD_STATUS_FULL = 2
uint8 LOAD_STATUS_OVERLOAD = 3

bool valid					# True if load data is valid
