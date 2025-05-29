# Quadrature encoder data

uint64 timestamp        # time since system start (microseconds)
uint32 device_id        # unique device ID for multiple encoders

# Raw encoder data
int32 position          # Encoder position in counts
int32 velocity          # Velocity in counts/second

# Processed data
float32 angle           # Angle in radians
float32 angular_velocity # Angular velocity in rad/s

# For linear applications (e.g., wheel, linear actuator)
float32 distance        # Linear distance (m) - calculated from counts
float32 speed           # Linear speed (m/s)

# Encoder configuration
uint16 pulses_per_rev   # Encoder resolution
uint8 mode              # Counting mode (1=X1, 2=X2, 4=X4)

# Status
uint8 status            # Encoder status flags
uint8 error_count       # Error counter
bool index_detected     # Index pulse detected

uint8 ORB_QUEUE_LENGTH = 4
