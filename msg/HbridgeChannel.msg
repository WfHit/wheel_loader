# H-Bridge channel status message

uint64 timestamp                  # Time since system start (microseconds)
uint8 channel_id                  # Channel identifier (0-based)

# Channel states
uint8 STATE_IDLE = 0
uint8 STATE_ACTIVE = 1
uint8 STATE_FAULT = 2
uint8 STATE_OVERCURRENT = 3
uint8 STATE_OVERHEAT = 4

bool enabled                      # Channel enabled status
uint8 state                       # Current channel state
float32 current_command           # Last command value applied (-1.0 to 1.0)
float32 actual_current            # Measured current (Amperes)
float32 temperature               # Channel temperature (Celsius)
bool limit_min_active             # Minimum limit switch active
bool limit_max_active             # Maximum limit switch active
bool fault_overcurrent            # Overcurrent fault flag
bool fault_overheat               # Overheating fault flag
uint32 fault_count                # Total fault count since startup
