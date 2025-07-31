uint64 timestamp          # time since system start (microseconds)
uint8 instance            # H-bridge driver instance
uint8 MAX_CHANNELS = 2    # Maximum number of channels

bool[2] channel_enabled       # Enable status for each channel
float32[2] channel_duty_cycle # Current duty cycle for each channel
float32[2] channel_current    # Current measurement (if available)
float32 temperature          # Board temperature (if available)
bool fault_detected          # Any fault detected

# Channel limit sensor status (connected to H-bridge driver)
bool[4] channel_limit_state    # State of channel limit sensors 0-3
bool channel_limits_available  # True if channel limit sensors are connected

uint8 ORB_QUEUE_LENGTH = 2
