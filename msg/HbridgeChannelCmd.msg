# H-Bridge channel command message
# Commands for individual H-bridge channels

uint64 timestamp         # time since system start (microseconds)

uint8 channel_id         # Channel identifier (0-based)
bool enable              # Enable/disable channel
float32 command_value    # Command value (-1.0 to 1.0, 0 = stop)
float32 max_current      # Maximum current limit (amps)
bool ignore_limits       # Override limit switches (safety critical)
uint8 priority           # Command priority (0=lowest, 255=highest)

# Command mode
uint8 command_mode       # 0: POSITION, 1: VELOCITY, 2: CURRENT
uint8 MODE_POSITION = 0
uint8 MODE_VELOCITY = 1
uint8 MODE_CURRENT = 2
