# H-Bridge channel status

# Channel states
uint8 STATE_IDLE = 0
uint8 STATE_ACTIVE = 1
uint8 STATE_FAULT = 2

bool enabled                       # Channel enabled
uint8 state                       # Channel state
bool limit_min_active             # Minimum limit switch active
bool limit_max_active             # Maximum limit switch active
float32 current_command           # Last command applied
