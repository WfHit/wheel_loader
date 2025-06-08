# H-Bridge system status message

uint64 timestamp                    # Time since system start (microseconds)

# System states
uint8 SYSTEM_STATE_IDLE = 0
uint8 SYSTEM_STATE_ACTIVE = 1
uint8 SYSTEM_STATE_FAULT = 2
uint8 SYSTEM_STATE_EMERGENCY = 3

# System-wide status
bool system_enabled                 # Overall system enable state
uint8 system_state                  # Overall system state
bool emergency_stop                 # Emergency stop active
float32 board_temperature           # H-bridge board temperature (Celsius)
float32 supply_voltage              # Supply voltage (Volts)
float32 total_current               # Total current draw (Amperes)

# Safety and limits
bool safety_override_active         # Safety system allowing limit override
uint8 active_channel_count          # Number of currently active channels
uint8 fault_channel_count           # Number of channels in fault state
uint32 system_fault_count           # Total system fault count since startup

# System capabilities
uint8 max_channels                  # Maximum number of channels supported
float32 max_total_current           # Maximum total current limit (Amperes)
float32 max_temperature             # Maximum operating temperature (Celsius)
