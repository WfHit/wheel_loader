# Operation mode selection command for dual-mode wheel loader control
# Supports switching between manual RC control and autonomous VLA control

uint64 timestamp                    # time since system start (microseconds)

# Operation modes
uint8 MODE_MANUAL = 0               # Manual RC/joystick control
uint8 MODE_AUTO = 1                 # Autonomous VLA control
uint8 MODE_TRANSITION = 2           # Transitioning between modes
uint8 operation_mode                # Requested operation mode

# Mode switching control
bool mode_switch_request            # Request to switch modes
bool manual_override_active         # Manual override is currently active
bool emergency_mode_switch          # Emergency switch to manual mode

# Safety and validation
bool system_ready_for_auto          # System is ready for autonomous operation
bool vla_connected                  # VLA system is connected and responding
float32 transition_timeout          # Timeout for mode transitions (s)