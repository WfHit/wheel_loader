# Operation mode selection command for dual-mode wheel loader control
# Supports switching between manual RC control and autonomous SmolVLA control

uint64 timestamp                    # microseconds since boot

# Operation modes
uint8 MODE_MANUAL = 0               # Manual RC/joystick control
uint8 MODE_AUTO = 1                 # Autonomous SmolVLA control
uint8 MODE_TRANSITION = 2           # Transitioning between modes
uint8 operation_mode                # Requested operation mode

# Mode switching control
bool mode_switch_request            # Request to switch modes
bool manual_override_active         # Manual override is currently active
bool emergency_mode_switch          # Emergency switch to manual mode

# Safety and validation
bool mode_switch_approved           # Mode switch has been approved by safety system
bool system_ready_for_auto          # System is ready for autonomous operation
bool smol_vla_connected             # SmolVLA system is connected and responding

# Manual control priority settings
bool allow_auto_to_manual           # Allow automatic fallback to manual
bool require_manual_approval        # Require manual approval for auto mode
float32 transition_timeout          # Timeout for mode transitions (s)

# Status feedback
uint8 current_active_mode           # Currently active operation mode
uint8 previous_mode                 # Previous operation mode
uint64 mode_change_time             # Time of last mode change (microseconds)