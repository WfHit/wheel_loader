# Operation mode status feedback for dual-mode wheel loader control
# Provides status information about current operation mode

uint64 timestamp                    # time since system start (microseconds)

# Current status
uint8 MODE_MANUAL = 0               # Manual RC/joystick control
uint8 MODE_AUTO = 1                 # Autonomous VLA control  
uint8 MODE_TRANSITION = 2           # Transitioning between modes
uint8 current_active_mode           # Currently active operation mode
uint8 previous_mode                 # Previous operation mode
uint64 mode_change_time             # Time of last mode change (microseconds)

# Status flags
bool mode_switch_approved           # Mode switch has been approved by safety system
bool allow_auto_to_manual           # Allow automatic fallback to manual
bool require_manual_approval        # Require manual approval for auto mode
bool transition_in_progress         # Mode transition is currently in progress
bool transition_timeout             # Mode transition has timed out