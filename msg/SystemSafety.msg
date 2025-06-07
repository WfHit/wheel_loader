# System-level safety status message
# Published by the system-level Safety Manager

uint64 timestamp                    # Time since system start (microseconds)

# Overall safety status
uint8 SAFETY_LEVEL_NORMAL = 0
uint8 SAFETY_LEVEL_CAUTION = 1
uint8 SAFETY_LEVEL_WARNING = 2
uint8 SAFETY_LEVEL_CRITICAL = 3
uint8 SAFETY_LEVEL_EMERGENCY = 4

# Safety modes
uint8 SAFETY_MODE_NORMAL = 0
uint8 SAFETY_MODE_MANUAL_OVERRIDE = 1
uint8 SAFETY_MODE_ZEROING = 2
uint8 SAFETY_MODE_EMERGENCY = 3

uint8 safety_level                  # Current system safety level
uint8 safety_mode                   # Current safety mode
float32 overall_risk_factor         # Overall system risk factor [0, 1]
bool emergency_active               # Emergency response active
bool safety_override_active         # Safety override active

# System fault status
uint32 active_faults                # Bitmask of active faults
uint32 fault_history               # History of occurred faults
uint64 last_fault_time             # Timestamp of last fault
uint32 safety_violation_count      # Total safety violations

# Safety permits
bool motion_permitted              # Vehicle motion allowed
bool steering_permitted            # Steering operations allowed
bool autonomous_permitted          # Autonomous operation allowed
bool electric_actuator_permitted   # Electric actuator operations allowed
bool boom_operation_permitted      # Boom operations allowed
bool bucket_operation_permitted    # Bucket operations allowed
bool articulation_permitted        # Articulation operations allowed
bool engine_start_permitted        # Engine start allowed
bool emergency_override_active     # Emergency override active

# System monitoring status
bool chassis_safe                  # Chassis systems safe
bool hydraulic_safe               # Electric actuator systems safe (legacy field name)
bool sensor_safe                  # Sensor systems safe
bool communication_safe           # Communication systems safe
bool power_safe                   # Power systems safe
bool thermal_safe                 # Thermal systems safe

# Emergency response
bool emergency_stop_commanded      # Emergency stop active
bool controlled_stop_active       # Controlled stop in progress
bool emergency_shutdown_active    # Emergency shutdown active
bool backup_systems_active        # Backup systems engaged

# Performance metrics
float32 safety_check_frequency_hz  # Safety check rate
float32 intervention_response_time_ms # Response time to interventions
float32 safety_system_availability   # Safety system availability [0, 1]
uint32 total_safety_checks         # Total safety checks performed
uint32 safety_interventions        # Total safety interventions
