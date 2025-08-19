# SmolVLA command interface for autonomous wheel loader control
# Receives bucket position and pose outputs from redesigned SmolVLA E2E algorithm

uint64 timestamp                    # microseconds since boot

# SmolVLA algorithm outputs
float32 bucket_position_x           # Target bucket position X (m)
float32 bucket_position_y           # Target bucket position Y (m)
float32 bucket_position_z           # Target bucket position Z (m)

# Bucket pose (orientation)
float32 bucket_roll                 # Bucket roll angle (rad)
float32 bucket_pitch                # Bucket pitch angle (rad)
float32 bucket_yaw                  # Bucket yaw angle (rad)

# Vehicle pose commands
float32 vehicle_position_x          # Target vehicle position X (m)
float32 vehicle_position_y          # Target vehicle position Y (m)
float32 vehicle_heading             # Target vehicle heading (rad)

# Operation mode and phase
uint8 OPERATION_IDLE = 0            # No active operation
uint8 OPERATION_LOAD = 1            # Auto load sequence
uint8 OPERATION_DUMP = 2            # Auto dump sequence
uint8 OPERATION_POSITION = 3        # Positioning only
uint8 operation_mode                # Current operation mode

uint8 PHASE_APPROACH = 0            # Approaching target
uint8 PHASE_ENGAGE = 1              # Engaging with material
uint8 PHASE_EXECUTE = 2             # Executing operation
uint8 PHASE_RETRACT = 3             # Retracting from operation
uint8 PHASE_COMPLETE = 4            # Operation complete
uint8 operation_phase               # Current operation phase

# Control parameters
float32 approach_speed              # Approach speed (m/s)
float32 operation_speed             # Operation execution speed (m/s)
float32 position_tolerance          # Position tolerance (m)
float32 orientation_tolerance       # Orientation tolerance (rad)

# Algorithm confidence and quality
float32 confidence_level            # SmolVLA output confidence (0-1)
float32 prediction_quality          # Prediction quality metric (0-1)
bool valid_prediction               # Whether prediction is valid
bool emergency_stop                 # Emergency stop from SmolVLA

# Sequence control
bool sequence_enable                # Enable autonomous sequence execution
bool manual_override_allowed        # Allow manual override during operation
uint32 sequence_id                  # Unique sequence identifier