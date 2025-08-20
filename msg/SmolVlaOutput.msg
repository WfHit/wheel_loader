# SmolVLA output message for wheel loader autonomous operation
# Contains bucket position, pose, and operation commands from SmolVLA algorithm

uint64 timestamp                    # microseconds since boot

# Bucket position and pose outputs from SmolVLA
float32 bucket_position_x           # Desired bucket position X (m)
float32 bucket_position_y           # Desired bucket position Y (m)  
float32 bucket_position_z           # Desired bucket position Z (m)
float32 bucket_orientation_roll     # Desired bucket roll angle (rad)
float32 bucket_orientation_pitch    # Desired bucket pitch angle (rad)
float32 bucket_orientation_yaw      # Desired bucket yaw angle (rad)

# Vehicle pose commands from SmolVLA
float32 vehicle_position_x          # Desired vehicle position X (m)
float32 vehicle_position_y          # Desired vehicle position Y (m)
float32 vehicle_heading             # Desired vehicle heading (rad)

# Operation mode from SmolVLA
uint8 OPERATION_IDLE = 0            # No active operation
uint8 OPERATION_APPROACH = 1        # Approaching material/target
uint8 OPERATION_LOAD = 2            # Loading material
uint8 OPERATION_TRANSPORT = 3       # Transporting material
uint8 OPERATION_DUMP = 4            # Dumping material
uint8 OPERATION_RETURN = 5          # Returning to start position
uint8 operation_mode                # Current operation mode

# Control confidence and validation
float32 confidence_score            # SmolVLA confidence in output (0-1)
bool valid_output                   # Whether output is valid for execution
bool emergency_stop                 # Emergency stop command from SmolVLA

# Sequence information
uint32 sequence_id                  # Sequence identifier for tracking
uint8 sequence_step                 # Current step in sequence
bool sequence_complete              # Whether current sequence is complete