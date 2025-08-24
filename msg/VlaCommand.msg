# VLA command message for wheel loader autonomous operation
# Contains bucket position and pose commands from VLA algorithm

uint64 timestamp                    # time since system start (microseconds)

# Bucket position and pose outputs from VLA
float32 bucket_position_x           # Desired bucket position X (m)
float32 bucket_position_y           # Desired bucket position Y (m)  
float32 bucket_position_z           # Desired bucket position Z (m)
float32 bucket_orientation_roll     # Desired bucket roll angle (rad)
float32 bucket_orientation_pitch    # Desired bucket pitch angle (rad)
float32 bucket_orientation_yaw      # Desired bucket yaw angle (rad)

# Control confidence and validation
float32 confidence_score            # VLA confidence in output [0.0-1.0]
bool valid_output                   # Whether output is valid for execution
bool emergency_stop                 # Emergency stop command from VLA

# Sequence information
uint32 sequence_id                  # Sequence identifier for tracking
uint8 sequence_step                 # Current step in sequence
bool sequence_complete              # Whether current sequence is complete