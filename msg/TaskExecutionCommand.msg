# Task execution command for autonomous wheel loader operations

uint64 timestamp                # time since system start (microseconds)

# Task identification
uint32 task_id                  # Unique task identifier
uint8 task_type                 # Type of task to execute
uint8 task_phase                # Current phase of the task
uint8 task_priority             # Task priority level

# Task types
uint8 TASK_LOADING = 0          # Loading material into bucket
uint8 TASK_TRANSPORT = 1        # Transporting material
uint8 TASK_DUMPING = 2          # Dumping material
uint8 TASK_POSITIONING = 3      # Positioning for operation
uint8 TASK_GRADING = 4          # Grading/leveling surface
uint8 TASK_DIGGING = 5          # Digging/excavating
uint8 TASK_IDLE = 6             # Idle/standby

# Task phases (generic)
uint8 PHASE_APPROACH = 0        # Approaching target
uint8 PHASE_ENGAGE = 1          # Engaging with material/target
uint8 PHASE_EXECUTE = 2         # Executing primary operation
uint8 PHASE_RETRACT = 3         # Retracting from operation
uint8 PHASE_COMPLETE = 4        # Task completed
uint8 PHASE_ABORT = 5           # Task aborted/failed

# Target information
float32 target_position_x       # Target X position (m)
float32 target_position_y       # Target Y position (m)
float32 target_position_z       # Target Z position (m)
float32 target_heading          # Target heading (rad)

# Operation parameters
float32 dig_depth              # Desired dig depth (m)
float32 load_weight_target     # Target load weight (kg)
float32 dump_height           # Dump height above ground (m)
float32 approach_speed        # Approach speed (m/s)
float32 operation_speed       # Operation speed (m/s)

# Quality and constraints
float32 position_tolerance     # Position tolerance (m)
float32 heading_tolerance      # Heading tolerance (rad)
float32 completion_threshold   # Task completion threshold (0-1)
bool precision_mode           # Enable precision mode
bool safety_override          # Safety override enabled

# Task state control
bool task_enable              # Enable task execution
bool task_pause               # Pause current task
bool task_abort               # Abort current task
bool manual_override          # Allow manual intervention

# Progress and feedback
float32 estimated_completion  # Estimated completion percentage (0-1)
float32 estimated_time_remaining # Estimated time to completion (s)
