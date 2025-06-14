# Traction control system status and commands

uint64 timestamp                # Time since system start (microseconds)

bool traction_control_active    # True if traction control is active
bool slip_detected              # True if wheel slip is detected
bool intervention_active        # True if actively controlling traction

float32 slip_ratio_front        # Front wheel slip ratio [-1, 1]
float32 slip_ratio_rear         # Rear wheel slip ratio [-1, 1]
float32 target_slip_ratio       # Target slip ratio for optimal traction

float32 torque_reduction_front  # Torque reduction factor for front wheels [0, 1]
float32 torque_reduction_rear   # Torque reduction factor for rear wheels [0, 1]
float32 torque_distribution     # Torque distribution bias [-1=rear, 1=front]

float32 surface_friction        # Estimated surface friction coefficient
uint8 traction_mode             # Traction control mode (0=off, 1=eco, 2=normal, 3=aggressive)
uint8 intervention_level        # Level of intervention (0=none, 1=mild, 2=moderate, 3=aggressive)

float32 steering_correction_rad # Steering correction for traction control (radians)

bool abs_active                 # Anti-lock braking active
bool stability_control_active   # Electronic stability control active
uint32 intervention_count       # Number of interventions since startup
