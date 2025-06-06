# Load-aware torque distribution

uint64 timestamp              # Time since system start (microseconds)

bool load_aware_active        # True if load-aware control is active
float32 bucket_load_kg        # Estimated bucket load in kg
float32 total_vehicle_mass_kg # Total vehicle mass including load
float32 center_of_gravity_x   # CoG x-position relative to rear axle
float32 center_of_gravity_z   # CoG z-position relative to ground

float32 front_axle_load_n     # Force on front axle in N
float32 rear_axle_load_n      # Force on rear axle in N
float32 weight_distribution   # Weight distribution factor [-1=rear, 1=front]

float32 optimal_torque_split  # Optimal torque split ratio [0=rear, 1=front]
float32 traction_limit_front  # Front wheel traction limit in Nm
float32 traction_limit_rear   # Rear wheel traction limit in Nm

float32 stability_margin      # Vehicle stability margin [0, 1]
bool stability_warning        # True if approaching stability limits
bool load_shift_detected      # True if significant load shift detected

uint8 load_estimation_mode    # Load estimation mode (0=static, 1=dynamic, 2=sensor)
float32 load_estimation_confidence # Confidence in load estimate [0, 1]
