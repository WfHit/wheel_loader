# Terrain adaptation system

uint64 timestamp              # Time since system start (microseconds)

bool terrain_adaptation_active # True if terrain adaptation is active
uint8 terrain_type            # Detected terrain type (0=unknown, 1=asphalt, 2=gravel, 3=mud, 4=sand, 5=snow, 6=ice)
float32 terrain_confidence    # Confidence in terrain classification [0, 1]

float32 surface_roughness     # Surface roughness estimate [0, 1]
float32 friction_coefficient  # Surface friction coefficient [0, 1]
float32 penetration_resistance # Terrain penetration resistance [0, 1]
float32 slope_angle_rad       # Terrain slope angle in radians

float32 adapted_max_speed     # Adapted maximum speed for terrain
float32 adapted_acceleration  # Adapted acceleration limit
float32 adapted_traction_mode # Adapted traction control aggressiveness

bool learning_mode_active     # True if terrain learning is active
float32 learning_confidence   # Confidence in learned parameters
uint32 terrain_transitions    # Number of terrain transitions detected

float32[8] terrain_features   # Terrain feature vector for classification
float32 banking_angle_rad     # Estimated terrain banking in radians

bool hazard_detected          # True if terrain hazard detected
uint8 hazard_type            # Hazard type (0=none, 1=slope, 2=soft, 3=ice, 4=obstacle)
float32 hazard_severity       # Hazard severity [0, 1]
