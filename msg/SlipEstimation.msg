# Slip estimation results

uint64 timestamp               # Time since system start (microseconds)

float32 slip_ratio_front       # Front wheel slip ratio [-1, 1]
float32 slip_ratio_rear        # Rear wheel slip ratio [-1, 1]
float32 slip_angle_front_rad   # Front wheel slip angle in radians
float32 slip_angle_rear_rad    # Rear wheel slip angle in radians

float32 lateral_slip_velocity  # Lateral slip velocity in m/s
float32 longitudinal_slip_velocity # Longitudinal slip velocity in m/s

float32 friction_coefficient   # Estimated friction coefficient [0, 1]
float32 surface_roughness      # Estimated surface roughness [0, 1]

bool slip_detected             # True if significant slip detected
bool critical_slip             # True if slip above critical threshold
bool estimator_converged       # True if EKF has converged
bool sensor_fault              # True if sensor fault detected

float32[4] covariance_diagonal # EKF covariance diagonal elements
uint8 terrain_type             # Estimated terrain type (0=unknown, 1=asphalt, 2=gravel, 3=mud, etc.)
