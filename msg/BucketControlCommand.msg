# Bucket control command
# Published by bucket trajectory follower to boom/bucket control modules

uint64 timestamp        # time since system start (microseconds)

# Target bucket position
float32 x_position      # Target bucket position X [m]
float32 y_position      # Target bucket position Y [m]
float32 z_position      # Target bucket position Z [m]

# Target bucket orientation (quaternion)
float32 q_w             # Quaternion w component
float32 q_x             # Quaternion x component
float32 q_y             # Quaternion y component
float32 q_z             # Quaternion z component

# Target bucket velocity
float32 x_velocity      # Target bucket velocity X [m/s]
float32 y_velocity      # Target bucket velocity Y [m/s]
float32 z_velocity      # Target bucket velocity Z [m/s]

# Target bucket angular velocity
float32 angular_x_velocity  # Target angular velocity around X axis [rad/s]
float32 angular_y_velocity  # Target angular velocity around Y axis [rad/s]
float32 angular_z_velocity  # Target angular velocity around Z axis [rad/s]

# Validity flag
bool valid              # Validity flag
