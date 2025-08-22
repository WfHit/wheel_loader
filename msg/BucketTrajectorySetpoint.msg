# Bucket trajectory setpoint in WORLD coordinates
# Published by operation modes, subscribed by bucket trajectory follower

uint64 timestamp        # time since system start (microseconds)

# Bucket position in world frame
float32 x_position      # X position in world frame [m]
float32 y_position      # Y position in world frame [m]
float32 z_position      # Z position in world frame [m]

# Bucket orientation in world frame (quaternion)
float32 q_w             # Quaternion w component
float32 q_x             # Quaternion x component
float32 q_y             # Quaternion y component
float32 q_z             # Quaternion z component

# Bucket velocity in world frame
float32 x_velocity      # X velocity in world frame [m/s]
float32 y_velocity      # Y velocity in world frame [m/s]
float32 z_velocity      # Z velocity in world frame [m/s]

# Bucket angular velocity in world frame
float32 angular_x_velocity  # Angular velocity around X axis [rad/s]
float32 angular_y_velocity  # Angular velocity around Y axis [rad/s]
float32 angular_z_velocity  # Angular velocity around Z axis [rad/s]

# Validity flag
bool valid              # Validity flag
