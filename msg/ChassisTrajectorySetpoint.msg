# Chassis trajectory setpoint in WORLD coordinates
# Published by operation modes, subscribed by chassis trajectory follower

uint64 timestamp        # time since system start (microseconds)

# Position and heading in world frame
float32 x_position      # X position in world frame [m]
float32 y_position      # Y position in world frame [m]
float32 yaw             # Yaw angle in world frame [rad]

# Velocity in world frame
float32 x_velocity      # X velocity in world frame [m/s]
float32 y_velocity      # Y velocity in world frame [m/s]
float32 z_velocity      # Z velocity in world frame [m/s]
float32 yaw_rate        # Yaw rate [rad/s]

# Validity flag
bool valid              # Validity flag
