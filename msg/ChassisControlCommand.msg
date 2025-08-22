# Chassis control command
# Published by chassis trajectory follower to wheel/steering modules

uint64 timestamp        # time since system start (microseconds)

# Control commands
float32 velocity         # Forward/backward velocity [m/s]
float32 steering_angle   # Steering angle [rad]

# Validity flag
bool valid              # Validity flag
