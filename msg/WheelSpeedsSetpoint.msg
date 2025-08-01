# Wheel speed setpoints for chassis control

uint64 timestamp                # Time since system start (microseconds)

float32 wheel_speed_rad_s       # Wheel speed setpoint in rad/s
float32 max_acceleration        # Maximum allowed acceleration in rad/s²
float32 torque_limit            # Torque limit in Nm (0 = no limit)

bool synchronized               # True if front and rear wheels should be synchronized
bool differential_enable        # True if differential speed control is enabled
