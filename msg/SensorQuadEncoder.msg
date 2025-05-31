# Quadrature encoder sensor data
# Generic topic for quadrature encoders (wheels, actuators, linear motors, etc.)

uint64 timestamp              # time since system start (microseconds)

uint8 MAX_ENCODERS = 8       # Maximum number of encoders supported

uint8 count                  # Number of valid encoder readings

# Encoder type definitions
uint8 TYPE_ROTARY = 0        # Rotary encoder (wheels, motors)
uint8 TYPE_LINEAR = 1        # Linear encoder (actuators, linear motors)

# Raw encoder data
int32[8] position            # Encoder position (pulses for rotary, mm for linear)
float32[8] velocity          # Velocity (rad/s for rotary, m/s for linear)
float32[8] angle_or_distance # Cumulative angle (rad) for rotary or distance (m) for linear
uint8[8] valid               # Validity flags for each encoder
uint8[8] encoder_type        # Encoder type (TYPE_ROTARY or TYPE_LINEAR)

# Configuration data
int32[8] pulses_per_rev      # Pulses per revolution (rotary) or pulses per mm (linear)
bool[8] invert_direction     # Direction inversion flags
float32[8] gear_ratio        # Gear ratio or screw pitch for linear actuators
