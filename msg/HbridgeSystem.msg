# H-Bridge system coordination message

uint64 timestamp         # time since system start (microseconds)

bool enabled             # System enable state
bool emergency_stop      # Emergency stop active
float32 temperature      # Board temperature
bool safety_override     # Safety system allowing limit override

# Per-channel status
hbridge_channel[2] channel         # Status for each channel
