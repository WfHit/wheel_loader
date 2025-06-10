uint64 timestamp          # time since system start (microseconds)
uint8 channel             # H-bridge channel (0 or 1)
float32 duty_cycle        # PWM duty cycle (-1.0 to 1.0, negative for reverse)
bool enable               # Enable/disable the H-bridge

uint8 ORB_QUEUE_LENGTH = 4
