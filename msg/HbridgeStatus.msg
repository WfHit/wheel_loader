# H-Bridge motor status message

uint64 timestamp         # time since system start (microseconds)

uint8 channel           # 0 or 1 - which channel this status is from
float32 actual_duty     # actual PWM duty cycle (0-1.0)
bool fault              # fault status
uint8 fault_code        # specific fault code from DRV8701
uint8 state             # 0: DISABLED, 1: RUNNING, 2: FAULT, 3: BRAKE

# Runtime info
uint32 runtime_ms       # total runtime for this channel
