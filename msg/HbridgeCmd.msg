# H-Bridge motor command message

uint64 timestamp          # time since system start (microseconds)

uint8 channel             # 0 or 1 - which channel this command is for
float32 speed             # normalized speed (-1.0 to 1.0)
uint8 direction           # 0: forward, 1: reverse
bool enable_request       # channel enable request
uint8 control_mode        # 0: IDLE, 1: NORMAL, 2: BRAKE, 3: COAST

# Control parameters
float32 current_limit     # current limit in Amps (for future use)
uint32 timeout_ms         # command timeout
