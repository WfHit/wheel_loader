# H-Bridge system coordination message

uint64 timestamp         # time since system start (microseconds)

bool master_enable       # actual hardware enable pin state
bool emergency_stop      # system-wide emergency stop
float32 board_temp       # board temperature

uint8 ch0_request        # channel 0 enable request state
uint8 ch1_request        # channel 1 enable request state
uint8 ch0_state          # channel 0 state
uint8 ch1_state          # channel 1 state
