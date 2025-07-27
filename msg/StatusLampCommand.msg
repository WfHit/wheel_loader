# Status lamp command message

uint64 timestamp    # time since system start (microseconds)

uint8 STATE_OFF = 0
uint8 STATE_GREEN = 1
uint8 STATE_YELLOW = 2
uint8 STATE_RED = 3
uint8 STATE_BLUE = 4
uint8 STATE_WHITE = 5
uint8 STATE_PURPLE = 6
uint8 STATE_ORANGE = 7

uint8 target_state  # Target lamp state (0-7)
