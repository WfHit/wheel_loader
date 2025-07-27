# Navigation lamp command message

uint64 timestamp    # time since system start (microseconds)

# Command types
uint8 CMD_SET_POWER = 0      # Control lamp system power
uint8 CMD_SET_STATE = 1      # Set state indicator lamp
uint8 CMD_SET_ILLUMINATION = 2  # Control illumination lamp

# State constants (used with CMD_SET_STATE)
uint8 STATE_OFF = 0
uint8 STATE_GREEN = 1
uint8 STATE_YELLOW = 2
uint8 STATE_RED = 3
uint8 STATE_BLUE = 4
uint8 STATE_WHITE = 5
uint8 STATE_PURPLE = 6
uint8 STATE_ORANGE = 7

# Power/Illumination constants (used with CMD_SET_POWER and CMD_SET_ILLUMINATION)
uint8 OPTION_OFF = 0
uint8 OPTION_ON = 1

uint8 command       # Command type (CMD_*)
uint8 option        # Command option (STATE_* for CMD_SET_STATE, OPTION_* for others)
