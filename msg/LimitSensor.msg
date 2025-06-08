uint64 timestamp                    # time since system start (microseconds)

uint8 instance                      # Instance ID of this limit sensor
uint8 type                         # Type of limit sensor (see LIMIT_TYPE_*)
uint8 function                     # Function of this limit sensor (see LimitFunction enum)
bool state                         # Combined state (true = limit reached)
bool state_switch_1                # Individual switch 1 state
bool state_switch_2                # Individual switch 2 state
bool redundancy_enabled            # Using redundant switches
bool redundancy_fault              # Mismatch between switches
bool healthy                       # Sensor is responding properly
uint32 activation_count            # Total number of activations
uint64 last_activation_time        # Last time limit was activated

# Limit sensor types
uint8 LIMIT_TYPE_BUCKET_DUMP = 0
uint8 LIMIT_TYPE_BUCKET_LOAD = 1
uint8 LIMIT_TYPE_BOOM_UPPER = 2
uint8 LIMIT_TYPE_BOOM_LOWER = 3
uint8 LIMIT_TYPE_STEERING_LEFT = 4
uint8 LIMIT_TYPE_STEERING_RIGHT = 5
uint8 LIMIT_TYPE_CUSTOM = 99
