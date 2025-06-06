# Module status for chassis control modules

uint64 timestamp             # Time since system start (microseconds)

uint8 module_id              # Module ID (0=front_wheel, 1=rear_wheel, 2=steering, etc.)
bool healthy                 # Overall module health status
bool initialized             # True if module is initialized
bool armed                   # True if module is armed and ready

float32 cpu_usage_percent    # CPU usage percentage [0, 100]
float32 memory_usage_bytes   # Memory usage in bytes
float32 temperature_c        # Module temperature in Celsius
float32 voltage_v            # Supply voltage in V
float32 current_draw_a       # Current draw in A

uint32 error_count           # Total error count since startup
uint32 warning_count         # Total warning count since startup
uint32 cycle_count           # Number of execution cycles
uint64 last_update_time      # Timestamp of last successful update

uint8 performance_level      # Performance level (0=degraded, 1=normal, 2=optimal)
float32 control_quality      # Control performance quality [0, 1]
float32 response_time_ms     # Average response time in milliseconds

bool emergency_stop_active   # True if emergency stop is active
uint8 failure_mode          # Failure mode (0=none, 1=sensor, 2=actuator, 3=communication, 4=other)
uint16 diagnostic_flags     # Diagnostic flags bitfield
