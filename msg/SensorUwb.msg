# UWB sensor data with enhanced diagnostics for indoor positioning

uint64 timestamp          # time since system start (microseconds)
uint32 sessionid          # UWB session id
uint32 time_offset        # Time offset in microseconds for batch measurements
uint8 anchor_id           # Anchor ID
uint8 tag_id              # Tag ID

float32 range             # Distance in meters

# Signal quality metrics
int8 rssi                 # Received Signal Strength Indicator (dBm)
uint8 los_confidence      # Line-of-sight confidence (0-100)
uint16 first_path_power   # First path signal amplitude
uint16 total_path_power   # Total received power

# Anchor position (if known)
float32 anchor_x          # Anchor X position in meters
float32 anchor_y          # Anchor Y position in meters
float32 anchor_z          # Anchor Z position in meters
bool anchor_pos_valid     # True if anchor position is valid

# Additional diagnostics
uint8 multipath_count     # Number of detected multipaths
float32 range_bias        # Estimated range bias due to NLOS

# Figure of merit for the angle measurements
uint8 aoa_azimuth_fom		# AOA Azimuth FOM
uint8 aoa_elevation_fom		# AOA Elevation FOM
uint8 aoa_dest_azimuth_fom	# AOA Azimuth FOM
uint8 aoa_dest_elevation_fom	# AOA Elevation FOM

# Initiator physical configuration
uint8 orientation		# Direction the sensor faces from MAV_SENSOR_ORIENTATION enum
				# Standard configuration is Antennas facing down and azimuth aligened in forward direction
float32 offset_x		# UWB initiator offset in X axis (NED drone frame)
float32 offset_y		# UWB initiator offset in Y axis (NED drone frame)
float32 offset_z		# UWB initiator offset in Z axis (NED drone frame)
