# Antenna tracker status for debugging and monitoring.
# Published by the antenna_tracker module.

uint64 timestamp		# [us] time since system start

bool target_valid		# true if a valid target is being tracked
uint8 target_system		# MAVLink system ID of the tracked target

float32 bearing_rad		# [rad] Compass bearing from tracker to target
float32 pitch_rad		# [rad] Elevation angle from tracker to target
float32 distance_m		# [m] Horizontal distance to target

float32 yaw_error_rad		# [rad] Yaw tracking error (bearing - tracker_yaw)
float32 pitch_error_rad		# [rad] Pitch tracking error

float32 yaw_output		# [-1,1] Normalized yaw servo output
float32 pitch_output		# [-1,1] Normalized pitch servo output

uint8 mode			# Current operating mode (0=STOP, 1=AUTO, 2=SCAN)

uint8 STATE_IDLE = 0
uint8 STATE_TRACKING = 1
uint8 STATE_TIMEOUT = 2
uint8 STATE_SERVO_TEST = 3
uint8 state			# Current tracker state
