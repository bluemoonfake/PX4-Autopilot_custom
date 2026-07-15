# Antenna tracker status for debugging, verification, and logging.
# Published by the antenna_tracker module.

uint64 timestamp		# [us] Time since system start

bool target_valid		# True if a fresh, qualified target is available; tracking can still be inhibited by a safety state
uint8 target_system		# MAVLink system ID of the tracked target
uint32 target_age_ms		# [ms] Age of the accepted target update

float32 bearing_rad		# [rad] Compass bearing from tracker to target
float32 pitch_rad		# [rad] Elevation angle from tracker to target
float32 distance_m		# [m] Horizontal distance to target

float32 yaw_error_rad		# [rad] Desired head yaw minus measured tracker yaw
float32 pitch_error_rad		# [rad] Desired head pitch minus measured tracker pitch

float32 yaw_output		# [-1,1] Normalized yaw servo output
float32 pitch_output		# [-1,1] Normalized pitch servo output

uint8 mode			# 0=STOP, 1=AUTO, 2=SCAN, 3=MANUAL

uint8 STATE_IDLE = 0
uint8 STATE_TRACKING = 1
uint8 STATE_TIMEOUT = 2
uint8 STATE_SCANNING = 3
uint8 STATE_MANUAL = 4
uint8 STATE_SERVO_TEST = 5
uint8 STATE_SENSOR_INVALID = 6
uint8 STATE_TARGET_TOO_CLOSE = 7
uint8 STATE_RECONFIGURING = 8
uint8 state			# Current tracker state

uint8 REASON_NONE = 0
uint8 REASON_STOP = 1
uint8 REASON_STARTUP = 2
uint8 REASON_TARGET_TIMEOUT = 3
uint8 REASON_SENSOR_INVALID = 4
uint8 REASON_TARGET_TOO_CLOSE = 5
uint8 REASON_YAW_LIMIT = 6
uint8 REASON_PITCH_LIMIT = 7
uint8 REASON_SOURCE_REJECTED = 8
uint8 REASON_REFERENCE_NOT_READY = 9
uint8 REASON_HOME_UNPROVISIONED = 10
uint8 REASON_CONFIGURATION_CHANGED = 11
uint8 state_reason		# Reason for safe/clipped state

float32 scan_yaw_deg		# [deg] Current scan yaw physical angle
float32 scan_pitch_deg		# [deg] Current scan pitch physical angle

# Appended fields: preserve the original field order for existing consumers.
float32 yaw_setpoint_rad	# [rad] Desired world yaw after mechanical sector planning
float32 pitch_setpoint_rad	# [rad] Desired world pitch after mechanical limit planning
float32 yaw_measured_rad	# [rad] Measured vehicle attitude yaw
float32 pitch_measured_rad	# [rad] Measured vehicle attitude pitch
float32 yaw_command_deg		# [deg] Physical yaw command before normalized output mapping
float32 pitch_command_deg	# [deg] Physical pitch command before normalized output mapping
bool yaw_clipped		# True if yaw command was constrained by mechanics
bool pitch_clipped		# True if pitch command was constrained by mechanics

uint8 POSITION_SOURCE_NONE = 0
uint8 POSITION_SOURCE_GLOBAL = 1
uint8 POSITION_SOURCE_HOME = 2
uint8 position_source		# Own-position source used by tracker geometry
