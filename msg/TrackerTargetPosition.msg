# Antenna tracker target position received from MAVLink.
# Published by mavlink_receiver when GLOBAL_POSITION_INT is received from target UAV.

uint64 timestamp		# [us] time since system start

uint8 target_system		# MAVLink system ID of the target

bool valid			# true if target data is current and usable

int32 lat			# [degE7] Latitude
int32 lon			# [degE7] Longitude
int32 alt_mm			# [mm] Altitude MSL
int32 relative_alt_mm		# [mm] Altitude relative to home

float32 vx_m_s			# [m/s] Ground speed X (latitude direction)
float32 vy_m_s			# [m/s] Ground speed Y (longitude direction)
float32 vz_m_s			# [m/s] Ground speed Z (positive down)

uint64 last_update_us		# [us] Time of last update from MAVLink
