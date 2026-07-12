# Antenna tracker target position received from MAVLink.
# Published by mavlink_receiver when GLOBAL_POSITION_INT is accepted from target UAV.

uint64 timestamp		# [us] Time since system start

uint8 target_system		# MAVLink system ID of the target

bool valid			# True if target data is current and usable

int32 lat			# [degE7] Latitude
int32 lon			# [degE7] Longitude
int32 alt_mm			# [mm] Altitude MSL
int32 relative_alt_mm		# [mm] Altitude relative to target home

float32 vx_m_s			# [m/s] Ground speed north
float32 vy_m_s			# [m/s] Ground speed east
float32 vz_m_s			# [m/s] Ground speed down

uint64 last_update_us		# [us] Time of last accepted MAVLink update

# Appended source metadata. Keep original fields above in their existing order.
uint8 source_component		# MAVLink component ID that supplied the target
uint8 source_instance		# PX4 MAVLink instance that accepted the message
bool position_valid		# Latitude/longitude/altitude passed validation
bool velocity_valid		# Velocity fields are finite and usable for prediction
bool altitude_valid		# Absolute MSL altitude is valid
bool heartbeat_valid		# A fresh HEARTBEAT qualified this source
uint8 source_type		# MAV_TYPE reported by the qualifying HEARTBEAT
