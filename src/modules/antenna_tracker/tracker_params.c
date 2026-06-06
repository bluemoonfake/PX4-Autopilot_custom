/**
 * @file tracker_params.c
 *
 * Parameters for the antenna tracker module.
 *
 * @author PX4 Antenna Tracker
 */

/**
 * Tracker operating mode.
 *
 * 0 = STOP (servos centered, no tracking)
 * 1 = AUTO (track target)
 * 2 = SCAN (future: scan pattern)
 *
 * @min 0
 * @max 2
 * @value 0 STOP
 * @value 1 AUTO
 * @value 2 SCAN
 * @group Antenna Tracker
 */
PARAM_DEFINE_INT32(TRK_MODE, 0);

/**
 * Enable servo test sweep.
 *
 * When enabled, the tracker runs a slow servo sweep
 * instead of tracking. Useful for verifying servo direction.
 *
 * 0 = disabled, 1 = enabled
 *
 * @boolean
 * @group Antenna Tracker
 */
PARAM_DEFINE_INT32(TRK_SERVO_TEST, 0);

/**
 * Yaw PID proportional gain.
 *
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.1
 * @group Antenna Tracker
 */
PARAM_DEFINE_FLOAT(TRK_YAW_P, 1.0f);

/**
 * Yaw PID integral gain.
 *
 * @min 0.0
 * @max 5.0
 * @decimal 3
 * @increment 0.01
 * @group Antenna Tracker
 */
PARAM_DEFINE_FLOAT(TRK_YAW_I, 0.0f);

/**
 * Yaw PID derivative gain.
 *
 * @min 0.0
 * @max 5.0
 * @decimal 3
 * @increment 0.01
 * @group Antenna Tracker
 */
PARAM_DEFINE_FLOAT(TRK_YAW_D, 0.0f);

/**
 * Pitch PID proportional gain.
 *
 * @min 0.0
 * @max 10.0
 * @decimal 2
 * @increment 0.1
 * @group Antenna Tracker
 */
PARAM_DEFINE_FLOAT(TRK_PIT_P, 1.0f);

/**
 * Pitch PID integral gain.
 *
 * @min 0.0
 * @max 5.0
 * @decimal 3
 * @increment 0.01
 * @group Antenna Tracker
 */
PARAM_DEFINE_FLOAT(TRK_PIT_I, 0.0f);

/**
 * Pitch PID derivative gain.
 *
 * @min 0.0
 * @max 5.0
 * @decimal 3
 * @increment 0.01
 * @group Antenna Tracker
 */
PARAM_DEFINE_FLOAT(TRK_PIT_D, 0.0f);

/**
 * Yaw trim offset.
 *
 * Added to yaw servo output to compensate for mounting offset.
 *
 * @min -1.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Antenna Tracker
 */
PARAM_DEFINE_FLOAT(TRK_YAW_TRIM, 0.0f);

/**
 * Pitch trim offset.
 *
 * Added to pitch servo output to compensate for mounting offset.
 *
 * @min -1.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Antenna Tracker
 */
PARAM_DEFINE_FLOAT(TRK_PIT_TRIM, 0.0f);

/**
 * Yaw output minimum limit.
 *
 * Normalized servo output lower bound for yaw.
 *
 * @min -1.0
 * @max 0.0
 * @decimal 2
 * @group Antenna Tracker
 */
PARAM_DEFINE_FLOAT(TRK_YAW_MIN, -1.0f);

/**
 * Yaw output maximum limit.
 *
 * Normalized servo output upper bound for yaw.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group Antenna Tracker
 */
PARAM_DEFINE_FLOAT(TRK_YAW_MAX, 1.0f);

/**
 * Pitch output minimum limit.
 *
 * Normalized servo output lower bound for pitch.
 *
 * @min -1.0
 * @max 0.0
 * @decimal 2
 * @group Antenna Tracker
 */
PARAM_DEFINE_FLOAT(TRK_PIT_MIN, -1.0f);

/**
 * Pitch output maximum limit.
 *
 * Normalized servo output upper bound for pitch.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group Antenna Tracker
 */
PARAM_DEFINE_FLOAT(TRK_PIT_MAX, 1.0f);

/**
 * Fake target latitude.
 *
 * Latitude in degrees * 1E7 for testing without MAVLink target.
 * Set to 0 to disable fake target.
 *
 * @group Antenna Tracker
 */
PARAM_DEFINE_INT32(TRK_TGT_LAT, 0);

/**
 * Fake target longitude.
 *
 * Longitude in degrees * 1E7 for testing without MAVLink target.
 *
 * @group Antenna Tracker
 */
PARAM_DEFINE_INT32(TRK_TGT_LON, 0);

/**
 * Fake target altitude.
 *
 * Altitude in millimeters MSL for testing without MAVLink target.
 *
 * @group Antenna Tracker
 */
PARAM_DEFINE_INT32(TRK_TGT_ALT, 0);

/**
 * MAVLink target system ID.
 *
 * If nonzero, only accept GLOBAL_POSITION_INT from this sysid.
 * If zero and TRK_AUTO_LOCK is enabled, lock first valid vehicle.
 *
 * @min 0
 * @max 255
 * @group Antenna Tracker
 */
PARAM_DEFINE_INT32(TRK_SYSID_TGT, 0);

/**
 * Auto-lock target sysid.
 *
 * When target sysid is 0, automatically lock the first
 * vehicle sysid that sends GLOBAL_POSITION_INT.
 *
 * @boolean
 * @group Antenna Tracker
 */
PARAM_DEFINE_INT32(TRK_AUTO_LOCK, 1);

/**
 * Target timeout.
 *
 * If no target update is received within this time,
 * target is marked invalid and servos go to safe position.
 *
 * @min 1000
 * @max 30000
 * @unit ms
 * @group Antenna Tracker
 */
PARAM_DEFINE_INT32(TRK_TIMEOUT_MS, 5000);
