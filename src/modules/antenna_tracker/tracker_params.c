/**
 * @file tracker_params.c
 *
 * Parameters for the antenna tracker module.
 *
 * @author PX4 Antenna Tracker
 */

/*==========================================================================
 * Mode & General
 *==========================================================================*/

/**
 * Tracker operating mode.
 *
 * 0 = STOP (servos move to configured park position)
 * 1 = AUTO (track target)
 * 2 = SCAN (sweep only within configured mechanical limits)
 * 3 = MANUAL (map manual input to physical axis angles)
 *
 * This is a tracker submode. It is independent of PX4 vehicle navigation
 * modes so estimator or Commander mode fallbacks cannot start scanning.
 *
 * @min 0
 * @max 3
 * @value 0 STOP
 * @value 1 AUTO
 * @value 2 SCAN
 * @value 3 MANUAL
 * @group Antenna Tracker
 */
PARAM_DEFINE_INT32(TRK_MODE, 0);

/**
 * Deprecated QGroundControl navigation-mode mapping option.
 *
 * Retained only so existing parameter files can be loaded. The tracker always
 * uses TRK_MODE because PX4 navigation modes are not tracker submodes.
 *
 * @boolean
 * @group Antenna Tracker
 */
PARAM_DEFINE_INT32(TRK_QGC_MODE, 0);

/**
 * Startup delay before servo movement.
 *
 * Servos held at trim for this duration after boot.
 * Useful for some servo types that need settling time.
 *
 * @unit s
 * @min 0
 * @max 10
 * @decimal 1
 * @group Antenna Tracker
 */
PARAM_DEFINE_FLOAT(TRK_STRT_DLY, 0.0f);

/*==========================================================================
 * PID Gains — Yaw axis
 *==========================================================================*/

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
 * Yaw feed-forward gain.
 *
 * Proportional to the target bearing angular rate. Reduces tracking lag
 * when the target moves quickly across the sky.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.01
 * @group Antenna Tracker
 */
PARAM_DEFINE_FLOAT(TRK_YAW_FF, 0.0f);

/**
 * Yaw integrator maximum.
 *
 * Limits the integrator accumulation to prevent windup.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group Antenna Tracker
 */
PARAM_DEFINE_FLOAT(TRK_YAW_IMAX, 0.5f);

/*==========================================================================
 * PID Gains — Pitch axis
 *==========================================================================*/

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
 * Pitch feed-forward gain.
 *
 * Proportional to the target elevation angular rate. Reduces tracking lag
 * when the target climbs or descends quickly.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 3
 * @increment 0.01
 * @group Antenna Tracker
 */
PARAM_DEFINE_FLOAT(TRK_PIT_FF, 0.0f);

/**
 * Pitch integrator maximum.
 *
 * Limits the integrator accumulation to prevent windup.
 *
 * @min 0.0
 * @max 1.0
 * @decimal 2
 * @group Antenna Tracker
 */
PARAM_DEFINE_FLOAT(TRK_PIT_IMAX, 0.5f);

/*==========================================================================
 * Servo trim / output limits
 *==========================================================================*/

/**
 * Yaw servo trim offset (mechanical).
 *
 * Added to yaw servo output AFTER PID control to compensate for
 * servo horn misalignment or mechanical neutral offset.
 *
 * NOTE: This is NOT for flight controller orientation correction.
 * If the FC is mounted rotated/backwards/upside-down, use
 * SENS_BOARD_ROT and SENS_BOARD_Z_OFF in QGC Sensors page instead.
 * Only adjust this parameter after SENS_BOARD_ROT is correctly set
 * and attitude readings are verified correct via 'antenna_tracker status'.
 *
 * @min -1.0
 * @max 1.0
 * @decimal 2
 * @increment 0.01
 * @group Antenna Tracker
 */
PARAM_DEFINE_FLOAT(TRK_YAW_TRIM, 0.0f);

/**
 * Pitch servo trim offset (mechanical).
 *
 * Added to pitch servo output AFTER PID control to compensate for
 * servo horn misalignment or mechanical neutral offset.
 *
 * NOTE: This is NOT for flight controller orientation correction.
 * If the FC is mounted with a pitch offset, use SENS_BOARD_ROT and
 * SENS_BOARD_Y_OFF (level horizon calibration) in QGC instead.
 * Only adjust this parameter after attitude readings are verified
 * correct via 'antenna_tracker status'.
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

/*==========================================================================
 * Mechanical limits (degrees)
 *==========================================================================*/

/**
 * Yaw range in degrees.
 *
 * Total mechanical range the yaw axis can sweep.
 * The tracker operates from -range/2 to +range/2 relative to center.
 *
 * @unit deg
 * @min 0
 * @max 360
 * @decimal 0
 * @group Antenna Tracker
 */
PARAM_DEFINE_FLOAT(TRK_YAW_RANGE, 360.0f);

/**
 * Physical yaw minimum angle.
 *
 * This defines the reachable yaw sector relative to the park position.
 * It supersedes the symmetric TRK_YAW_RANGE assumption when TRK_YAW_MIND is
 * less than TRK_YAW_MAXD.
 *
 * @unit deg
 * @min -360
 * @max 360
 * @decimal 1
 * @group Antenna Tracker
 */
PARAM_DEFINE_FLOAT(TRK_YAW_MIND, -180.0f);

/**
 * Physical yaw maximum angle.
 *
 * @unit deg
 * @min -360
 * @max 360
 * @decimal 1
 * @group Antenna Tracker
 */
PARAM_DEFINE_FLOAT(TRK_YAW_MAXD, 180.0f);

/**
 * Physical yaw park angle.
 *
 * STOP, timeout, invalid sensor, and startup states command this angle.
 *
 * @unit deg
 * @min -360
 * @max 360
 * @decimal 1
 * @group Antenna Tracker
 */
PARAM_DEFINE_FLOAT(TRK_YAW_PARK, 0.0f);

/**
 * Reverse yaw servo angle-to-output mapping.
 *
 * @boolean
 * @group Antenna Tracker
 */
PARAM_DEFINE_INT32(TRK_YAW_REV, 0);

/**
 * Pitch minimum angle.
 *
 * The lowest angle the pitch axis can reach. 0 = horizontal, -90 = straight down.
 *
 * @unit deg
 * @min -90
 * @max 0
 * @decimal 0
 * @group Antenna Tracker
 */
PARAM_DEFINE_FLOAT(TRK_PIT_MIND, 0.0f);

/**
 * Pitch maximum angle.
 *
 * The highest angle the pitch axis can reach. 90 = straight up.
 *
 * @unit deg
 * @min 0
 * @max 90
 * @decimal 0
 * @group Antenna Tracker
 */
PARAM_DEFINE_FLOAT(TRK_PIT_MAXD, 90.0f);

/**
 * Physical pitch park angle.
 *
 * STOP, timeout, invalid sensor, and startup states command this angle.
 *
 * @unit deg
 * @min -90
 * @max 90
 * @decimal 1
 * @group Antenna Tracker
 */
PARAM_DEFINE_FLOAT(TRK_PIT_PARK, 0.0f);

/**
 * Reverse pitch servo angle-to-output mapping.
 *
 * @boolean
 * @group Antenna Tracker
 */
PARAM_DEFINE_INT32(TRK_PIT_REV, 0);

/*==========================================================================
 * Slew rate limiting
 *==========================================================================*/

/**
 * Yaw slew time.
 *
 * Time in seconds for yaw servo to traverse its full range.
 * Used to limit servo speed and prevent mechanical stress.
 * 0 = no limit.
 *
 * @unit s
 * @min 0.0
 * @max 20.0
 * @decimal 1
 * @increment 0.5
 * @group Antenna Tracker
 */
PARAM_DEFINE_FLOAT(TRK_YAW_SLEW, 2.0f);

/**
 * Pitch slew time.
 *
 * Time in seconds for pitch servo to traverse its full range.
 * Used to limit servo speed and prevent mechanical stress.
 * 0 = no limit.
 *
 * @unit s
 * @min 0.0
 * @max 20.0
 * @decimal 1
 * @increment 0.5
 * @group Antenna Tracker
 */
PARAM_DEFINE_FLOAT(TRK_PIT_SLEW, 2.0f);

/*==========================================================================
 * Distance & Altitude
 *==========================================================================*/

/**
 * Minimum tracking distance.
 *
 * Tracker will only track targets at least this distance away.
 * Prevents erratic behavior when target is very close.
 *
 * @unit m
 * @min 0
 * @max 100
 * @decimal 0
 * @group Antenna Tracker
 */
PARAM_DEFINE_FLOAT(TRK_DIST_MIN, 5.0f);

/**
 * Altitude source.
 *
 * Selects the source for altitude difference calculation.
 *
 * 0 = GPS MSL altitude. This is the only production-supported choice because
 * GLOBAL_POSITION_INT.relative_alt is relative to the target vehicle home,
 * which is not necessarily the tracker home.
 *
 * @min 0
 * @max 1
 * @value 0 GPS_MSL
 * @value 1 RESERVED_UNVERIFIED
 * @group Antenna Tracker
 */
PARAM_DEFINE_INT32(TRK_ALT_SRC, 0);

/*==========================================================================
 * SCAN mode
 *==========================================================================*/

/**
 * Scan yaw speed.
 *
 * Speed of yaw sweep in SCAN mode.
 *
 * @unit deg/s
 * @min 0
 * @max 100
 * @decimal 1
 * @increment 1
 * @group Antenna Tracker
 */
PARAM_DEFINE_FLOAT(TRK_SCAN_YSPD, 2.0f);

/**
 * Scan pitch speed.
 *
 * Speed of pitch sweep in SCAN mode.
 *
 * @unit deg/s
 * @min 0
 * @max 100
 * @decimal 1
 * @increment 1
 * @group Antenna Tracker
 */
PARAM_DEFINE_FLOAT(TRK_SCAN_PSPD, 5.0f);

/**
 * Auto scan on target loss.
 *
 * When in AUTO mode and target is lost (timeout), automatically
 * switch to SCAN behavior to search for the target.
 * When target is reacquired, resume AUTO tracking.
 *
 * @boolean
 * @group Antenna Tracker
 */
PARAM_DEFINE_INT32(TRK_AUTO_SCAN, 0);

/*==========================================================================
 * Dead reckoning
 *==========================================================================*/

/**
 * Enable dead reckoning.
 *
 * When enabled, uses target velocity (vx/vy/vz) from
 * GLOBAL_POSITION_INT to extrapolate target position between
 * MAVLink updates. Improves tracking smoothness.
 *
 * @boolean
 * @group Antenna Tracker
 */
PARAM_DEFINE_INT32(TRK_DEADRECK, 0);

/*==========================================================================
 * Fake target for testing
 *==========================================================================*/

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

/*==========================================================================
 * Tracker home fallback
 *==========================================================================*/

/**
 * Enable tracker home fallback.
 *
 * If enabled and live vehicle_global_position is unavailable,
 * the tracker uses TRK_HOME_LAT/LON/ALT as its own position.
 * This is intended for bench testing or fixed tripod setups. The default
 * coordinate (latitude=0 and longitude=0) is an unprovisioned placeholder and
 * is rejected; the tracker remains parked until a real home is supplied.
 *
 * 0 = disabled, 1 = enabled
 *
 * @boolean
 * @group Antenna Tracker
 */
PARAM_DEFINE_INT32(TRK_HOME_EN, 0);

/**
 * Tracker home latitude.
 *
 * Latitude in degrees * 1E7. Used only when TRK_HOME_EN is enabled
 * and live tracker global position is unavailable. Together with TRK_HOME_LON,
 * this must not be the default (0,0) placeholder.
 *
 * @group Antenna Tracker
 */
PARAM_DEFINE_INT32(TRK_HOME_LAT, 0);

/**
 * Tracker home longitude.
 *
 * Longitude in degrees * 1E7. Used only when TRK_HOME_EN is enabled
 * and live tracker global position is unavailable. Together with TRK_HOME_LAT,
 * this must not be the default (0,0) placeholder.
 *
 * @group Antenna Tracker
 */
PARAM_DEFINE_INT32(TRK_HOME_LON, 0);

/**
 * Tracker home altitude.
 *
 * Altitude in millimeters MSL. Used only when TRK_HOME_EN is enabled
 * and live tracker global position is unavailable.
 *
 * @unit mm
 * @group Antenna Tracker
 */
PARAM_DEFINE_INT32(TRK_HOME_ALT, 0);

/*==========================================================================
 * MAVLink target filtering
 *==========================================================================*/

/**
 * MAVLink target system ID.
 *
 * If nonzero, only accept GLOBAL_POSITION_INT from this sysid.
 * If zero and TRK_AUTO_LOCK is enabled, lock the first valid vehicle.
 * In both cases the source must be MAV_COMP_ID_AUTOPILOT1 and must have sent
 * a fresh non-GCS HEARTBEAT from the same system/component pair.
 *
 * @min 0
 * @max 255
 * @group Antenna Tracker
 */
PARAM_DEFINE_INT32(TRK_SYSID_TGT, 0);

/**
 * Auto-lock target sysid.
 *
 * When target sysid is 0, automatically lock the first source that passes
 * the component, HEARTBEAT, position, and altitude checks. The lock is
 * released after TRK_TIMEOUT_MS without an accepted position update.
 *
 * @boolean
 * @group Antenna Tracker
 */
PARAM_DEFINE_INT32(TRK_AUTO_LOCK, 1);

/**
 * Target timeout.
 *
 * If no target update is received within this time,
 * target is marked invalid and servos go to safe position
 * or SCAN mode if TRK_AUTO_SCAN is enabled.
 *
 * @min 1000
 * @max 30000
 * @unit ms
 * @group Antenna Tracker
 */
PARAM_DEFINE_INT32(TRK_TIMEOUT_MS, 5000);

/**
 * Enable servo test sweep.
 *
 * When enabled, the tracker runs a slow servo sweep
 * instead of tracking. Useful for verifying servo direction.
 * Can also use QGC Actuator Test panel instead.
 *
 * 0 = disabled, 1 = enabled
 *
 * @boolean
 * @group Antenna Tracker
 */
PARAM_DEFINE_INT32(TRK_SERVO_TEST, 0);
