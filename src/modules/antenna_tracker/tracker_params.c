/**
 * @file tracker_params.c
 *
 * Parameters for the antenna tracker module.
 *
 * @author PX4 Antenna Tracker
 */

#include <parameters/param.h>

/*==========================================================================
 * Mode & General
 *==========================================================================*/

/**
 * Tracker operating mode.
 *
 * 0 = STOP (servos move to configured park position)
 * 1 = AUTO (track target)
 * 2 = MANUAL (map manual input to physical axis angles)
 *
 * This is a tracker submode. It is independent of PX4 vehicle navigation
 * modes so estimator or Commander mode fallbacks cannot start manual control.
 *
 * @min 0
 * @max 2
 * @value 0 STOP
 * @value 1 AUTO
 * @value 2 MANUAL
 * @group Antenna Tracker
 */
PARAM_DEFINE_INT32(TRK_MODE, 0);

/**
 * Startup delay before servo movement.
 *
 * Servos are commanded to the configured park pose for this duration after boot.
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
 * Mechanical limits (degrees)
 *==========================================================================*/

/**
 * Physical yaw minimum angle.
 *
 * This defines the reachable yaw sector relative to the park position.
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
 * Only accept GLOBAL_POSITION_INT from this sysid. Set this parameter to the
 * UAV system ID before enabling AUTO tracking. A value of zero disables
 * MAVLink target acceptance.
 *
 * @min 0
 * @max 255
 * @group Antenna Tracker
 */
PARAM_DEFINE_INT32(TRK_SYSID_TARGET, 0);

/**
 * Reserved for future auto-lock target selection.
 *
 * The initial tracker implementation requires TRK_SYSID_TARGET to be set and
 * does not acquire targets automatically.
 *
 * @boolean
 * @group Antenna Tracker
 */
PARAM_DEFINE_INT32(TRK_AUTO_LOCK, 1);

/**
 * Target timeout.
 *
 * If no target update is received within this time,
 * target is marked invalid and servos go to the configured safe position.
 *
 * @min 1000
 * @max 30000
 * @unit ms
 * @group Antenna Tracker
 */
PARAM_DEFINE_INT32(TRK_TIMEOUT_MS, 5000);

/**
 * Park-command settle time before head-reference capture.
 *
 * On each transition to AUTO, the tracker first commands both axes to their
 * configured park angles. It waits this long after the normalized outputs
 * reach park before sampling the moving-IMU reference. This is a command
 * settle time, not proof of physical servo feedback; increase it for a slow
 * mechanism.
 *
 * @unit s
 * @min 0.1
 * @max 10.0
 * @decimal 1
 * @group Antenna Tracker
 */
PARAM_DEFINE_FLOAT(TRK_REF_SETTLE, 1.0f);
