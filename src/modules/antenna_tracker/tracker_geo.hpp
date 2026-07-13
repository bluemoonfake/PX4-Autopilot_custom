#pragma once

#include <cmath>
#include <cstdint>

namespace tracker_geo {

/** Earth radius in meters (WGS84 mean). */
static constexpr double EARTH_RADIUS_M = 6371000.0;

/** Location scaling factor: degrees-E7 to meters. */
static constexpr double LOCATION_SCALING_FACTOR = 0.011131884502145034;  // pi/180 * EARTH_RADIUS / 1e7

/**
 * Longitude scale factor based on latitude.
 *
 * Compensates for meridian convergence at higher latitudes.
 * @param latitude_rad  Latitude in radians
 * @return scale factor (0..1)
 */
inline double longitude_scale(double latitude_rad)
{
	return cos(latitude_rad);
}

/**
 * Compute horizontal distance between two points in meters.
 *
 * Uses flat-earth approximation with longitude scaling.
 *
 * @param tracker_lat_e7   Tracker latitude in degE7
 * @param tracker_lon_e7   Tracker longitude in degE7
 * @param target_lat_e7    Target latitude in degE7
 * @param target_lon_e7    Target longitude in degE7
 * @return horizontal distance in meters
 */
float horizontal_distance_m(int32_t tracker_lat_e7, int32_t tracker_lon_e7,
			     int32_t target_lat_e7, int32_t target_lon_e7);

/**
 * Compute bearing from tracker to target.
 *
 * Returns compass bearing: 0 = North, pi/2 = East, pi = South, etc.
 * Result is in range [0, 2*pi).
 *
 * @param tracker_lat_e7   Tracker latitude in degE7
 * @param tracker_lon_e7   Tracker longitude in degE7
 * @param target_lat_e7    Target latitude in degE7
 * @param target_lon_e7    Target longitude in degE7
 * @return bearing in radians [0, 2*pi)
 */
float bearing_rad(int32_t tracker_lat_e7, int32_t tracker_lon_e7,
		  int32_t target_lat_e7, int32_t target_lon_e7);

/**
 * Compute pitch/elevation angle from tracker to target.
 *
 * @param delta_alt_m        Altitude difference (target - tracker) in meters
 * @param horizontal_dist_m  Horizontal distance in meters
 * @return pitch angle in radians (positive = target above)
 */
float pitch_rad(float delta_alt_m, float horizontal_dist_m);

/**
 * Wrap angle to [-pi, pi].
 *
 * @param angle_rad  Input angle in radians
 * @return wrapped angle in [-pi, pi]
 */
float wrap_pi(float angle_rad);

/** Wrap an angle in degrees to [-180, 180]. */
float wrap_180_deg(float angle_deg);

} // namespace tracker_geo
