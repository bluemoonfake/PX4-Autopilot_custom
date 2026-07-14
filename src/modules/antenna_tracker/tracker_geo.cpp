#include "tracker_geo.hpp"
#include <math.h>

namespace tracker_geo {

float horizontal_distance_m(int32_t tracker_lat_e7, int32_t tracker_lon_e7,
			     int32_t target_lat_e7, int32_t target_lon_e7)
{
	const double tracker_lat_rad = static_cast<double>(tracker_lat_e7) * 1.0e-7 * M_PI / 180.0;

	const double dlat = static_cast<double>(target_lat_e7 - tracker_lat_e7);
	const double dlon = static_cast<double>(target_lon_e7 - tracker_lon_e7);
	const double dlon_scaled = dlon * longitude_scale(tracker_lat_rad);

	const double dist = sqrt(dlat * dlat + dlon_scaled * dlon_scaled) * LOCATION_SCALING_FACTOR;

	return static_cast<float>(dist);
}

float bearing_rad(int32_t tracker_lat_e7, int32_t tracker_lon_e7,
		  int32_t target_lat_e7, int32_t target_lon_e7)
{
	const double tracker_lat_rad = static_cast<double>(tracker_lat_e7) * 1.0e-7 * M_PI / 180.0;

	const double dlat = static_cast<double>(target_lat_e7 - tracker_lat_e7);  // north offset
	const double dlon = static_cast<double>(target_lon_e7 - tracker_lon_e7);
	const double dlon_scaled = dlon * longitude_scale(tracker_lat_rad);        // east offset

	// atan2(east, north) gives compass bearing directly: 0=N, pi/2=E
	float brg = static_cast<float>(atan2(dlon_scaled, dlat));

	// Normalize to [0, 2*pi)
	if (brg < 0.f) {
		brg += 2.f * static_cast<float>(M_PI);
	}

	return brg;
}

float pitch_rad(float delta_alt_m, float horizontal_dist_m)
{
	// Use atan2 for safety when distance is very small
	return atan2f(delta_alt_m, fmaxf(horizontal_dist_m, 0.001f));
}

float wrap_pi(float angle_rad)
{
	// Fast wrap to [-pi, pi]
	while (angle_rad > static_cast<float>(M_PI)) {
		angle_rad -= 2.f * static_cast<float>(M_PI);
	}

	while (angle_rad < -static_cast<float>(M_PI)) {
		angle_rad += 2.f * static_cast<float>(M_PI);
	}

	return angle_rad;
}

float wrap_180_deg(float angle_deg)
{
	while (angle_deg > 180.f) {
		angle_deg -= 360.f;
	}

	while (angle_deg < -180.f) {
		angle_deg += 360.f;
	}

	return angle_deg;
}

bool valid_home_fallback(int32_t latitude_e7, int32_t longitude_e7)
{
	return latitude_e7 >= -900000000 && latitude_e7 <= 900000000
	       && longitude_e7 >= -1800000000 && longitude_e7 <= 1800000000
	       && (latitude_e7 != 0 || longitude_e7 != 0);
}

} // namespace tracker_geo
