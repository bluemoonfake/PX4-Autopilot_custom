#include <gtest/gtest.h>

#include "tracker_controller.hpp"
#include "tracker_geo.hpp"

TEST(TrackerGeo, CardinalBearings)
{
	constexpr int32_t lat = 473977420;
	constexpr int32_t lon = 85455940;

	EXPECT_NEAR(tracker_geo::bearing_rad(lat, lon, lat + 10000, lon), 0.f, 1e-4f);
	EXPECT_NEAR(tracker_geo::bearing_rad(lat, lon, lat, lon + 10000), M_PI_2_F, 1e-4f);
	EXPECT_NEAR(tracker_geo::bearing_rad(lat, lon, lat - 10000, lon), M_PI_F, 1e-4f);
	EXPECT_NEAR(tracker_geo::bearing_rad(lat, lon, lat, lon - 10000), 3.f * M_PI_2_F, 1e-4f);
}

TEST(TrackerGeo, PitchAndWrap)
{
	EXPECT_NEAR(tracker_geo::pitch_rad(100.f, 100.f), M_PI_4_F, 1e-5f);
	EXPECT_NEAR(tracker_geo::wrap_pi(3.f * M_PI_F), M_PI_F, 1e-5f);
	EXPECT_NEAR(tracker_geo::wrap_pi(-3.f * M_PI_F), -M_PI_F, 1e-5f);
}

TEST(TrackerPID, FeedForwardUsesTargetRate)
{
	TrackerPID pid;
	pid.set_gains(0.f, 0.f, 0.f, 0.5f);
	pid.set_output_limits(-1.f, 1.f);

	EXPECT_NEAR(pid.update(0.f, 0.02f, 1.f), 0.5f, 1e-6f);
}

TEST(TrackerPID, SaturationPreventsIntegratorWindup)
{
	TrackerPID pid;
	pid.set_gains(2.f, 1.f, 0.f);
	pid.set_output_limits(-1.f, 1.f);
	pid.set_integrator_limit(1.f);

	for (int i = 0; i < 100; ++i) {
		EXPECT_FLOAT_EQ(pid.update(1.f, 0.02f), 1.f);
	}

	EXPECT_FLOAT_EQ(pid.get_integrator(), 0.f);
}

TEST(TrackerPID, IntegratorAccumulatesInsideLimits)
{
	TrackerPID pid;
	pid.set_gains(0.f, 1.f, 0.f);
	pid.set_output_limits(-1.f, 1.f);
	pid.set_integrator_limit(0.5f);

	EXPECT_NEAR(pid.update(0.5f, 0.1f), 0.05f, 1e-6f);
	EXPECT_NEAR(pid.get_integrator(), 0.05f, 1e-6f);
}
