#include <gtest/gtest.h>

#include "tracker_controller.hpp"
#include "tracker_axis_controller.hpp"
#include "tracker_geo.hpp"
#include "tracker_servo_mapper.hpp"
#include "tracker_setpoint_planner.hpp"
#include "tracker_target_manager.hpp"

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
	EXPECT_NEAR(tracker_geo::wrap_180_deg(270.f), -90.f, 1e-5f);
	EXPECT_NEAR(tracker_geo::wrap_180_deg(-270.f), 90.f, 1e-5f);
}

TEST(TrackerGeo, HomeFallbackRejectsPlaceholderButAllowsEquatorAndPrimeMeridian)
{
	EXPECT_FALSE(tracker_geo::valid_home_fallback(0, 0));
	EXPECT_FALSE(tracker_geo::valid_home_fallback(900000001, 0));
	EXPECT_FALSE(tracker_geo::valid_home_fallback(0, 1800000001));
	EXPECT_TRUE(tracker_geo::valid_home_fallback(0, 85455940));
	EXPECT_TRUE(tracker_geo::valid_home_fallback(473977420, 0));
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

TEST(TrackerServoMapper, NonNeutralAngleRetainsNonNeutralCommandAtZeroError)
{
	TrackerServoMapper mapper;
	mapper.configure(-90.f, 90.f, -1.f, 1.f, 0.f, false);

	// Positional mapping must not collapse to neutral just because a feedback
	// correction happens to be zero at the desired head angle.
	EXPECT_NEAR(mapper.map(45.f), 0.5f, 1e-6f);
	EXPECT_NE(mapper.map(45.f), mapper.map(0.f));
}

TEST(TrackerServoMapper, ParkAndReverseMappingAreDeterministic)
{
	TrackerServoMapper mapper;
	mapper.configure(-180.f, 180.f, -0.8f, 0.8f, 0.f, false);
	EXPECT_NEAR(mapper.map(90.f), 0.4f, 1e-6f);

	mapper.configure(-180.f, 180.f, -0.8f, 0.8f, 0.f, true);
	EXPECT_NEAR(mapper.map(90.f), -0.4f, 1e-6f);

	TrackerSetpointPlanner planner;
	planner.configure(-150.f, 120.f, 30.f, -10.f, 80.f, 20.f);
	const auto park = planner.park();
	EXPECT_FLOAT_EQ(park.yaw_deg, 30.f);
	EXPECT_FLOAT_EQ(park.pitch_deg, 20.f);
	EXPECT_NEAR(mapper.map(park.yaw_deg), -0.13333333f, 1e-6f);
}

TEST(TrackerSetpointPlanner, AutoAndScanCommandsStayInsideMechanicalSector)
{
	TrackerSetpointPlanner planner;
	planner.configure(-90.f, 90.f, 0.f, 0.f, 70.f, 10.f);

	const auto auto_command = planner.from_auto(math::radians(170.f), math::radians(100.f), 0.f, 0.f);
	EXPECT_FLOAT_EQ(auto_command.yaw_deg, 90.f);
	EXPECT_FLOAT_EQ(auto_command.pitch_deg, 70.f);
	EXPECT_TRUE(auto_command.yaw_clipped);
	EXPECT_TRUE(auto_command.pitch_clipped);

	const auto scan_command = planner.constrain(-120.f, -5.f);
	EXPECT_FLOAT_EQ(scan_command.yaw_deg, -90.f);
	EXPECT_FLOAT_EQ(scan_command.pitch_deg, 0.f);
	EXPECT_TRUE(scan_command.yaw_clipped);
	EXPECT_TRUE(scan_command.pitch_clipped);
}

TEST(TrackerAxisController, ParkAndZeroErrorKeepTheMappedPhysicalCommand)
{
	TrackerServoMapper mapper;
	mapper.configure(-90.f, 90.f, -1.f, 1.f, 0.f, false);

	TrackerAxisController axis;
	axis.configure(mapper, 1.f, 0.f, 0.f, 0.f, 0.f, 0.f);
	EXPECT_NEAR(axis.command_without_correction(45.f, 0.02f), 0.5f, 1e-6f);
	EXPECT_NEAR(axis.command(45.f, 0.f, 0.f, 0.02f), 0.5f, 1e-6f);
	axis.reset();
	EXPECT_NEAR(axis.command_without_correction(0.f, 0.02f), 0.f, 1e-6f);
	axis.initialize_at_angle(-45.f);
	EXPECT_NEAR(axis.output(), -0.5f, 1e-6f);
}

TEST(TrackerAxisController, ReconfigurationResetDiscardsPriorIntegralCorrection)
{
	TrackerServoMapper mapper;
	mapper.configure(-90.f, 90.f, -1.f, 1.f, 0.f, false);

	TrackerAxisController axis;
	axis.configure(mapper, 0.f, 1.f, 0.f, 0.f, 0.25f, 0.f);
	axis.initialize_at_angle(0.f);
	EXPECT_NEAR(axis.command(45.f, 0.5f, 0.f, 0.2f), 0.6f, 1e-6f);

	// The module resets each axis before it accepts a changed physical/output
	// configuration. A new park command must not retain the old I-term.
	axis.configure(mapper, 0.f, 1.f, 0.f, 0.f, 0.25f, 0.f);
	axis.reset();
	EXPECT_NEAR(axis.command(30.f, 0.f, 0.f, 0.02f), 1.f / 3.f, 1e-6f);
}

TEST(TrackerTargetManager, MavlinkTimeoutParksWhileFakeTargetIsDeterministic)
{
	TrackerTargetManager manager;
	manager.configure(2, 1000);

	tracker_target_position_s message{};
	message.valid = true;
	message.position_valid = true;
	message.altitude_valid = true;
	message.heartbeat_valid = true;
	message.target_system = 2;
	message.lat = 473977420;
	message.lon = 85455940;
	message.alt_mm = 500000;
	message.last_update_us = 1000000;
	EXPECT_TRUE(manager.update_from_mavlink(message, 1000000));
	EXPECT_TRUE(manager.valid(1999999));
	EXPECT_FALSE(manager.valid(2000001));

	manager.set_fake_target(1, 2, 3, 2000001);
	EXPECT_TRUE(manager.valid(9000000));
	EXPECT_TRUE(manager.target().fake);
	EXPECT_EQ(manager.target().lat_e7, 1);
}
