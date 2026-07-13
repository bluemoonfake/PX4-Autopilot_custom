/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#include "MavlinkTrackerTargetBridge.hpp"

#include "mavlink_bridge_header.h"

#include <gtest/gtest.h>
#include <limits.h>

namespace
{
MavlinkTrackerTargetBridge::PositionInput valid_position(uint8_t system_id = 2,
		uint8_t component_id = MAV_COMP_ID_AUTOPILOT1)
{
	MavlinkTrackerTargetBridge::PositionInput input{};
	input.system_id = system_id;
	input.component_id = component_id;
	input.lat = 473977420;
	input.lon = 85455940;
	input.alt_mm = 588000;
	input.time_boot_ms = 1;
	input.vx_cm_s = 100;
	input.vy_cm_s = 200;
	input.vz_cm_s = 0;
	return input;
}
}

TEST(MavlinkTrackerTargetBridge, RequiresFreshHeartbeat)
{
	MavlinkTrackerTargetBridge bridge;
	bridge.configure(2, false, 5000000);
	const auto input = valid_position();

	auto result = bridge.evaluate(input, 1, true, 1000000);
	EXPECT_FALSE(result.accepted);
	EXPECT_EQ(result.reason, MavlinkTrackerTargetBridge::RejectionReason::HeartbeatMissing);

	bridge.observe_heartbeat(2, MAV_COMP_ID_AUTOPILOT1, MAV_TYPE_QUADROTOR, 1000000);
	result = bridge.evaluate(input, 1, true, 1000001);
	EXPECT_TRUE(result.accepted);
	EXPECT_TRUE(result.heartbeat_valid);
	EXPECT_EQ(result.source_type, MAV_TYPE_QUADROTOR);

	result = bridge.evaluate(input, 1, true,
			1000000 + MavlinkTrackerTargetBridge::HEARTBEAT_TIMEOUT_US + 1);
	EXPECT_FALSE(result.accepted);
	EXPECT_EQ(result.reason, MavlinkTrackerTargetBridge::RejectionReason::HeartbeatStale);
}

TEST(MavlinkTrackerTargetBridge, RejectsSelfGcsAndNonAutopilotComponents)
{
	MavlinkTrackerTargetBridge bridge;
	bridge.configure(0, true, 5000000);

	auto self = valid_position(1);
	bridge.observe_heartbeat(1, MAV_COMP_ID_AUTOPILOT1, MAV_TYPE_QUADROTOR, 1000);
	auto result = bridge.evaluate(self, 1, true, 1001);
	EXPECT_EQ(result.reason, MavlinkTrackerTargetBridge::RejectionReason::SelfSource);

	auto gcs = valid_position(2);
	bridge.observe_heartbeat(2, MAV_COMP_ID_AUTOPILOT1, MAV_TYPE_GCS, 1000);
	result = bridge.evaluate(gcs, 1, true, 1001);
	EXPECT_EQ(result.reason, MavlinkTrackerTargetBridge::RejectionReason::GcsSource);

	auto companion = valid_position(3, MAV_COMP_ID_ONBOARD_COMPUTER);
	bridge.observe_heartbeat(3, MAV_COMP_ID_ONBOARD_COMPUTER, MAV_TYPE_ONBOARD_CONTROLLER, 1000);
	result = bridge.evaluate(companion, 1, true, 1001);
	EXPECT_EQ(result.reason, MavlinkTrackerTargetBridge::RejectionReason::InvalidComponent);
}

TEST(MavlinkTrackerTargetBridge, AppliesConfiguredSystemAndFieldValidation)
{
	MavlinkTrackerTargetBridge bridge;
	bridge.configure(2, false, 5000000);
	bridge.observe_heartbeat(2, MAV_COMP_ID_AUTOPILOT1, MAV_TYPE_FIXED_WING, 1000);
	bridge.observe_heartbeat(3, MAV_COMP_ID_AUTOPILOT1, MAV_TYPE_FIXED_WING, 1000);

	auto wrong_system = valid_position(3);
	auto result = bridge.evaluate(wrong_system, 1, true, 1001);
	EXPECT_EQ(result.reason, MavlinkTrackerTargetBridge::RejectionReason::WrongSystem);

	auto invalid_position = valid_position();
	invalid_position.lat = 900000001;
	result = bridge.evaluate(invalid_position, 1, true, 1001);
	EXPECT_EQ(result.reason, MavlinkTrackerTargetBridge::RejectionReason::InvalidPosition);

	auto invalid_altitude = valid_position();
	invalid_altitude.alt_mm = INT32_MAX;
	result = bridge.evaluate(invalid_altitude, 1, true, 1001);
	EXPECT_EQ(result.reason, MavlinkTrackerTargetBridge::RejectionReason::InvalidAltitude);

	auto invalid_velocity = valid_position();
	invalid_velocity.vx_cm_s = INT16_MAX;
	result = bridge.evaluate(invalid_velocity, 1, true, 1001);
	EXPECT_TRUE(result.accepted);
	EXPECT_FALSE(result.velocity_valid);
}

TEST(MavlinkTrackerTargetBridge, AutoLockIsNotAcquiredUntilAdmissionIsFullyQualified)
{
	MavlinkTrackerTargetBridge bridge;
	bridge.configure(0, true, 5000000);
	bridge.observe_heartbeat(2, MAV_COMP_ID_AUTOPILOT1, MAV_TYPE_QUADROTOR, 1000);

	auto invalid_position = valid_position(2);
	invalid_position.lat = 900000001;
	auto result = bridge.evaluate(invalid_position, 1, true, 1001);
	EXPECT_FALSE(result.accepted);
	EXPECT_FALSE(result.lock_acquired);
	EXPECT_EQ(bridge.locked_system_id(), 0);

	result = bridge.evaluate(valid_position(2), 1, true, 1002);
	EXPECT_TRUE(result.accepted);
	EXPECT_TRUE(result.lock_acquired);
	EXPECT_EQ(bridge.locked_system_id(), 2);
}

TEST(MavlinkTrackerTargetBridge, TracksMoreThanOneHeartbeatSource)
{
	MavlinkTrackerTargetBridge bridge;
	bridge.configure(8, false, 5000000);

	for (uint8_t system_id = 2; system_id <= 8; ++system_id) {
		bridge.observe_heartbeat(system_id, MAV_COMP_ID_AUTOPILOT1, MAV_TYPE_QUADROTOR, system_id * 1000);
	}

	auto result = bridge.evaluate(valid_position(8), 1, true, 9000);
	EXPECT_TRUE(result.accepted);
	EXPECT_TRUE(result.heartbeat_valid);
}

TEST(MavlinkTrackerTargetBridge, KeepsAndReleasesAutoLock)
{
	MavlinkTrackerTargetBridge bridge;
	bridge.configure(0, true, 5000000);
	bridge.observe_heartbeat(2, MAV_COMP_ID_AUTOPILOT1, MAV_TYPE_QUADROTOR, 1000000);
	bridge.observe_heartbeat(3, MAV_COMP_ID_AUTOPILOT1, MAV_TYPE_FIXED_WING, 1000000);

	auto result = bridge.evaluate(valid_position(2), 1, true, 1000001);
	EXPECT_TRUE(result.accepted);
	EXPECT_TRUE(result.lock_acquired);
	EXPECT_EQ(bridge.locked_system_id(), 2);

	result = bridge.evaluate(valid_position(3), 1, true, 2000000);
	EXPECT_FALSE(result.accepted);
	EXPECT_EQ(result.reason, MavlinkTrackerTargetBridge::RejectionReason::LockedToOtherSource);
	EXPECT_EQ(bridge.locked_system_id(), 2);

	result = bridge.evaluate(valid_position(3), 1, true, 7000001);
	EXPECT_FALSE(result.accepted);
	EXPECT_EQ(result.reason, MavlinkTrackerTargetBridge::RejectionReason::HeartbeatStale);
	EXPECT_FALSE(result.lock_released);
	EXPECT_EQ(bridge.locked_system_id(), 2);

	bridge.observe_heartbeat(3, MAV_COMP_ID_AUTOPILOT1, MAV_TYPE_FIXED_WING, 7000000);
	result = bridge.evaluate(valid_position(3), 1, true, 7000001);
	EXPECT_TRUE(result.accepted);
	EXPECT_TRUE(result.lock_released);
	EXPECT_EQ(result.released_system_id, 2);
	EXPECT_EQ(result.released_component_id, MAV_COMP_ID_AUTOPILOT1);
	EXPECT_TRUE(result.lock_acquired);
	EXPECT_EQ(bridge.locked_system_id(), 3);
}

TEST(MavlinkTrackerTargetBridge, RejectsWhenBridgeOrSelectionIsDisabled)
{
	MavlinkTrackerTargetBridge bridge;
	bridge.configure(0, false, 5000000);
	bridge.observe_heartbeat(2, MAV_COMP_ID_AUTOPILOT1, MAV_TYPE_QUADROTOR, 1000);

	auto result = bridge.evaluate(valid_position(), 1, false, 1001);
	EXPECT_EQ(result.reason, MavlinkTrackerTargetBridge::RejectionReason::Disabled);

	result = bridge.evaluate(valid_position(), 1, true, 1001);
	EXPECT_EQ(result.reason, MavlinkTrackerTargetBridge::RejectionReason::AutoLockDisabled);
}

TEST(MavlinkTrackerTargetBridge, RejectsNonIncreasingPositionTimestampWhileSourceIsFresh)
{
	MavlinkTrackerTargetBridge bridge;
	bridge.configure(2, false, 5000000);
	bridge.observe_heartbeat(2, MAV_COMP_ID_AUTOPILOT1, MAV_TYPE_QUADROTOR, 1000);

	auto position = valid_position(2);
	position.time_boot_ms = 100;
	EXPECT_TRUE(bridge.evaluate(position, 1, true, 1001).accepted);
	EXPECT_EQ(bridge.evaluate(position, 1, true, 1002).reason,
		  MavlinkTrackerTargetBridge::RejectionReason::InvalidTimestamp);

	position.time_boot_ms = 101;
	EXPECT_TRUE(bridge.evaluate(position, 1, true, 1003).accepted);
}
