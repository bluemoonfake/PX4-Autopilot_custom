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

#pragma once

#include <stdint.h>

class MavlinkTrackerTargetBridge
{
public:
	static constexpr uint64_t HEARTBEAT_TIMEOUT_US = 2500000ULL;

	enum class RejectionReason : uint8_t {
		Accepted = 0,
		Disabled,
		SelfSource,
		InvalidComponent,
		InvalidPosition,
		InvalidAltitude,
		InvalidTimestamp,
		HeartbeatMissing,
		HeartbeatStale,
		GcsSource,
		WrongSystem,
		AutoLockDisabled,
		LockedToOtherSource
	};

	struct PositionInput {
		uint8_t system_id{0};
		uint8_t component_id{0};
		int32_t lat{0};
		int32_t lon{0};
		int32_t alt_mm{0};
		uint32_t time_boot_ms{0};
		int16_t vx_cm_s{0};
		int16_t vy_cm_s{0};
		int16_t vz_cm_s{0};
	};

	struct Evaluation {
		RejectionReason reason{RejectionReason::Disabled};
		bool accepted{false};
		bool position_valid{false};
		bool altitude_valid{false};
		bool velocity_valid{false};
		bool heartbeat_valid{false};
		bool lock_acquired{false};
		bool lock_released{false};
		uint8_t released_system_id{0};
		uint8_t released_component_id{0};
		uint8_t source_type{0};
	};

	void configure(int32_t target_system, bool auto_lock, uint64_t target_timeout_us);
	void observe_heartbeat(uint8_t system_id, uint8_t component_id, uint8_t source_type, uint64_t now_us);
	Evaluation evaluate(const PositionInput &input, uint8_t local_system_id, bool tracker_enabled, uint64_t now_us);

	uint8_t locked_system_id() const { return _locked_system_id; }
	uint8_t locked_component_id() const { return _locked_component_id; }

private:
	static constexpr unsigned MAX_HEARTBEAT_SOURCES = 16;

	struct HeartbeatSource {
		uint64_t last_update_us{0};
		uint8_t system_id{0};
		uint8_t component_id{0};
		uint8_t source_type{0};
		uint32_t last_position_time_boot_ms{0};
		bool has_position_time{false};
	};

	HeartbeatSource *find_heartbeat(uint8_t system_id, uint8_t component_id);
	static bool time_boot_is_newer(uint32_t newer, uint32_t older);
	void clear_lock();

	HeartbeatSource _heartbeat_sources[MAX_HEARTBEAT_SOURCES] {};
	uint8_t _locked_system_id{0};
	uint8_t _locked_component_id{0};
	uint64_t _last_accepted_update_us{0};
	int32_t _target_system{0};
	bool _auto_lock{false};
	uint64_t _target_timeout_us{0};
};
