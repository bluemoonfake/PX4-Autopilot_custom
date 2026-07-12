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

#include <limits.h>

void MavlinkTrackerTargetBridge::configure(int32_t target_system, bool auto_lock, uint64_t target_timeout_us)
{
	if (target_system != _target_system || auto_lock != _auto_lock) {
		clear_lock();
	}

	_target_system = target_system;
	_auto_lock = auto_lock;
	_target_timeout_us = target_timeout_us;
}

void MavlinkTrackerTargetBridge::observe_heartbeat(uint8_t system_id, uint8_t component_id, uint8_t source_type,
		uint64_t now_us)
{
	HeartbeatSource *selected = nullptr;
	HeartbeatSource *oldest = &_heartbeat_sources[0];

	for (HeartbeatSource &source : _heartbeat_sources) {
		if (source.system_id == system_id && source.component_id == component_id) {
			selected = &source;
			break;
		}

		if (source.last_update_us == 0) {
			selected = &source;
			break;
		}

		if (source.last_update_us < oldest->last_update_us) {
			oldest = &source;
		}
	}

	if (selected == nullptr) {
		selected = oldest;
	}

	selected->system_id = system_id;
	selected->component_id = component_id;
	selected->source_type = source_type;
	selected->last_update_us = now_us;
}

MavlinkTrackerTargetBridge::Evaluation MavlinkTrackerTargetBridge::evaluate(const PositionInput &input,
		uint8_t local_system_id, bool tracker_enabled, uint64_t now_us)
{
	Evaluation result{};

	if (!tracker_enabled) {
		result.reason = RejectionReason::Disabled;
		return result;
	}

	if (input.system_id == local_system_id) {
		result.reason = RejectionReason::SelfSource;
		return result;
	}

	if (input.component_id == 0 || input.component_id != MAV_COMP_ID_AUTOPILOT1) {
		result.reason = RejectionReason::InvalidComponent;
		return result;
	}

	result.position_valid = input.lat >= -900000000 && input.lat <= 900000000
				&& input.lon >= -1800000000 && input.lon <= 1800000000;
	result.altitude_valid = input.alt_mm != INT32_MAX;
	result.velocity_valid = input.vx_cm_s != INT16_MAX && input.vy_cm_s != INT16_MAX && input.vz_cm_s != INT16_MAX;

	if (!result.position_valid) {
		result.reason = RejectionReason::InvalidPosition;
		return result;
	}

	if (!result.altitude_valid) {
		result.reason = RejectionReason::InvalidAltitude;
		return result;
	}

	const HeartbeatSource *heartbeat = find_heartbeat(input.system_id, input.component_id);

	if (heartbeat == nullptr || heartbeat->last_update_us == 0 || now_us < heartbeat->last_update_us) {
		result.reason = RejectionReason::HeartbeatMissing;
		return result;
	}

	result.source_type = heartbeat->source_type;

	if (now_us - heartbeat->last_update_us > HEARTBEAT_TIMEOUT_US) {
		result.reason = RejectionReason::HeartbeatStale;
		return result;
	}

	if (heartbeat->source_type == MAV_TYPE_GCS) {
		result.reason = RejectionReason::GcsSource;
		return result;
	}

	result.heartbeat_valid = true;

	if (_target_system == 0 && _auto_lock && _locked_system_id != 0 && _last_accepted_update_us > 0
	    && _target_timeout_us > 0 && now_us >= _last_accepted_update_us
	    && now_us - _last_accepted_update_us > _target_timeout_us) {
		result.released_system_id = _locked_system_id;
		result.released_component_id = _locked_component_id;
		clear_lock();
		result.lock_released = true;
	}

	if (_target_system != 0) {
		if (input.system_id != static_cast<uint8_t>(_target_system)) {
			result.reason = RejectionReason::WrongSystem;
			return result;
		}

	} else if (_auto_lock) {
		if (_locked_system_id == 0) {
			_locked_system_id = input.system_id;
			_locked_component_id = input.component_id;
			result.lock_acquired = true;

		} else if (input.system_id != _locked_system_id || input.component_id != _locked_component_id) {
			result.reason = RejectionReason::LockedToOtherSource;
			return result;
		}

	} else {
		result.reason = RejectionReason::AutoLockDisabled;
		return result;
	}

	_last_accepted_update_us = now_us;
	result.reason = RejectionReason::Accepted;
	result.accepted = true;
	return result;
}

const MavlinkTrackerTargetBridge::HeartbeatSource *MavlinkTrackerTargetBridge::find_heartbeat(uint8_t system_id,
		uint8_t component_id) const
{
	for (const HeartbeatSource &source : _heartbeat_sources) {
		if (source.system_id == system_id && source.component_id == component_id) {
			return &source;
		}
	}

	return nullptr;
}

void MavlinkTrackerTargetBridge::clear_lock()
{
	_locked_system_id = 0;
	_locked_component_id = 0;
	_last_accepted_update_us = 0;
}
