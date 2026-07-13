#pragma once

#include <cstdint>

#include <uORB/topics/tracker_target_position.h>

/**
 * Owns target freshness and the source data consumed by tracker geometry.
 *
 * MAVLink source admission belongs to MavlinkTrackerTargetBridge. This class
 * handles only the module-side cache, configured system-ID guard, timeout,
 * and deterministic bench fake target.
 */
class TrackerTargetManager
{
public:
	struct Target {
		int32_t lat_e7{0};
		int32_t lon_e7{0};
		int32_t alt_mm{0};
		float vx_m_s{0.f};
		float vy_m_s{0.f};
		float vz_m_s{0.f};
		uint8_t system_id{0};
		uint8_t component_id{0};
		bool velocity_valid{false};
		bool fake{false};
		uint64_t last_update_us{0};
	};

	void configure(int32_t configured_system_id, uint32_t timeout_ms)
	{
		if (configured_system_id != _configured_system_id) {
			reset();
		}

		_configured_system_id = configured_system_id;
		_timeout_us = static_cast<uint64_t>(timeout_ms) * 1000ULL;
	}

	void reset()
	{
		_target = {};
	}

	bool update_from_mavlink(const tracker_target_position_s &message, uint64_t now_us)
	{
		const bool selected_system = _configured_system_id == 0 || message.target_system == _configured_system_id;
		const bool fresh_timestamp = message.last_update_us > 0 && message.last_update_us <= now_us;

		if (!message.valid || !message.position_valid || !message.altitude_valid || !message.heartbeat_valid
		    || !selected_system || !fresh_timestamp) {
			return false;
		}

		_target.lat_e7 = message.lat;
		_target.lon_e7 = message.lon;
		_target.alt_mm = message.alt_mm;
		_target.vx_m_s = message.vx_m_s;
		_target.vy_m_s = message.vy_m_s;
		_target.vz_m_s = message.vz_m_s;
		_target.system_id = message.target_system;
		_target.component_id = message.source_component;
		_target.velocity_valid = message.velocity_valid;
		_target.fake = false;
		_target.last_update_us = message.last_update_us;
		return true;
	}

	void set_fake_target(int32_t lat_e7, int32_t lon_e7, int32_t alt_mm, uint64_t now_us)
	{
		_target.lat_e7 = lat_e7;
		_target.lon_e7 = lon_e7;
		_target.alt_mm = alt_mm;
		_target.vx_m_s = 0.f;
		_target.vy_m_s = 0.f;
		_target.vz_m_s = 0.f;
		_target.system_id = 0;
		_target.component_id = 0;
		_target.velocity_valid = false;
		_target.fake = true;
		_target.last_update_us = now_us;
	}

	bool valid(uint64_t now_us) const
	{
		if (_target.last_update_us == 0 || now_us < _target.last_update_us) {
			return false;
		}

		return _target.fake || _timeout_us == 0 || (now_us - _target.last_update_us <= _timeout_us);
	}

	const Target &target() const { return _target; }
	uint32_t age_ms(uint64_t now_us) const
	{
		return _target.last_update_us > 0 && now_us >= _target.last_update_us
		       ? static_cast<uint32_t>((now_us - _target.last_update_us) / 1000ULL) : 0;
	}

private:
	Target _target{};
	int32_t _configured_system_id{-1};
	uint64_t _timeout_us{0};
};
