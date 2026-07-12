#pragma once

#include <mathlib/mathlib.h>
#include <matrix/math.hpp>

/**
 * Converts earth-frame target angles into bounded physical pan/tilt angles.
 *
 * The yaw axis is expressed relative to the heading measured while the head is
 * at its configured park angle. This lets a positional servo retain a physical
 * command after the earth-frame tracking error reaches zero.
 */
class TrackerSetpointPlanner
{
public:
	struct Setpoint {
		float yaw_deg{0.f};
		float pitch_deg{0.f};
		bool yaw_clipped{false};
		bool pitch_clipped{false};
	};

	void configure(float yaw_min_deg, float yaw_max_deg, float yaw_park_deg,
		       float pitch_min_deg, float pitch_max_deg, float pitch_park_deg)
	{
		_yaw_min_deg = math::min(yaw_min_deg, yaw_max_deg);
		_yaw_max_deg = math::max(yaw_min_deg, yaw_max_deg);
		_yaw_park_deg = math::constrain(yaw_park_deg, _yaw_min_deg, _yaw_max_deg);
		_pitch_min_deg = math::min(pitch_min_deg, pitch_max_deg);
		_pitch_max_deg = math::max(pitch_min_deg, pitch_max_deg);
		_pitch_park_deg = math::constrain(pitch_park_deg, _pitch_min_deg, _pitch_max_deg);
	}

	Setpoint park() const
	{
		return {_yaw_park_deg, _pitch_park_deg, false, false};
	}

	Setpoint from_auto(float bearing_rad, float elevation_rad, float yaw_home_rad, float pitch_home_rad) const
	{
		const float yaw_relative_deg = math::degrees(matrix::wrap_pi(bearing_rad - yaw_home_rad));
		const float pitch_relative_deg = math::degrees(elevation_rad - pitch_home_rad);
		return constrain(_yaw_park_deg + yaw_relative_deg, _pitch_park_deg + pitch_relative_deg);
	}

	Setpoint constrain(float yaw_deg, float pitch_deg) const
	{
		Setpoint result{};
		result.yaw_deg = math::constrain(yaw_deg, _yaw_min_deg, _yaw_max_deg);
		result.pitch_deg = math::constrain(pitch_deg, _pitch_min_deg, _pitch_max_deg);
		result.yaw_clipped = fabsf(result.yaw_deg - yaw_deg) > 1e-3f;
		result.pitch_clipped = fabsf(result.pitch_deg - pitch_deg) > 1e-3f;
		return result;
	}

	float yaw_min_deg() const { return _yaw_min_deg; }
	float yaw_max_deg() const { return _yaw_max_deg; }
	float pitch_min_deg() const { return _pitch_min_deg; }
	float pitch_max_deg() const { return _pitch_max_deg; }
	float yaw_park_deg() const { return _yaw_park_deg; }
	float pitch_park_deg() const { return _pitch_park_deg; }

private:
	float _yaw_min_deg{-180.f};
	float _yaw_max_deg{180.f};
	float _yaw_park_deg{0.f};
	float _pitch_min_deg{0.f};
	float _pitch_max_deg{90.f};
	float _pitch_park_deg{0.f};
};
