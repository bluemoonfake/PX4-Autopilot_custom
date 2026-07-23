#pragma once

#include <mathlib/mathlib.h>

/**
 * Maps a physical axis angle into a normalized PX4 servo output.
 *
 * The output is calibrated against the physical mechanical limits rather than
 * treating a controller correction as a servo position directly.
 */
class ServoMapping
{
public:
	void configure(float angle_min_deg, float angle_max_deg)
	{
		_angle_min_deg = math::min(angle_min_deg, angle_max_deg);
		_angle_max_deg = math::max(angle_min_deg, angle_max_deg);
	}

	float map(float angle_deg) const
	{
		const float angle = math::constrain(angle_deg, _angle_min_deg, _angle_max_deg);
		const float span = _angle_max_deg - _angle_min_deg;
		const float ratio = span > 1e-3f ? (angle - _angle_min_deg) / span : 0.5f;

		return math::constrain(-1.f + 2.f * ratio, -1.f, 1.f);
	}

private:
	float _angle_min_deg{-90.f};
	float _angle_max_deg{90.f};
};
