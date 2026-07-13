#pragma once

#include <mathlib/mathlib.h>

/**
 * Maps a physical axis angle into a normalized PX4 Servo output.
 *
 * The output is calibrated against the physical mechanical limits rather than
 * treating a controller correction as a servo position directly.
 */
class TrackerServoMapper
{
public:
	void configure(float angle_min_deg, float angle_max_deg, float output_min, float output_max, float trim, bool reverse)
	{
		_angle_min_deg = math::min(angle_min_deg, angle_max_deg);
		_angle_max_deg = math::max(angle_min_deg, angle_max_deg);
		_output_min = math::min(output_min, output_max);
		_output_max = math::max(output_min, output_max);
		_trim = trim;
		_reverse = reverse;
	}

	float map(float angle_deg) const
	{
		const float angle = math::constrain(angle_deg, _angle_min_deg, _angle_max_deg);
		const float span = _angle_max_deg - _angle_min_deg;
		float ratio = span > 1e-3f ? (angle - _angle_min_deg) / span : 0.5f;
		if (_reverse) {
			ratio = 1.f - ratio;
		}
		const float output = _output_min + ratio * (_output_max - _output_min) + _trim;
		return math::constrain(output, _output_min, _output_max);
	}

	/** Convert a bounded normalized test command back to the calibrated angle domain. */
	float angle_for_output(float output) const
	{
		const float adjusted_output = math::constrain(output, _output_min, _output_max) - _trim;
		const float output_span = _output_max - _output_min;
		float ratio = output_span > 1e-3f ? (adjusted_output - _output_min) / output_span : 0.5f;
		ratio = math::constrain(ratio, 0.f, 1.f);

		if (_reverse) {
			ratio = 1.f - ratio;
		}

		return _angle_min_deg + ratio * (_angle_max_deg - _angle_min_deg);
	}

private:
	float _angle_min_deg{-90.f};
	float _angle_max_deg{90.f};
	float _output_min{-1.f};
	float _output_max{1.f};
	float _trim{0.f};
	bool _reverse{false};
};
