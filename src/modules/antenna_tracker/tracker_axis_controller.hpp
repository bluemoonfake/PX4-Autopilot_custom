#pragma once

#include <cstdint>

#include <mathlib/mathlib.h>

#include "tracker_controller.hpp"
#include "lib/servo_mapping.hpp"

/**
 * Stateful command path for one positional servo axis.
 */
class TrackerAxisController
{
public:
	void configure(const ServoMapping &mapper, float p, float i, float d, float ff,
		       float integrator_limit, float slew_time)
	{
		_mapper = mapper;
		_pid.set_gains(p, i, d, ff);
		_pid.set_output_limits(-POSITION_CORRECTION_LIMIT, POSITION_CORRECTION_LIMIT);
		_pid.set_integrator_limit(math::constrain(integrator_limit, 0.f, POSITION_CORRECTION_LIMIT));
		_slew_time = math::max(slew_time, 0.f);
	}

	void reset()
	{
		_pid.reset();
	}

	float command(float angle_deg, float error_rad, float target_rate_rad_s, float dt)
	{
		const float controller_output = _pid.update(error_rad, dt, target_rate_rad_s);
		const float desired = _mapper.map(angle_deg) + controller_output;
		return apply_slew(desired, dt);
	}

	float command_without_correction(float angle_deg, float dt)
	{
		return apply_slew(_mapper.map(angle_deg), dt);
	}

	/** Set the initial output before the first actuator publication at boot. */
	void initialize_at_angle(float angle_deg)
	{
		_pid.reset();
		_output = _mapper.map(angle_deg);
	}

	float safe_output(float park_angle_deg) const
	{
		return _mapper.map(park_angle_deg);
	}

	float output() const { return _output; }

private:
	static constexpr float POSITION_CORRECTION_LIMIT = 0.25f;

	float apply_slew(float desired, float dt)
	{
		desired = math::constrain(desired, -1.f, 1.f);

		if (_slew_time < 0.01f || dt < 1e-6f) {
			_output = desired;
			return _output;
		}

		const float max_delta = 2.f * dt / _slew_time;
		_output = math::constrain(desired, _output - max_delta, _output + max_delta);
		return _output;
	}

	ServoMapping _mapper{};
	TrackerPID _pid{};
	float _slew_time{0.f};
	float _output{0.f};
};
