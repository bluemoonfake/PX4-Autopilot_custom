#pragma once

#include <math.h>
#include <mathlib/mathlib.h>

/**
 * Simple PID controller for antenna tracker axes.
 *
 * Output is clamped to [-1, 1].
 * Integrator has anti-windup: stops accumulating when output is saturated.
 */
class TrackerPID
{
public:
	TrackerPID() = default;
	~TrackerPID() = default;

	/**
	 * Set PID gains.
	 */
	void set_gains(float kp, float ki, float kd)
	{
		_kp = kp;
		_ki = ki;
		_kd = kd;
	}

	/**
	 * Set output limits.
	 */
	void set_output_limits(float min_out, float max_out)
	{
		_min_out = min_out;
		_max_out = max_out;
	}

	/**
	 * Set integrator limit (symmetric).
	 */
	void set_integrator_limit(float limit)
	{
		_integrator_limit = fabsf(limit);
	}

	/**
	 * Reset the controller state (integrator + previous error).
	 */
	void reset()
	{
		_integrator = 0.f;
		_prev_error = 0.f;
		_prev_valid = false;
	}

	/**
	 * Compute PID output.
	 *
	 * @param error  Tracking error (setpoint - measurement)
	 * @param dt     Time step in seconds
	 * @return normalized output clamped to [min_out, max_out]
	 */
	float update(float error, float dt)
	{
		if (dt < 1e-6f || !PX4_ISFINITE(error)) {
			return 0.f;
		}

		// Proportional
		const float p_term = _kp * error;

		// Integral with anti-windup
		_integrator += _ki * error * dt;
		_integrator = math::constrain(_integrator, -_integrator_limit, _integrator_limit);

		// Derivative (on error)
		float d_term = 0.f;

		if (_prev_valid && _kd > 0.f) {
			d_term = _kd * (error - _prev_error) / dt;
		}

		_prev_error = error;
		_prev_valid = true;

		// Sum and clamp
		float output = p_term + _integrator + d_term;
		output = math::constrain(output, _min_out, _max_out);

		return output;
	}

	float get_integrator() const { return _integrator; }

private:
	float _kp{0.f};
	float _ki{0.f};
	float _kd{0.f};

	float _integrator{0.f};
	float _integrator_limit{0.5f};

	float _prev_error{0.f};
	bool  _prev_valid{false};

	float _min_out{-1.f};
	float _max_out{1.f};
};
