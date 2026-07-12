#pragma once

#include <math.h>
#include <mathlib/mathlib.h>

/**
 * PID controller for antenna tracker axes.
 *
 * Features:
 *  - P, I, D gains
 *  - Feed-forward (FF) from the target angular rate
 *  - Configurable integrator limit (IMAX)
 *  - Conditional integration anti-windup
 *  - Output clamped to configurable limits
 *  - Derivative low-pass filtering via exponential moving average
 *
 * Modeled after ArduPilot AC_PID for antenna tracker.
 */
class TrackerPID
{
public:
	TrackerPID() = default;
	~TrackerPID() = default;

	/**
	 * Set PID + feed-forward gains.
	 */
	void set_gains(float kp, float ki, float kd, float kff = 0.f)
	{
		_kp = kp;
		_ki = ki;
		_kd = kd;
		_kff = kff;
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
		_d_filtered = 0.f;
	}

	/**
	 * Compute PID + feed-forward output.
	 *
	 * @param error        Tracking error (setpoint - measurement)
	 * @param dt           Time step in seconds
	 * @param target_rate  Target angular rate in radians/second
	 * @return normalized output clamped to [min_out, max_out]
	 */
	float update(float error, float dt, float target_rate = 0.f)
	{
		if (dt < 1e-6f || !PX4_ISFINITE(error) || !PX4_ISFINITE(target_rate)) {
			return 0.f;
		}

		// Proportional
		const float p_term = _kp * error;

		// Derivative (on error) with simple low-pass filter
		float d_term = 0.f;

		if (_prev_valid && _kd > 0.f) {
			const float d_raw = _kd * (error - _prev_error) / dt;
			// Simple EMA filter, alpha ~ dt / (dt + 1/(2*pi*cutoff))
			// With cutoff ~10Hz: tau = 1/(2*pi*10) ≈ 0.016
			const float tau = 0.016f;
			const float alpha = dt / (dt + tau);
			_d_filtered = _d_filtered + alpha * (d_raw - _d_filtered);
			d_term = _d_filtered;
		}

		const float ff_term = _kff * target_rate;
		const float integrator_delta = _ki * error * dt;
		const float integrator_candidate = math::constrain(_integrator + integrator_delta,
						   -_integrator_limit, _integrator_limit);
		const float candidate_output = p_term + integrator_candidate + d_term + ff_term;

		// Only integrate when it does not drive an already saturated output farther into saturation.
		if (!((candidate_output > _max_out && integrator_delta > 0.f)
		      || (candidate_output < _min_out && integrator_delta < 0.f))) {
			_integrator = integrator_candidate;
		}

		_prev_error = error;
		_prev_valid = true;

		// Sum and clamp
		float output = p_term + _integrator + d_term + ff_term;
		output = math::constrain(output, _min_out, _max_out);

		return output;
	}

	float get_integrator() const { return _integrator; }

private:
	float _kp{0.f};
	float _ki{0.f};
	float _kd{0.f};
	float _kff{0.f};

	float _integrator{0.f};
	float _integrator_limit{0.5f};

	float _prev_error{0.f};
	bool  _prev_valid{false};
	float _d_filtered{0.f};

	float _min_out{-1.f};
	float _max_out{1.f};
};
