#pragma once

#include <cstdint>

/** Emits edge-triggered PX4 Events for tracker operator-visible transitions. */
class TrackerEvents
{
public:
	void update(uint8_t state, uint8_t reason, bool target_valid, bool yaw_clipped, bool pitch_clipped);

private:
	uint8_t _previous_state{UINT8_MAX};
	uint8_t _previous_reason{UINT8_MAX};
	bool _previous_target_valid{false};
	bool _previous_yaw_clipped{false};
	bool _previous_pitch_clipped{false};
};
