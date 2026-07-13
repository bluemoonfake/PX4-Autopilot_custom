#include "tracker_events.hpp"

#include <px4_platform_common/events.h>
#include <uORB/topics/tracker_status.h>

void TrackerEvents::update(uint8_t state, uint8_t reason, bool target_valid, bool yaw_clipped, bool pitch_clipped)
{
	if (target_valid != _previous_target_valid) {
		if (target_valid) {
			/* EVENT
			 * @description A fresh, qualified target position is available.
			 */
			events::send(events::ID("tracker_target_acquired"), events::Log::Info, "Tracker target acquired");

		} else {
			/* EVENT
			 * @description The target is unavailable or timed out. The tracker selects its configured safe pose.
			 */
			events::send(events::ID("tracker_target_lost"), events::Log::Warning, "Tracker target lost, parking");
		}
	}

	if (state != _previous_state || reason != _previous_reason) {
		switch (state) {
		case tracker_status_s::STATE_SCANNING:
			/* EVENT
			 * @description SCAN mode is active inside the configured mechanical limits.
			 */
			events::send(events::ID("tracker_scan_active"), events::Log::Info, "Tracker scan active");
			break;

		case tracker_status_s::STATE_IDLE:
		case tracker_status_s::STATE_TIMEOUT:
		case tracker_status_s::STATE_SENSOR_INVALID:
		case tracker_status_s::STATE_TARGET_TOO_CLOSE:
			/* EVENT
			 * @description The tracker selected its configured physical park pose.
			 */
			events::send(events::ID("tracker_parked"), events::Log::Info, "Tracker parking output selected");
			break;

		default:
			break;
		}
	}

	if (yaw_clipped && !_previous_yaw_clipped) {
		/* EVENT
		 * @description The desired target heading is outside the configured reachable yaw sector.
		 */
		events::send(events::ID("tracker_yaw_sector_clipped"), events::Log::Warning,
			     "Tracker yaw command limited by mechanical sector");
	}

	if (pitch_clipped && !_previous_pitch_clipped) {
		/* EVENT
		 * @description The desired target elevation is outside the configured pitch limits.
		 */
		events::send(events::ID("tracker_pitch_limit_clipped"), events::Log::Warning,
			     "Tracker pitch command limited by mechanical range");
	}

	_previous_state = state;
	_previous_reason = reason;
	_previous_target_valid = target_valid;
	_previous_yaw_clipped = yaw_clipped;
	_previous_pitch_clipped = pitch_clipped;
}
