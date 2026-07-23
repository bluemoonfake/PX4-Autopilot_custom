#include "antenna_tracker.hpp"

#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <matrix/math.hpp>
#include <parameters/param.h>

#include <inttypes.h>
#include <string.h>

using namespace time_literals;

AntennaTracker::AntennaTracker() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers)
{
	_boot_time_us = hrt_absolute_time();
}

AntennaTracker::~AntennaTracker()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool AntennaTracker::ControlConfiguration::equals(const ControlConfiguration &other) const
{
	const auto same_float = [](float lhs, float rhs) { return memcmp(&lhs, &rhs, sizeof(lhs)) == 0; };

	return same_float(yaw_p, other.yaw_p) && same_float(yaw_i, other.yaw_i) && same_float(yaw_d, other.yaw_d)
	       && same_float(yaw_ff, other.yaw_ff) && same_float(yaw_imax, other.yaw_imax)
	       && same_float(pitch_p, other.pitch_p) && same_float(pitch_i, other.pitch_i)
	       && same_float(pitch_d, other.pitch_d) && same_float(pitch_ff, other.pitch_ff)
	       && same_float(pitch_imax, other.pitch_imax) && same_float(yaw_min_deg, other.yaw_min_deg)
	       && same_float(yaw_max_deg, other.yaw_max_deg)
	       && same_float(yaw_park_deg, other.yaw_park_deg)
	       && same_float(pitch_min_deg, other.pitch_min_deg) && same_float(pitch_max_deg, other.pitch_max_deg)
	       && same_float(pitch_park_deg, other.pitch_park_deg)
	       && same_float(yaw_slew, other.yaw_slew) && same_float(pitch_slew, other.pitch_slew)
	       && same_float(reference_settle, other.reference_settle);
}

AntennaTracker::ControlConfiguration AntennaTracker::control_configuration() const
{
	return {
		_param_trk_yaw_p.get(), _param_trk_yaw_i.get(), _param_trk_yaw_d.get(), _param_trk_yaw_ff.get(),
		_param_trk_yaw_imax.get(), _param_trk_pit_p.get(), _param_trk_pit_i.get(), _param_trk_pit_d.get(),
		_param_trk_pit_ff.get(), _param_trk_pit_imax.get(),
		_param_trk_yaw_mind.get(), _param_trk_yaw_maxd.get(), _param_trk_yaw_park.get(),
		_param_trk_pit_mind.get(), _param_trk_pit_maxd.get(), _param_trk_pit_park.get(),
		_param_trk_yaw_slew.get(), _param_trk_pit_slew.get(), _param_trk_ref_settle.get()
	};
}

bool AntennaTracker::init()
{
	parameters_update();

	if (!_vehicle_attitude_sub.registerCallback()) {
		PX4_ERR("vehicle_attitude callback registration failed");
		return false;
	}

	return true;
}

int AntennaTracker::determine_mode()
{
	return math::constrain(_param_trk_mode.get(), int32_t{0}, int32_t{2});
}

bool AntennaTracker::safety_inhibited() const
{
	return _actuator_armed.kill || _actuator_armed.lockdown || _actuator_armed.termination;
}

bool AntennaTracker::operation_permitted() const
{
	return _actuator_armed.armed && !safety_inhibited();
}

void AntennaTracker::handle_arming_transition(bool operation_allowed)
{
	if (_arming_state_initialized && operation_allowed == _operation_permitted) {
		return;
	}

	reset_tracking_controller();
	_active_mode = -1;
	invalidate_head_reference();

	if (!operation_allowed) {
		const auto park = _setpoint_planner.park();
		_yaw_axis_controller.initialize_at_angle(park.yaw_deg);
		_pitch_axis_controller.initialize_at_angle(park.pitch_deg);
	}

	_operation_permitted = operation_allowed;
	_arming_state_initialized = true;

	if (operation_allowed) {
		PX4_INFO("tracker operation enabled by ARM");

	} else {
		PX4_INFO("tracker operation inhibited, parking");
	}
}

void AntennaTracker::reset_tracking_controller()
{
	_yaw_axis_controller.reset();
	_pitch_axis_controller.reset();
	_prev_bearing_rad = 0.f;
	_prev_pitch_target_rad = 0.f;
	_prev_bearing_valid = false;
}

void AntennaTracker::invalidate_head_reference()
{
	_head_reference_valid = false;
	_yaw_home_rad = 0.f;
	_pitch_home_rad = 0.f;
	_park_output_reached_us = 0;
}

bool AntennaTracker::ensure_head_reference(const vehicle_attitude_s &attitude, uint64_t now_us)
{
	if (_head_reference_valid) {
		return true;
	}

	const uint64_t settle_us = static_cast<uint64_t>(math::constrain(_param_trk_ref_settle.get(), 0.1f, 10.f) * 1e6f);

	if (_park_output_reached_us == 0 || now_us - _park_output_reached_us < settle_us) {
		return false;
	}

	const matrix::Eulerf euler{matrix::Quatf{attitude.q}};
	_yaw_home_rad = euler.psi();
	_pitch_home_rad = euler.theta();
	_head_reference_valid = true;
	PX4_INFO("tracker head reference captured at park");
	return true;
}

void AntennaTracker::update_park_settle_time(uint64_t now_us, float yaw_output, float pitch_output)
{
	const auto park = _setpoint_planner.park();
	const bool yaw_at_park = fabsf(yaw_output - _yaw_axis_controller.safe_output(park.yaw_deg)) <= 0.01f;
	const bool pitch_at_park = fabsf(pitch_output - _pitch_axis_controller.safe_output(park.pitch_deg)) <= 0.01f;

	if (yaw_at_park && pitch_at_park) {
		if (_park_output_reached_us == 0) {
			_park_output_reached_us = now_us;
		}

	} else {
		_park_output_reached_us = 0;
	}
}

void AntennaTracker::handle_mode_change(int new_mode)
{
	if (new_mode == _active_mode) {
		return;
	}

	reset_tracking_controller();
	invalidate_head_reference();

	_active_mode = new_mode;
}

bool AntennaTracker::is_global_position_valid(const vehicle_global_position_s &gpos)
{
	return gpos.timestamp > 0 && hrt_elapsed_time(&gpos.timestamp) < 2_s
	       && gpos.lat_lon_valid && gpos.alt_valid
	       && PX4_ISFINITE(gpos.lat) && PX4_ISFINITE(gpos.lon) && PX4_ISFINITE(gpos.alt)
	       && fabs(gpos.lat) <= 90.0 && fabs(gpos.lon) <= 180.0;
}

bool AntennaTracker::is_attitude_valid(const vehicle_attitude_s &attitude)
{
	if (attitude.timestamp == 0 || hrt_elapsed_time(&attitude.timestamp) >= 500_ms) {
		return false;
	}

	float norm_squared = 0.f;

	for (float component : attitude.q) {
		if (!PX4_ISFINITE(component)) {
			return false;
		}

		norm_squared += component * component;
	}

	return norm_squared > 0.5f && norm_squared < 1.5f;
}

uint8_t AntennaTracker::safe_reason_for_state(uint8_t state) const
{
	switch (state) {
	case tracker_status_s::STATE_TIMEOUT:
		return tracker_status_s::REASON_TARGET_TIMEOUT;
	case tracker_status_s::STATE_SENSOR_INVALID:
		return tracker_status_s::REASON_SENSOR_INVALID;
	case tracker_status_s::STATE_TARGET_TOO_CLOSE:
		return tracker_status_s::REASON_TARGET_TOO_CLOSE;
	case tracker_status_s::STATE_RECONFIGURING:
		return tracker_status_s::REASON_CONFIGURATION_CHANGED;
	case tracker_status_s::STATE_IDLE:
		return _startup_delay_done ? tracker_status_s::REASON_STOP : tracker_status_s::REASON_STARTUP;
	default:
		return tracker_status_s::REASON_NONE;
	}
}

void AntennaTracker::publish_servo_output(float yaw_output, float pitch_output, int mode, uint8_t state,
		bool target_valid, uint8_t state_reason)
{
	_yaw_output = math::constrain(yaw_output, -1.f, 1.f);
	_pitch_output = math::constrain(pitch_output, -1.f, 1.f);
	_state_reason = state_reason;

	actuator_servos_s servos{};
	servos.timestamp = hrt_absolute_time();
	servos.timestamp_sample = servos.timestamp;

	for (int i = 0; i < actuator_servos_s::NUM_CONTROLS; ++i) {
		servos.control[i] = NAN;
	}

	servos.control[0] = _yaw_output;
	servos.control[1] = _pitch_output;
	_actuator_servos_pub.publish(servos);

	tracker_status_s status{};
	status.timestamp = servos.timestamp;
	status.target_valid = target_valid;
	status.target_system = _target_manager.target().system_id;
	status.target_age_ms = _target_manager.age_ms(servos.timestamp);
	status.bearing_rad = _bearing_rad;
	status.pitch_rad = _pitch_rad;
	status.distance_m = _distance_m;
	status.yaw_error_rad = _yaw_error_rad;
	status.pitch_error_rad = _pitch_error_rad;
	status.yaw_output = _yaw_output;
	status.pitch_output = _pitch_output;
	status.mode = mode;
	status.state = state;
	status.state_reason = state_reason;
	status.yaw_setpoint_rad = _yaw_setpoint_rad;
	status.pitch_setpoint_rad = _pitch_setpoint_rad;
	status.yaw_measured_rad = _yaw_measured_rad;
	status.pitch_measured_rad = _pitch_measured_rad;
	status.yaw_command_deg = _yaw_command_deg;
	status.pitch_command_deg = _pitch_command_deg;
	status.yaw_clipped = _yaw_clipped;
	status.pitch_clipped = _pitch_clipped;
	status.position_source = _position_source;
	status.armed = _actuator_armed.armed;
	status.requested_mode = determine_mode();
	status.effective_mode = operation_permitted() ? mode : 0;
	_tracker_status_pub.publish(status);
	_events.update(state, state_reason, target_valid, _yaw_clipped, _pitch_clipped);
}

void AntennaTracker::publish_safe_output(int mode, uint8_t state, bool target_valid, uint8_t state_reason)
{
	reset_tracking_controller();
	const auto park = _setpoint_planner.park();
	_yaw_command_deg = park.yaw_deg;
	_pitch_command_deg = park.pitch_deg;
	_yaw_clipped = false;
	_pitch_clipped = false;
	_yaw_setpoint_rad = NAN;
	_pitch_setpoint_rad = NAN;
	_yaw_error_rad = 0.f;
	_pitch_error_rad = 0.f;

	const float yaw_output = _yaw_axis_controller.command_without_correction(park.yaw_deg, 0.02f);
	const float pitch_output = _pitch_axis_controller.command_without_correction(park.pitch_deg, 0.02f);
	update_park_settle_time(hrt_absolute_time(), yaw_output, pitch_output);

	publish_servo_output(yaw_output, pitch_output, mode, state, target_valid,
			     state_reason == tracker_status_s::REASON_NONE ? safe_reason_for_state(state) : state_reason);
}

void AntennaTracker::Run()
{
	if (should_exit()) {
		_vehicle_attitude_sub.unregisterCallback();
		ScheduleClear();
		exit_and_cleanup();
		return;
	}

	perf_begin(_loop_perf);
	perf_count(_loop_interval_perf);

	if (_parameter_update_sub.updated()) {
		parameter_update_s update{};
		_parameter_update_sub.copy(&update);
		parameters_update();
	}

	actuator_armed_s actuator_armed{};

	if (_actuator_armed_sub.update(&actuator_armed)) {
		_actuator_armed = actuator_armed;
	}

	const bool operation_allowed = operation_permitted();
	handle_arming_transition(operation_allowed);

	const uint64_t now_us = hrt_absolute_time();
	float dt = 0.02f;

	if (_last_run_us > 0) {
		dt = math::constrain((now_us - _last_run_us) * 1e-6f, 0.001f, 0.1f);
	}

	_last_run_us = now_us;

	if (_param_trk_strt_dly.get() > 0.f && !_startup_delay_done) {
		if ((now_us - _boot_time_us) < static_cast<uint64_t>(_param_trk_strt_dly.get() * 1e6f)) {
			publish_safe_output(determine_mode(), tracker_status_s::STATE_IDLE);
			perf_end(_loop_perf);
			return;
		}

		_startup_delay_done = true;
	}

	const int active_mode = determine_mode();
	if (_configuration_repark_pending) {
		_position_source = tracker_status_s::POSITION_SOURCE_NONE;
		publish_safe_output(active_mode, tracker_status_s::STATE_RECONFIGURING, _target_manager.valid(now_us),
				    tracker_status_s::REASON_CONFIGURATION_CHANGED);

		const uint64_t settle_us = static_cast<uint64_t>(math::constrain(_param_trk_ref_settle.get(), 0.1f, 10.f) * 1e6f);
		const uint64_t settle_check_us = hrt_absolute_time();

		if (_park_output_reached_us != 0 && settle_check_us >= _park_output_reached_us
		    && settle_check_us - _park_output_reached_us >= settle_us) {
			_configuration_repark_pending = false;
			PX4_INFO("tracker configuration re-park complete");
		}

		perf_end(_loop_perf);
		return;
	}

	handle_mode_change(active_mode);

	if (!operation_allowed) {
		_position_source = tracker_status_s::POSITION_SOURCE_NONE;
		const bool inhibited = safety_inhibited();
		const bool waiting_for_arm = !inhibited && active_mode != 0;
		const uint8_t safe_state = inhibited ? tracker_status_s::STATE_SAFETY_INHIBITED :
					   (waiting_for_arm ? tracker_status_s::STATE_WAITING_FOR_ARM : tracker_status_s::STATE_IDLE);
		const uint8_t safe_reason = inhibited ? tracker_status_s::REASON_SAFETY_INHIBITED :
					    (waiting_for_arm ? tracker_status_s::REASON_DISARMED : tracker_status_s::REASON_NONE);
		publish_safe_output(active_mode, safe_state, _target_manager.valid(now_us), safe_reason);
		perf_end(_loop_perf);
		return;
	}

	switch (active_mode) {
	case 0:
		_position_source = tracker_status_s::POSITION_SOURCE_NONE;
		publish_safe_output(0, tracker_status_s::STATE_IDLE);
		break;
	case 1:
		run_tracking(dt);
		break;
	case 2:
		_position_source = tracker_status_s::POSITION_SOURCE_NONE;
		run_manual(dt);
		break;
	default:
		publish_safe_output(0, tracker_status_s::STATE_IDLE);
		break;
	}

	perf_end(_loop_perf);
}

void AntennaTracker::parameters_update()
{
	updateParams();
	const ControlConfiguration new_control = control_configuration();
	const bool control_changed = _configured_control_valid && !_configured_control.equals(new_control);
	_configured_control = new_control;
	_configured_control_valid = true;

	if (_configured_target_sysid != _param_trk_sysid_tgt.get()
	    || _configured_auto_lock != _param_trk_auto_lock.get()) {
		_configured_target_sysid = _param_trk_sysid_tgt.get();
		_configured_auto_lock = _param_trk_auto_lock.get();
		_target_manager.reset();
		reset_tracking_controller();
	}

	_setpoint_planner.configure(_param_trk_yaw_mind.get(), _param_trk_yaw_maxd.get(), _param_trk_yaw_park.get(),
				   _param_trk_pit_mind.get(), _param_trk_pit_maxd.get(), _param_trk_pit_park.get());
	_yaw_servo_mapper.configure(_setpoint_planner.yaw_min_deg(), _setpoint_planner.yaw_max_deg());
	_pitch_servo_mapper.configure(_setpoint_planner.pitch_min_deg(), _setpoint_planner.pitch_max_deg());

	// PID supplies a bounded trim correction around, not instead of, positional mapping.
	_yaw_axis_controller.configure(_yaw_servo_mapper, _param_trk_yaw_p.get(), _param_trk_yaw_i.get(),
				       _param_trk_yaw_d.get(), _param_trk_yaw_ff.get(), _param_trk_yaw_imax.get(),
				       _param_trk_yaw_slew.get());
	_pitch_axis_controller.configure(_pitch_servo_mapper, _param_trk_pit_p.get(), _param_trk_pit_i.get(),
					 _param_trk_pit_d.get(), _param_trk_pit_ff.get(), _param_trk_pit_imax.get(),
					 _param_trk_pit_slew.get());
	_target_manager.configure(_param_trk_sysid_tgt.get(),
				  static_cast<uint32_t>(math::max(_param_trk_timeout_ms.get(), int32_t{0})));

	if (!_axis_outputs_initialized) {
		const auto park = _setpoint_planner.park();
		_yaw_axis_controller.initialize_at_angle(park.yaw_deg);
		_pitch_axis_controller.initialize_at_angle(park.pitch_deg);
		_axis_outputs_initialized = true;
	}

	if (control_changed) {
		reset_tracking_controller();
		invalidate_head_reference();
		_configuration_repark_pending = determine_mode() != 0;
		PX4_WARN("tracker output configuration changed; re-parking before resuming");
	}
}

void AntennaTracker::run_tracking(float dt)
{
	vehicle_attitude_s attitude{};
	const bool attitude_valid = _vehicle_attitude_sub.copy(&attitude) && is_attitude_valid(attitude);

	vehicle_global_position_s global_position{};
	bool position_valid = _vehicle_global_position_sub.copy(&global_position) && is_global_position_valid(global_position);
	_position_source = tracker_status_s::POSITION_SOURCE_NONE;
	int32_t tracker_lat_e7 = 0;
	int32_t tracker_lon_e7 = 0;
	float tracker_alt_m = 0.f;

	if (position_valid) {
		tracker_lat_e7 = static_cast<int32_t>(global_position.lat * 1e7);
		tracker_lon_e7 = static_cast<int32_t>(global_position.lon * 1e7);
		tracker_alt_m = global_position.alt;
		_position_source = tracker_status_s::POSITION_SOURCE_GLOBAL;

	} else if (_param_trk_home_en.get() != 0
		   && tracker_geo::valid_home_fallback(_param_trk_home_lat.get(), _param_trk_home_lon.get())) {
		tracker_lat_e7 = _param_trk_home_lat.get();
		tracker_lon_e7 = _param_trk_home_lon.get();
		tracker_alt_m = static_cast<float>(_param_trk_home_alt.get()) * 0.001f;
		position_valid = true;
		_position_source = tracker_status_s::POSITION_SOURCE_HOME;
	}

	const uint64_t now_us = hrt_absolute_time();
	if (_tracker_target_sub.updated()) {
		tracker_target_position_s target{};
		_tracker_target_sub.copy(&target);
		_target_manager.update_from_mavlink(target, now_us);
	}

	const bool have_target = _target_manager.valid(now_us);
	const bool home_requested_unprovisioned = !position_valid && _param_trk_home_en.get() != 0
					 && !tracker_geo::valid_home_fallback(_param_trk_home_lat.get(), _param_trk_home_lon.get());

	if (_param_trk_alt_src.get() != 0) {
		_bearing_rad = 0.f;
		_pitch_rad = 0.f;
		_distance_m = 0.f;
		publish_safe_output(1, tracker_status_s::STATE_SENSOR_INVALID, have_target,
				    tracker_status_s::REASON_SOURCE_REJECTED);
		return;
	}

	if (!position_valid || !attitude_valid) {
		_bearing_rad = 0.f;
		_pitch_rad = 0.f;
		_distance_m = 0.f;
		const uint8_t reason = attitude_valid && home_requested_unprovisioned
				       ? tracker_status_s::REASON_HOME_UNPROVISIONED : tracker_status_s::REASON_SENSOR_INVALID;
		publish_safe_output(1, tracker_status_s::STATE_SENSOR_INVALID, have_target, reason);
		return;
	}

	if (!have_target) {
		_bearing_rad = 0.f;
		_pitch_rad = 0.f;
		_distance_m = 0.f;
		publish_safe_output(1, tracker_status_s::STATE_TIMEOUT, false);
		return;
	}

	if (!ensure_head_reference(attitude, hrt_absolute_time())) {
		publish_safe_output(1, tracker_status_s::STATE_SENSOR_INVALID, true,
				    tracker_status_s::REASON_REFERENCE_NOT_READY);
		return;
	}

	const TargetManager::Target &target = _target_manager.target();
	_distance_m = tracker_geo::horizontal_distance_m(tracker_lat_e7, tracker_lon_e7, target.lat_e7, target.lon_e7);
	_bearing_rad = tracker_geo::bearing_rad(tracker_lat_e7, tracker_lon_e7, target.lat_e7, target.lon_e7);
	_pitch_rad = tracker_geo::pitch_rad(static_cast<float>(target.alt_mm) * 0.001f - tracker_alt_m, _distance_m);

	if (_param_trk_dist_min.get() > 0.f && _distance_m < _param_trk_dist_min.get()) {
		reset_tracking_controller();
		publish_safe_output(1, tracker_status_s::STATE_TARGET_TOO_CLOSE, true);
		return;
	}

	const matrix::Eulerf euler{matrix::Quatf{attitude.q}};
	_yaw_measured_rad = euler.psi();
	_pitch_measured_rad = euler.theta();
	const auto command = _setpoint_planner.from_auto(_bearing_rad, _pitch_rad, _yaw_home_rad, _pitch_home_rad);
	_yaw_command_deg = command.yaw_deg;
	_pitch_command_deg = command.pitch_deg;
	_yaw_clipped = command.yaw_clipped;
	_pitch_clipped = command.pitch_clipped;
	_yaw_setpoint_rad = matrix::wrap_pi(_yaw_home_rad + math::radians(_yaw_command_deg - _setpoint_planner.yaw_park_deg()));
	_pitch_setpoint_rad = _pitch_home_rad + math::radians(_pitch_command_deg - _setpoint_planner.pitch_park_deg());
	_yaw_error_rad = matrix::wrap_pi(_yaw_setpoint_rad - _yaw_measured_rad);
	_pitch_error_rad = _pitch_setpoint_rad - _pitch_measured_rad;

	float yaw_target_rate = 0.f;
	float pitch_target_rate = 0.f;
	if (_prev_bearing_valid && dt > 1e-6f) {
		yaw_target_rate = math::constrain(matrix::wrap_pi(_bearing_rad - _prev_bearing_rad) / dt, -2.f * M_PI_F, 2.f * M_PI_F);
		pitch_target_rate = math::constrain((_pitch_rad - _prev_pitch_target_rad) / dt, -M_PI_F, M_PI_F);
	}
	_prev_bearing_rad = _bearing_rad;
	_prev_pitch_target_rad = _pitch_rad;
	_prev_bearing_valid = true;

	const float yaw_output = _yaw_axis_controller.command(_yaw_command_deg, _yaw_error_rad, yaw_target_rate, dt);
	const float pitch_output = _pitch_axis_controller.command(_pitch_command_deg, _pitch_error_rad, pitch_target_rate, dt);

	uint8_t reason = tracker_status_s::REASON_NONE;
	if (_yaw_clipped) {
		reason = tracker_status_s::REASON_YAW_LIMIT;
	} else if (_pitch_clipped) {
		reason = tracker_status_s::REASON_PITCH_LIMIT;
	}
	publish_servo_output(yaw_output, pitch_output, 1, tracker_status_s::STATE_TRACKING, true, reason);
}

void AntennaTracker::run_manual(float dt)
{
	manual_control_setpoint_s manual{};
	const bool manual_valid = _manual_control_sub.copy(&manual) && manual.valid && manual.timestamp > 0
				  && hrt_elapsed_time(&manual.timestamp) < 500_ms
				  && PX4_ISFINITE(manual.yaw) && PX4_ISFINITE(manual.pitch);

	if (!manual_valid) {
		publish_safe_output(2, tracker_status_s::STATE_SENSOR_INVALID, false);
		return;
	}

	const float yaw_input = math::constrain(manual.yaw, -1.f, 1.f);
	const float pitch_input = math::constrain(manual.pitch, -1.f, 1.f);

	const float yaw_angle = _setpoint_planner.yaw_min_deg()
				+ (yaw_input + 1.f) * 0.5f * (_setpoint_planner.yaw_max_deg() - _setpoint_planner.yaw_min_deg());
	const float pitch_angle = _setpoint_planner.pitch_min_deg()
				+ (pitch_input + 1.f) * 0.5f * (_setpoint_planner.pitch_max_deg() - _setpoint_planner.pitch_min_deg());
	const auto command = _setpoint_planner.constrain(yaw_angle, pitch_angle);
	_yaw_command_deg = command.yaw_deg;
	_pitch_command_deg = command.pitch_deg;
	_yaw_clipped = command.yaw_clipped;
	_pitch_clipped = command.pitch_clipped;
	_yaw_setpoint_rad = NAN;
	_pitch_setpoint_rad = NAN;
	_yaw_error_rad = 0.f;
	_pitch_error_rad = 0.f;
	publish_servo_output(_yaw_axis_controller.command_without_correction(command.yaw_deg, dt),
			     _pitch_axis_controller.command_without_correction(command.pitch_deg, dt),
			     2, tracker_status_s::STATE_MANUAL, false);
}

int AntennaTracker::task_spawn(int argc, char *argv[])
{
	AntennaTracker *instance = new AntennaTracker();
	if (instance) {
		_object.store(instance);
		_task_id = task_id_is_work_queue;
		if (instance->init()) {
			return PX4_OK;
		}
	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	_object.store(nullptr);
	_task_id = -1;
	return PX4_ERROR;
}

int AntennaTracker::custom_command(int argc, char *argv[])
{
	if (argc > 0 && !strcmp(argv[0], "set_home")) {
		vehicle_global_position_s global_position{};
		const int subscription = orb_subscribe(ORB_ID(vehicle_global_position));
		if (subscription < 0) {
			PX4_ERR("subscribe failed");
			return PX4_ERROR;
		}
		const int result = orb_copy(ORB_ID(vehicle_global_position), subscription, &global_position);
		orb_unsubscribe(subscription);
		if (result != PX4_OK || !is_global_position_valid(global_position)) {
			PX4_ERR("no valid position");
			return PX4_ERROR;
		}
		const int32_t enabled = 1;
		const int32_t latitude = static_cast<int32_t>(global_position.lat * 1e7);
		const int32_t longitude = static_cast<int32_t>(global_position.lon * 1e7);
		const int32_t altitude = static_cast<int32_t>(global_position.alt * 1000.f);
		bool failed = false;
		failed = failed || (param_set(param_find("TRK_HOME_EN"), &enabled) != PX4_OK);
		failed = failed || (param_set(param_find("TRK_HOME_LAT"), &latitude) != PX4_OK);
		failed = failed || (param_set(param_find("TRK_HOME_LON"), &longitude) != PX4_OK);
		failed = failed || (param_set(param_find("TRK_HOME_ALT"), &altitude) != PX4_OK);
		if (failed) {
			PX4_ERR("failed to set params");
			return PX4_ERROR;
		}
		PX4_INFO("home set: lat=%" PRId32 " lon=%" PRId32 " alt=%" PRId32 "mm", latitude, longitude, altitude);
		PX4_INFO("run 'param save' to persist");
		return PX4_OK;
	}
	return print_usage("unknown command");
}

int AntennaTracker::print_status()
{
	PX4_INFO("Control trigger: vehicle_attitude callback (%s)",
		 _vehicle_attitude_sub.registered() ? "registered" : "not registered");
	PX4_INFO("Control work queue: nav_and_controllers; actuator_servos publishes on each attitude update");
	PX4_INFO("PWM output rate is selected by the output driver (set it to 50 Hz for these servos)");
	const int requested_mode = determine_mode();
	const int effective_mode = operation_permitted() ? requested_mode : 0;
	const auto mode_string = [](int mode) {
		return mode == 0 ? "STOP" : (mode == 1 ? "AUTO" : "MANUAL");
	};
	PX4_INFO("Arming: %s%s", _actuator_armed.armed ? "ARMED" : "DISARMED",
		 safety_inhibited() ? " (safety inhibited)" : "");
	PX4_INFO("Requested mode: %s (TRK_MODE)", mode_string(requested_mode));
	PX4_INFO("Effective mode: %s", mode_string(effective_mode));
	const auto &target = _target_manager.target();
	PX4_INFO("Target: %s sysid=%u", _target_manager.valid(hrt_absolute_time()) ? "VALID" : "none", target.system_id);
	PX4_INFO("Bearing: %.1f deg Pitch: %.1f deg Dist: %.1f m", (double)math::degrees(_bearing_rad),
		 (double)math::degrees(_pitch_rad), (double)_distance_m);
	PX4_INFO("Command: yaw=%.1f deg pitch=%.1f deg Out: yaw=%.3f pitch=%.3f", (double)_yaw_command_deg,
		 (double)_pitch_command_deg, (double)_yaw_output, (double)_pitch_output);
	PX4_INFO("Limits: yaw=[%.1f, %.1f] park=%.1f pitch=[%.1f, %.1f] park=%.1f", (double)_setpoint_planner.yaw_min_deg(),
		 (double)_setpoint_planner.yaw_max_deg(), (double)_setpoint_planner.yaw_park_deg(),
		 (double)_setpoint_planner.pitch_min_deg(), (double)_setpoint_planner.pitch_max_deg(),
		 (double)_setpoint_planner.pitch_park_deg());
	PX4_INFO("Reference: %s", _head_reference_valid ? "captured at park" : "not captured");
	const char *position_source = _position_source == tracker_status_s::POSITION_SOURCE_GLOBAL ? "vehicle_global_position"
				      : (_position_source == tracker_status_s::POSITION_SOURCE_HOME ? "TRK_HOME fallback" : "none");
	PX4_INFO("Position source: %s", position_source);

	perf_print_counter(_loop_perf);
	perf_print_counter(_loop_interval_perf);
	return 0;
}

int AntennaTracker::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}
	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
PX4-native antenna tracker controller for positional yaw and pitch servos.

Tracker modes are selected with TRK_MODE: STOP, AUTO, or MANUAL.
The module does not reinterpret PX4 flight modes as tracker modes.

### Safety
The FC IMU must move with the antenna assembly. Set SENS_BOARD_ROT and run
level-horizon calibration before configuring mechanical servo trim.
)DESCR_STR");
	PRINT_MODULE_USAGE_NAME("antenna_tracker", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_COMMAND("set_home");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();
	return 0;
}

extern "C" __EXPORT int antenna_tracker_main(int argc, char *argv[])
{
	return AntennaTracker::main(argc, argv);
}
