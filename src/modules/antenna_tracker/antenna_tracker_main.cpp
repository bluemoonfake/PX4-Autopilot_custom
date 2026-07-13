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
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
	_boot_time_us = hrt_absolute_time();
}

AntennaTracker::~AntennaTracker()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool AntennaTracker::init()
{
	parameters_update();
	ScheduleOnInterval(20_ms);
	return true;
}

int AntennaTracker::determine_mode()
{
	return math::constrain(_param_trk_mode.get(), int32_t{0}, int32_t{3});
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
}

bool AntennaTracker::ensure_head_reference(const vehicle_attitude_s &attitude, uint64_t now_us)
{
	if (_head_reference_valid) {
		return true;
	}

	if ((now_us - _boot_time_us) < 1_s) {
		return false;
	}

	const matrix::Eulerf euler{matrix::Quatf{attitude.q}};
	_yaw_home_rad = euler.psi();
	_pitch_home_rad = euler.theta();
	_head_reference_valid = true;
	PX4_INFO("tracker head reference captured at park");
	return true;
}

void AntennaTracker::reset_scan_state()
{
	const auto park = _setpoint_planner.park();
	_scan_yaw_deg = park.yaw_deg;
	_scan_pitch_deg = park.pitch_deg;
	_scan_reverse_yaw = false;
	_scan_reverse_pitch = false;
}

void AntennaTracker::handle_mode_change(int new_mode)
{
	if (new_mode == _active_mode) {
		return;
	}

	reset_tracking_controller();
	_auto_scan_active = false;
	invalidate_head_reference();

	if (new_mode == 2) {
		reset_scan_state();
	}

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
	case tracker_status_s::STATE_IDLE:
		return _startup_delay_done ? tracker_status_s::REASON_STOP : tracker_status_s::REASON_STARTUP;
	default:
		return tracker_status_s::REASON_NONE;
	}
}

void AntennaTracker::publish_servo_output(float yaw_output, float pitch_output, int mode, uint8_t state,
		bool target_valid, uint8_t state_reason)
{
	_yaw_output = math::constrain(yaw_output, _param_trk_yaw_min.get(), _param_trk_yaw_max.get());
	_pitch_output = math::constrain(pitch_output, _param_trk_pit_min.get(), _param_trk_pit_max.get());
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
	status.scan_yaw_deg = _scan_yaw_deg;
	status.scan_pitch_deg = _scan_pitch_deg;
	status.yaw_setpoint_rad = _yaw_setpoint_rad;
	status.pitch_setpoint_rad = _pitch_setpoint_rad;
	status.yaw_measured_rad = _yaw_measured_rad;
	status.pitch_measured_rad = _pitch_measured_rad;
	status.yaw_command_deg = _yaw_command_deg;
	status.pitch_command_deg = _pitch_command_deg;
	status.yaw_clipped = _yaw_clipped;
	status.pitch_clipped = _pitch_clipped;
	_tracker_status_pub.publish(status);
	_events.update(state, state_reason, target_valid, _yaw_clipped, _pitch_clipped);
}

void AntennaTracker::publish_safe_output(int mode, uint8_t state, bool target_valid)
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

	publish_servo_output(yaw_output, pitch_output, mode, state, target_valid, safe_reason_for_state(state));
}

void AntennaTracker::Run()
{
	if (should_exit()) {
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

	const uint64_t now_us = hrt_absolute_time();
	float dt = 0.02f;

	if (_last_run_us > 0) {
		dt = math::constrain((now_us - _last_run_us) * 1e-6f, 0.001f, 0.1f);
	}

	_last_run_us = now_us;

	if (_param_trk_strt_dly.get() > 0.f && !_startup_delay_done) {
		if ((now_us - _boot_time_us) < static_cast<uint64_t>(_param_trk_strt_dly.get() * 1e6f)) {
			publish_safe_output(0, tracker_status_s::STATE_IDLE);
			perf_end(_loop_perf);
			return;
		}

		_startup_delay_done = true;
	}

	if (_param_trk_servo_test.get() != 0) {
		_active_mode = -1;
		reset_tracking_controller();
		invalidate_head_reference();
		run_servo_test(dt);
		perf_end(_loop_perf);
		return;
	}

	const int active_mode = determine_mode();
	handle_mode_change(active_mode);

	switch (active_mode) {
	case 0:
		publish_safe_output(0, tracker_status_s::STATE_IDLE);
		break;
	case 1:
		run_tracking(dt);
		break;
	case 2:
		run_scan(dt);
		break;
	case 3:
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

	if (_configured_target_sysid != _param_trk_sysid_tgt.get()
	    || _configured_auto_lock != _param_trk_auto_lock.get()) {
		_configured_target_sysid = _param_trk_sysid_tgt.get();
		_configured_auto_lock = _param_trk_auto_lock.get();
		_target_manager.reset();
		reset_tracking_controller();
	}

	const float legacy_half_range = math::max(_param_trk_yaw_range.get(), 0.f) * 0.5f;
	const float yaw_min = _param_trk_yaw_mind.get() < _param_trk_yaw_maxd.get()
			      ? _param_trk_yaw_mind.get() : -legacy_half_range;
	const float yaw_max = _param_trk_yaw_mind.get() < _param_trk_yaw_maxd.get()
			      ? _param_trk_yaw_maxd.get() : legacy_half_range;

	_setpoint_planner.configure(yaw_min, yaw_max, _param_trk_yaw_park.get(),
				   _param_trk_pit_mind.get(), _param_trk_pit_maxd.get(), _param_trk_pit_park.get());
	_yaw_servo_mapper.configure(_setpoint_planner.yaw_min_deg(), _setpoint_planner.yaw_max_deg(),
				    _param_trk_yaw_min.get(), _param_trk_yaw_max.get(), _param_trk_yaw_trim.get(),
				    _param_trk_yaw_rev.get() != 0);
	_pitch_servo_mapper.configure(_setpoint_planner.pitch_min_deg(), _setpoint_planner.pitch_max_deg(),
				      _param_trk_pit_min.get(), _param_trk_pit_max.get(), _param_trk_pit_trim.get(),
				      _param_trk_pit_rev.get() != 0);

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
}

void AntennaTracker::run_servo_test(float elapsed_s)
{
	_servo_test_phase += elapsed_s;
	if (_servo_test_phase > 4.f) {
		_servo_test_phase -= 4.f;
	}

	float value = 0.f;
	if (_servo_test_phase < 1.f) {
		value = -0.5f + _servo_test_phase * 0.5f;
	} else if (_servo_test_phase < 2.f) {
		value = (_servo_test_phase - 1.f) * 0.5f;
	} else if (_servo_test_phase < 3.f) {
		value = 0.5f - (_servo_test_phase - 2.f) * 0.5f;
	} else {
		value = -(_servo_test_phase - 3.f) * 0.5f;
	}

	value = math::constrain(value, -0.5f, 0.5f);
	const float yaw_angle = _yaw_servo_mapper.angle_for_output(value);
	const float pitch_angle = _pitch_servo_mapper.angle_for_output(value);
	_yaw_command_deg = yaw_angle;
	_pitch_command_deg = pitch_angle;
	publish_servo_output(_yaw_axis_controller.command_without_correction(yaw_angle, elapsed_s),
			     _pitch_axis_controller.command_without_correction(pitch_angle, elapsed_s),
			     determine_mode(), tracker_status_s::STATE_SERVO_TEST, false);
}

void AntennaTracker::run_tracking(float dt)
{
	vehicle_attitude_s attitude{};
	const bool attitude_valid = _vehicle_attitude_sub.copy(&attitude) && is_attitude_valid(attitude);

	vehicle_global_position_s global_position{};
	bool position_valid = _vehicle_global_position_sub.copy(&global_position) && is_global_position_valid(global_position);
	int32_t tracker_lat_e7 = 0;
	int32_t tracker_lon_e7 = 0;
	float tracker_alt_m = 0.f;

	if (position_valid) {
		tracker_lat_e7 = static_cast<int32_t>(global_position.lat * 1e7);
		tracker_lon_e7 = static_cast<int32_t>(global_position.lon * 1e7);
		tracker_alt_m = global_position.alt;

	} else if (_param_trk_home_en.get() != 0
		   && _param_trk_home_lat.get() >= -900000000 && _param_trk_home_lat.get() <= 900000000
		   && _param_trk_home_lon.get() >= -1800000000 && _param_trk_home_lon.get() <= 1800000000) {
		tracker_lat_e7 = _param_trk_home_lat.get();
		tracker_lon_e7 = _param_trk_home_lon.get();
		tracker_alt_m = static_cast<float>(_param_trk_home_alt.get()) * 0.001f;
		position_valid = true;
	}

	const uint64_t now_us = hrt_absolute_time();

	if (_tracker_target_sub.updated()) {
		tracker_target_position_s target{};
		_tracker_target_sub.copy(&target);
		_target_manager.update_from_mavlink(target, now_us);
	}

	if ((_target_manager.target().fake || !_target_manager.valid(now_us))
	    && (_param_trk_tgt_lat.get() != 0 || _param_trk_tgt_lon.get() != 0 || _param_trk_tgt_alt.get() != 0)) {
		_target_manager.set_fake_target(_param_trk_tgt_lat.get(), _param_trk_tgt_lon.get(), _param_trk_tgt_alt.get(), now_us);
	}

	const bool have_target = _target_manager.valid(now_us);

	if (!have_target && _param_trk_auto_scan.get() != 0) {
		if (!_auto_scan_active) {
			reset_tracking_controller();
			reset_scan_state();
			_auto_scan_active = true;
		}
		run_scan(dt);
		return;
	}

	if (have_target && _auto_scan_active) {
		reset_tracking_controller();
		_auto_scan_active = false;
	}

	if (!have_target || !position_valid || !attitude_valid) {
		reset_tracking_controller();
		_bearing_rad = 0.f;
		_pitch_rad = 0.f;
		_distance_m = 0.f;
		publish_safe_output(1, have_target ? tracker_status_s::STATE_SENSOR_INVALID : tracker_status_s::STATE_TIMEOUT,
				    have_target);
		return;
	}

	if (!ensure_head_reference(attitude, hrt_absolute_time())) {
		publish_safe_output(1, tracker_status_s::STATE_SENSOR_INVALID, true);
		return;
	}

	const TrackerTargetManager::Target &target = _target_manager.target();
	int32_t target_lat_e7 = target.lat_e7;
	int32_t target_lon_e7 = target.lon_e7;
	int32_t target_alt_mm = target.alt_mm;

	if (_param_trk_deadreck.get() != 0 && target.velocity_valid && target.last_update_us > 0) {
		const float target_dt = (now_us - target.last_update_us) * 1e-6f;
		if (target_dt > 0.f && target_dt < 5.f) {
			const double latitude_rad = static_cast<double>(tracker_lat_e7) * 1e-7 * M_PI / 180.0;
			const double longitude_scale = cos(latitude_rad);
			const double scaling = tracker_geo::LOCATION_SCALING_FACTOR;
			target_lat_e7 += static_cast<int32_t>(static_cast<double>(target.vx_m_s * target_dt) / scaling);
			if (longitude_scale > 0.01) {
				target_lon_e7 += static_cast<int32_t>(static_cast<double>(target.vy_m_s * target_dt)
								   / (scaling * longitude_scale));
			}
			target_alt_mm += static_cast<int32_t>(-target.vz_m_s * target_dt * 1000.f);
		}
	}

	_distance_m = tracker_geo::horizontal_distance_m(tracker_lat_e7, tracker_lon_e7, target_lat_e7, target_lon_e7);
	_bearing_rad = tracker_geo::bearing_rad(tracker_lat_e7, tracker_lon_e7, target_lat_e7, target_lon_e7);
	_pitch_rad = tracker_geo::pitch_rad(static_cast<float>(target_alt_mm) * 0.001f - tracker_alt_m, _distance_m);

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

void AntennaTracker::run_scan(float dt)
{
	const float yaw_delta = _param_trk_scan_yspd.get() * dt;
	if (_scan_reverse_yaw) {
		_scan_yaw_deg -= yaw_delta;
		if (_scan_yaw_deg < _setpoint_planner.yaw_min_deg()) {
			_scan_yaw_deg = _setpoint_planner.yaw_min_deg();
			_scan_reverse_yaw = false;
		}
	} else {
		_scan_yaw_deg += yaw_delta;
		if (_scan_yaw_deg > _setpoint_planner.yaw_max_deg()) {
			_scan_yaw_deg = _setpoint_planner.yaw_max_deg();
			_scan_reverse_yaw = true;
		}
	}

	const float pitch_delta = _param_trk_scan_pspd.get() * dt;
	if (_scan_reverse_pitch) {
		_scan_pitch_deg -= pitch_delta;
		if (_scan_pitch_deg < _setpoint_planner.pitch_min_deg()) {
			_scan_pitch_deg = _setpoint_planner.pitch_min_deg();
			_scan_reverse_pitch = false;
		}
	} else {
		_scan_pitch_deg += pitch_delta;
		if (_scan_pitch_deg > _setpoint_planner.pitch_max_deg()) {
			_scan_pitch_deg = _setpoint_planner.pitch_max_deg();
			_scan_reverse_pitch = true;
		}
	}

	const auto command = _setpoint_planner.constrain(_scan_yaw_deg, _scan_pitch_deg);
	_scan_yaw_deg = command.yaw_deg;
	_scan_pitch_deg = command.pitch_deg;
	_yaw_command_deg = command.yaw_deg;
	_pitch_command_deg = command.pitch_deg;
	_yaw_clipped = command.yaw_clipped;
	_pitch_clipped = command.pitch_clipped;
	const float yaw_output = _yaw_axis_controller.command_without_correction(command.yaw_deg, dt);
	const float pitch_output = _pitch_axis_controller.command_without_correction(command.pitch_deg, dt);
	publish_servo_output(yaw_output, pitch_output, 2, tracker_status_s::STATE_SCANNING, false);
}

void AntennaTracker::run_manual(float dt)
{
	manual_control_setpoint_s manual{};
	float yaw_input = 0.f;
	float pitch_input = 0.f;

	if (_manual_control_sub.copy(&manual) && manual.valid && manual.timestamp > 0
	    && hrt_elapsed_time(&manual.timestamp) < 500_ms
	    && PX4_ISFINITE(manual.yaw) && PX4_ISFINITE(manual.pitch)) {
		yaw_input = math::constrain(manual.yaw, -1.f, 1.f);
		pitch_input = math::constrain(manual.pitch, -1.f, 1.f);
	}

	const float yaw_angle = _setpoint_planner.yaw_min_deg()
				+ (yaw_input + 1.f) * 0.5f * (_setpoint_planner.yaw_max_deg() - _setpoint_planner.yaw_min_deg());
	const float pitch_angle = _setpoint_planner.pitch_min_deg()
				+ (pitch_input + 1.f) * 0.5f * (_setpoint_planner.pitch_max_deg() - _setpoint_planner.pitch_min_deg());
	const auto command = _setpoint_planner.constrain(yaw_angle, pitch_angle);
	_yaw_command_deg = command.yaw_deg;
	_pitch_command_deg = command.pitch_deg;
	_yaw_clipped = command.yaw_clipped;
	_pitch_clipped = command.pitch_clipped;
	const float yaw_output = _yaw_axis_controller.command_without_correction(command.yaw_deg, dt);
	const float pitch_output = _pitch_axis_controller.command_without_correction(command.pitch_deg, dt);
	publish_servo_output(yaw_output, pitch_output, 3, tracker_status_s::STATE_MANUAL, false);
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
	const int mode = determine_mode();
	const char *mode_string = mode == 0 ? "STOP" : (mode == 1 ? "AUTO" : (mode == 2 ? "SCAN" : "MANUAL"));
	PX4_INFO("Mode: %s (TRK_MODE)", mode_string);
	const auto &target = _target_manager.target();
	PX4_INFO("Target: %s sysid=%u compid=%u", _target_manager.valid(hrt_absolute_time()) ? "VALID" : "none",
		 target.system_id, target.component_id);
	PX4_INFO("Bearing: %.1f deg Pitch: %.1f deg Dist: %.1f m", (double)math::degrees(_bearing_rad),
		 (double)math::degrees(_pitch_rad), (double)_distance_m);
	PX4_INFO("Command: yaw=%.1f deg pitch=%.1f deg Out: yaw=%.3f pitch=%.3f", (double)_yaw_command_deg,
		 (double)_pitch_command_deg, (double)_yaw_output, (double)_pitch_output);
	PX4_INFO("Limits: yaw=[%.1f, %.1f] park=%.1f pitch=[%.1f, %.1f] park=%.1f", (double)_setpoint_planner.yaw_min_deg(),
		 (double)_setpoint_planner.yaw_max_deg(), (double)_setpoint_planner.yaw_park_deg(),
		 (double)_setpoint_planner.pitch_min_deg(), (double)_setpoint_planner.pitch_max_deg(),
		 (double)_setpoint_planner.pitch_park_deg());
	PX4_INFO("Reference: %s", _head_reference_valid ? "captured at park" : "not captured");
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

Tracker modes are selected with TRK_MODE: STOP, AUTO, SCAN, or MANUAL.
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
