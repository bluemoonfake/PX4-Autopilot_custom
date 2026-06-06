#include "antenna_tracker.hpp"

#include <drivers/drv_hrt.h>
#include <lib/mathlib/mathlib.h>
#include <matrix/math.hpp>

using namespace time_literals;

AntennaTracker::AntennaTracker() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::hp_default)
{
}

AntennaTracker::~AntennaTracker()
{
	perf_free(_loop_perf);
	perf_free(_loop_interval_perf);
}

bool AntennaTracker::init()
{
	ScheduleOnInterval(20000); // 50 Hz
	return true;
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

	// --- Parameter update ---
	if (_parameter_update_sub.updated()) {
		parameter_update_s param_update;
		_parameter_update_sub.copy(&param_update);
		parameters_update();
	}

	// --- Compute dt ---
	const uint64_t now_us = hrt_absolute_time();
	float dt = 0.02f; // default 50 Hz

	if (_last_run_us > 0) {
		dt = math::constrain((now_us - _last_run_us) * 1e-6f, 0.001f, 0.1f);
	}

	_last_run_us = now_us;

	// --- Servo test mode ---
	if (_param_trk_servo_test.get() != 0) {
		run_servo_test(dt);
		perf_end(_loop_perf);
		return;
	}

	// --- Mode: STOP ---
	if (_param_trk_mode.get() == 0) {
		// Center servos, reset PID
		actuator_servos_s servos{};
		servos.timestamp = hrt_absolute_time();
		servos.timestamp_sample = servos.timestamp;

		for (int i = 0; i < 8; i++) {
			servos.control[i] = NAN;
		}

		servos.control[0] = 0.f; // yaw centered
		servos.control[1] = 0.f; // pitch centered
		_actuator_servos_pub.publish(servos);

		_yaw_pid.reset();
		_pitch_pid.reset();
		_target_valid = false;

		// Publish status
		tracker_status_s status{};
		status.timestamp = hrt_absolute_time();
		status.target_valid = false;
		status.mode = 0;
		status.state = tracker_status_s::STATE_IDLE;
		_tracker_status_pub.publish(status);

		perf_end(_loop_perf);
		return;
	}

	// --- Mode: AUTO tracking ---
	if (_param_trk_mode.get() == 1) {
		run_tracking(dt);
	}

	perf_end(_loop_perf);
}

void AntennaTracker::parameters_update()
{
	updateParams();

	// Apply PID gains
	_yaw_pid.set_gains(_param_trk_yaw_p.get(), _param_trk_yaw_i.get(), _param_trk_yaw_d.get());
	_yaw_pid.set_output_limits(_param_trk_yaw_min.get(), _param_trk_yaw_max.get());

	_pitch_pid.set_gains(_param_trk_pit_p.get(), _param_trk_pit_i.get(), _param_trk_pit_d.get());
	_pitch_pid.set_output_limits(_param_trk_pit_min.get(), _param_trk_pit_max.get());
}

void AntennaTracker::run_servo_test(float elapsed_s)
{
	// Sweep: triangle wave over 4 seconds, range [-0.5, +0.5]
	_servo_test_phase += elapsed_s;

	if (_servo_test_phase > 4.f) {
		_servo_test_phase -= 4.f;
	}

	float val;

	if (_servo_test_phase < 1.f) {
		val = -0.5f + _servo_test_phase * 1.0f;      // -0.5 → +0.5

	} else if (_servo_test_phase < 3.f) {
		val = 0.5f - (_servo_test_phase - 1.f) * 1.0f; // +0.5 → -1.5 → clamped

	} else {
		val = -0.5f + (_servo_test_phase - 3.f) * 1.0f; // -0.5 → +0.5
	}

	val = math::constrain(val, -0.5f, 0.5f);

	actuator_servos_s servos{};
	servos.timestamp = hrt_absolute_time();
	servos.timestamp_sample = servos.timestamp;

	for (int i = 0; i < 8; i++) {
		servos.control[i] = NAN;
	}

	servos.control[0] = val; // yaw
	servos.control[1] = val; // pitch
	_actuator_servos_pub.publish(servos);

	// Publish status
	tracker_status_s status{};
	status.timestamp = hrt_absolute_time();
	status.target_valid = false;
	status.yaw_output = val;
	status.pitch_output = val;
	status.mode = _param_trk_mode.get();
	status.state = tracker_status_s::STATE_SERVO_TEST;
	_tracker_status_pub.publish(status);
}

void AntennaTracker::run_tracking(float dt)
{
	// --- Get tracker attitude (yaw) ---
	vehicle_attitude_s attitude{};
	bool att_valid = false;

	if (_vehicle_attitude_sub.copy(&attitude)) {
		att_valid = true;
	}

	// --- Get tracker global position ---
	vehicle_global_position_s gpos{};
	bool gpos_valid = false;

	if (_vehicle_global_position_sub.copy(&gpos)) {
		gpos_valid = (fabs(gpos.lat) > 1e-6 || fabs(gpos.lon) > 1e-6);
	}

	// --- Determine target position ---
	int32_t target_lat_e7 = 0;
	int32_t target_lon_e7 = 0;
	int32_t target_alt_mm = 0;
	bool have_target = false;

	// 1) Check MAVLink target first
	if (_tracker_target_sub.updated()) {
		tracker_target_position_s tgt{};
		_tracker_target_sub.copy(&tgt);

		if (tgt.valid) {
			target_lat_e7 = tgt.lat;
			target_lon_e7 = tgt.lon;
			target_alt_mm = tgt.alt_mm;
			_last_target_update_us = tgt.last_update_us;
			_target_sysid = tgt.target_system;
			have_target = true;
		}

	} else if (_target_valid && _last_target_update_us > 0) {
		// Use last known MAVLink target if still valid
		tracker_target_position_s tgt{};
		_tracker_target_sub.copy(&tgt);

		if (tgt.valid) {
			target_lat_e7 = tgt.lat;
			target_lon_e7 = tgt.lon;
			target_alt_mm = tgt.alt_mm;
			have_target = true;
		}
	}

	// 2) Fall back to fake target from parameters
	if (!have_target && _param_trk_tgt_lat.get() != 0) {
		target_lat_e7 = _param_trk_tgt_lat.get();
		target_lon_e7 = _param_trk_tgt_lon.get();
		target_alt_mm = _param_trk_tgt_alt.get();
		_last_target_update_us = hrt_absolute_time(); // fake target never times out
		have_target = true;
	}

	// --- Timeout check ---
	if (have_target && _param_trk_timeout_ms.get() > 0) {
		const uint64_t elapsed_us = hrt_absolute_time() - _last_target_update_us;

		if (elapsed_us > static_cast<uint64_t>(_param_trk_timeout_ms.get()) * 1000ULL) {
			have_target = false;
		}
	}

	_target_valid = have_target;

	// --- If no valid target or no tracker position: safe output ---
	if (!have_target || !gpos_valid || !att_valid) {
		_yaw_pid.reset();
		_pitch_pid.reset();

		actuator_servos_s servos{};
		servos.timestamp = hrt_absolute_time();
		servos.timestamp_sample = servos.timestamp;

		for (int i = 0; i < 8; i++) {
			servos.control[i] = NAN;
		}

		servos.control[0] = 0.f;
		servos.control[1] = 0.f;
		_actuator_servos_pub.publish(servos);

		tracker_status_s status{};
		status.timestamp = hrt_absolute_time();
		status.target_valid = false;
		status.mode = 1;
		status.state = have_target ? tracker_status_s::STATE_TRACKING : tracker_status_s::STATE_TIMEOUT;
		_tracker_status_pub.publish(status);
		return;
	}

	// --- Convert tracker position to degE7 ---
	const int32_t tracker_lat_e7 = static_cast<int32_t>(gpos.lat * 1e7);
	const int32_t tracker_lon_e7 = static_cast<int32_t>(gpos.lon * 1e7);
	const float tracker_alt_m = gpos.alt; // meters MSL

	// --- Geometry computations ---
	_distance_m = tracker_geo::horizontal_distance_m(tracker_lat_e7, tracker_lon_e7,
			target_lat_e7, target_lon_e7);

	_bearing_rad = tracker_geo::bearing_rad(tracker_lat_e7, tracker_lon_e7,
						target_lat_e7, target_lon_e7);

	const float target_alt_m = static_cast<float>(target_alt_mm) * 0.001f;
	const float delta_alt_m = target_alt_m - tracker_alt_m;
	_pitch_rad = tracker_geo::pitch_rad(delta_alt_m, _distance_m);

	// --- Get tracker yaw from attitude quaternion ---
	const matrix::Quatf q(attitude.q);
	const matrix::Eulerf euler(q);
	float tracker_yaw_rad = euler.psi(); // yaw in [-pi, pi]

	// Normalize tracker yaw to [0, 2*pi)
	if (tracker_yaw_rad < 0.f) {
		tracker_yaw_rad += 2.f * static_cast<float>(M_PI);
	}

	// --- Compute errors ---
	_yaw_error_rad = tracker_geo::wrap_pi(_bearing_rad - tracker_yaw_rad);

	// For pitch: assume tracker pitch reference is horizontal (0 = level)
	// target pitch is the desired elevation
	const float tracker_pitch_rad = euler.theta();
	_pitch_error_rad = _pitch_rad - tracker_pitch_rad;

	// --- PID ---
	_yaw_output = _yaw_pid.update(_yaw_error_rad, dt) + _param_trk_yaw_trim.get();
	_pitch_output = _pitch_pid.update(_pitch_error_rad, dt) + _param_trk_pit_trim.get();

	_yaw_output = math::constrain(_yaw_output, _param_trk_yaw_min.get(), _param_trk_yaw_max.get());
	_pitch_output = math::constrain(_pitch_output, _param_trk_pit_min.get(), _param_trk_pit_max.get());

	// --- Publish servo commands ---
	actuator_servos_s servos{};
	servos.timestamp = hrt_absolute_time();
	servos.timestamp_sample = servos.timestamp;

	for (int i = 0; i < 8; i++) {
		servos.control[i] = NAN;
	}

	servos.control[0] = _yaw_output;
	servos.control[1] = _pitch_output;
	_actuator_servos_pub.publish(servos);

	// --- Publish tracker status ---
	tracker_status_s status{};
	status.timestamp = hrt_absolute_time();
	status.target_valid = _target_valid;
	status.target_system = _target_sysid;
	status.bearing_rad = _bearing_rad;
	status.pitch_rad = _pitch_rad;
	status.distance_m = _distance_m;
	status.yaw_error_rad = _yaw_error_rad;
	status.pitch_error_rad = _pitch_error_rad;
	status.yaw_output = _yaw_output;
	status.pitch_output = _pitch_output;
	status.mode = 1; // AUTO
	status.state = tracker_status_s::STATE_TRACKING;
	_tracker_status_pub.publish(status);
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
	return print_usage("unknown command");
}

int AntennaTracker::print_status()
{
	PX4_INFO("Antenna Tracker Status:");
	PX4_INFO("  Mode:           %s", _param_trk_mode.get() == 0 ? "STOP" :
		 (_param_trk_mode.get() == 1 ? "AUTO" : "SCAN"));
	PX4_INFO("  Servo Test:     %s", _param_trk_servo_test.get() ? "ENABLED" : "disabled");
	PX4_INFO("  Target Valid:   %s", _target_valid ? "YES" : "no");
	PX4_INFO("  Target SysID:   %u", _target_sysid);
	PX4_INFO("  Bearing:        %.1f deg", static_cast<double>(_bearing_rad * 180.f / M_PI_F));
	PX4_INFO("  Pitch:          %.1f deg", static_cast<double>(_pitch_rad * 180.f / M_PI_F));
	PX4_INFO("  Distance:       %.1f m", static_cast<double>(_distance_m));
	PX4_INFO("  Yaw Error:      %.2f deg", static_cast<double>(_yaw_error_rad * 180.f / M_PI_F));
	PX4_INFO("  Pitch Error:    %.2f deg", static_cast<double>(_pitch_error_rad * 180.f / M_PI_F));
	PX4_INFO("  Yaw Output:     %.3f", static_cast<double>(_yaw_output));
	PX4_INFO("  Pitch Output:   %.3f", static_cast<double>(_pitch_output));

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
PX4-native antenna tracker controller module.

Tracks a target UAV by receiving its position via MAVLink or
from fake target parameters, computing bearing and pitch angles,
running PID controllers, and publishing normalized servo commands.

### Examples
Start tracking with fake target:
$ param set TRK_MODE 1
$ param set TRK_TGT_LAT 473977420
$ param set TRK_TGT_LON 85455940
$ param set TRK_TGT_ALT 500000
$ antenna_tracker start

Start servo test:
$ param set TRK_SERVO_TEST 1
$ antenna_tracker start
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("antenna_tracker", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int antenna_tracker_main(int argc, char *argv[])
{
	return AntennaTracker::main(argc, argv);
}
