#pragma once

#include <px4_platform_common/px4_config.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/posix.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/perf/perf_counter.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/tracker_target_position.h>
#include <uORB/topics/tracker_status.h>

#include "tracker_geo.hpp"
#include "tracker_controller.hpp"

using namespace time_literals;

class AntennaTracker : public ModuleBase<AntennaTracker>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	AntennaTracker();
	~AntennaTracker() override;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	bool init();

	int print_status() override;

private:
	void Run() override;

	/**
	 * Update parameters from the parameter store.
	 */
	void parameters_update();

	/**
	 * Run servo test sweep mode.
	 */
	void run_servo_test(float elapsed_s);

	/**
	 * Run tracking mode: compute geometry, PID, publish servo.
	 */
	void run_tracking(float dt);

	// Subscriptions
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_global_position_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _tracker_target_sub{ORB_ID(tracker_target_position)};

	// Publications
	uORB::Publication<actuator_servos_s> _actuator_servos_pub{ORB_ID(actuator_servos)};
	uORB::Publication<tracker_status_s>  _tracker_status_pub{ORB_ID(tracker_status)};

	// Controllers
	TrackerPID _yaw_pid;
	TrackerPID _pitch_pid;

	// State
	bool     _target_valid{false};
	uint8_t  _target_sysid{0};
	uint64_t _last_target_update_us{0};

	// Tracking outputs (for status display)
	float _bearing_rad{0.f};
	float _pitch_rad{0.f};
	float _distance_m{0.f};
	float _yaw_error_rad{0.f};
	float _pitch_error_rad{0.f};
	float _yaw_output{0.f};
	float _pitch_output{0.f};

	// Servo test state
	float _servo_test_phase{0.f};

	// Perf counters
	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	// Timestamps
	uint64_t _last_run_us{0};

	// --- Parameters ---
	DEFINE_PARAMETERS(
		(ParamInt<px4::params::TRK_MODE>)        _param_trk_mode,
		(ParamInt<px4::params::TRK_SERVO_TEST>)  _param_trk_servo_test,
		(ParamFloat<px4::params::TRK_YAW_P>)    _param_trk_yaw_p,
		(ParamFloat<px4::params::TRK_YAW_I>)    _param_trk_yaw_i,
		(ParamFloat<px4::params::TRK_YAW_D>)    _param_trk_yaw_d,
		(ParamFloat<px4::params::TRK_PIT_P>)    _param_trk_pit_p,
		(ParamFloat<px4::params::TRK_PIT_I>)    _param_trk_pit_i,
		(ParamFloat<px4::params::TRK_PIT_D>)    _param_trk_pit_d,
		(ParamFloat<px4::params::TRK_YAW_TRIM>) _param_trk_yaw_trim,
		(ParamFloat<px4::params::TRK_PIT_TRIM>) _param_trk_pit_trim,
		(ParamFloat<px4::params::TRK_YAW_MIN>)  _param_trk_yaw_min,
		(ParamFloat<px4::params::TRK_YAW_MAX>)  _param_trk_yaw_max,
		(ParamFloat<px4::params::TRK_PIT_MIN>)  _param_trk_pit_min,
		(ParamFloat<px4::params::TRK_PIT_MAX>)  _param_trk_pit_max,
		(ParamInt<px4::params::TRK_TGT_LAT>)    _param_trk_tgt_lat,
		(ParamInt<px4::params::TRK_TGT_LON>)    _param_trk_tgt_lon,
		(ParamInt<px4::params::TRK_TGT_ALT>)    _param_trk_tgt_alt,
		(ParamInt<px4::params::TRK_SYSID_TGT>)  _param_trk_sysid_tgt,
		(ParamInt<px4::params::TRK_AUTO_LOCK>)   _param_trk_auto_lock,
		(ParamInt<px4::params::TRK_TIMEOUT_MS>)  _param_trk_timeout_ms
	)
};
