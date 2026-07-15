#pragma once

#include <cstdint>

#include <px4_platform_common/defines.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>
#include <lib/perf/perf_counter.h>
#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/uORB.h>
#include <uORB/topics/actuator_servos.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/tracker_status.h>
#include <uORB/topics/tracker_target_position.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_global_position.h>

#include "tracker_axis_controller.hpp"
#include "tracker_geo.hpp"
#include "tracker_servo_mapper.hpp"
#include "tracker_setpoint_planner.hpp"
#include "tracker_target_manager.hpp"
#include "tracker_events.hpp"

using namespace time_literals;

class AntennaTracker : public ModuleBase<AntennaTracker>, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	AntennaTracker();
	~AntennaTracker() override;

	static int task_spawn(int argc, char *argv[]);
	static int custom_command(int argc, char *argv[]);
	static int print_usage(const char *reason = nullptr);

	bool init();
	int print_status() override;

private:
	void Run() override;
	void parameters_update();

	/** Returns the explicit tracker submode: STOP, AUTO, SCAN, or MANUAL. */
	int determine_mode();
	void handle_mode_change(int new_mode);
	void reset_tracking_controller();
	void reset_scan_state();
	void process_servo_test_request();
	void stop_servo_test();
	void run_servo_test(float elapsed_s, uint8_t axis);
	void run_tracking(float dt);
	void run_scan(float dt);
	void run_manual(float dt);

	static bool is_global_position_valid(const vehicle_global_position_s &gpos);
	static bool is_attitude_valid(const vehicle_attitude_s &attitude);

	void publish_safe_output(int mode, uint8_t state, bool target_valid = false,
				 uint8_t state_reason = tracker_status_s::REASON_NONE);
	void publish_servo_output(float yaw_output, float pitch_output, int mode, uint8_t state,
				 bool target_valid, uint8_t state_reason = tracker_status_s::REASON_NONE);
	bool ensure_head_reference(const vehicle_attitude_s &attitude, uint64_t now_us);
	void invalidate_head_reference();
	void update_park_settle_time(uint64_t now_us, float yaw_output, float pitch_output);
	uint8_t safe_reason_for_state(uint8_t state) const;

	struct ControlConfiguration {
		float yaw_p;
		float yaw_i;
		float yaw_d;
		float yaw_ff;
		float yaw_imax;
		float pitch_p;
		float pitch_i;
		float pitch_d;
		float pitch_ff;
		float pitch_imax;
		float yaw_trim;
		float pitch_trim;
		float yaw_output_min;
		float yaw_output_max;
		float pitch_output_min;
		float pitch_output_max;
		float yaw_range;
		float yaw_min_deg;
		float yaw_max_deg;
		float yaw_park_deg;
		int32_t yaw_reverse;
		float pitch_min_deg;
		float pitch_max_deg;
		float pitch_park_deg;
		int32_t pitch_reverse;
		float yaw_slew;
		float pitch_slew;
		float reference_settle;

		bool equals(const ControlConfiguration &other) const;
	};

	ControlConfiguration control_configuration() const;

	// Subscriptions
	uORB::Subscription _vehicle_attitude_sub{ORB_ID(vehicle_attitude)};
	uORB::Subscription _vehicle_global_position_sub{ORB_ID(vehicle_global_position)};
	uORB::Subscription _parameter_update_sub{ORB_ID(parameter_update)};
	uORB::Subscription _tracker_target_sub{ORB_ID(tracker_target_position)};
	uORB::Subscription _manual_control_sub{ORB_ID(manual_control_setpoint)};

	// Publications
	uORB::Publication<actuator_servos_s> _actuator_servos_pub{ORB_ID(actuator_servos)};
	uORB::Publication<tracker_status_s> _tracker_status_pub{ORB_ID(tracker_status)};

	// Physical angle planner, calibrated positional-servo mapping, and bounded trim correction.
	TrackerSetpointPlanner _setpoint_planner;
	TrackerServoMapper _yaw_servo_mapper;
	TrackerServoMapper _pitch_servo_mapper;
	TrackerAxisController _yaw_axis_controller;
	TrackerAxisController _pitch_axis_controller;
	TrackerTargetManager _target_manager;
	TrackerEvents _events;

	// Target selection and prediction cache.
	int32_t _configured_target_sysid{-1};
	int32_t _configured_auto_lock{-1};

	// Current geometry, setpoints, and output diagnostics.
	float _bearing_rad{0.f};
	float _pitch_rad{0.f};
	float _distance_m{0.f};
	float _yaw_error_rad{0.f};
	float _pitch_error_rad{0.f};
	float _yaw_output{0.f};
	float _pitch_output{0.f};
	float _yaw_setpoint_rad{0.f};
	float _pitch_setpoint_rad{0.f};
	float _yaw_measured_rad{0.f};
	float _pitch_measured_rad{0.f};
	float _yaw_command_deg{0.f};
	float _pitch_command_deg{0.f};
	bool _yaw_clipped{false};
	bool _pitch_clipped{false};
	uint8_t _state_reason{tracker_status_s::REASON_NONE};

	// SCAN state expressed in physical axis degrees.
	float _scan_yaw_deg{0.f};
	float _scan_pitch_deg{0.f};
	bool _scan_reverse_yaw{false};
	bool _scan_reverse_pitch{false};

	int8_t _active_mode{-1};
	bool _auto_scan_active{false};

	// Measured earth-frame head attitude when physical servos are at park.
	bool _head_reference_valid{false};
	float _yaw_home_rad{0.f};
	float _pitch_home_rad{0.f};
	uint8_t _position_source{tracker_status_s::POSITION_SOURCE_NONE};
	uint64_t _park_output_reached_us{0};
	bool _configuration_repark_pending{false};
	ControlConfiguration _configured_control{};
	bool _configured_control_valid{false};

	// Volatile and explicitly commanded: it is cleared on every reboot.
	static constexpr uint8_t SERVO_TEST_NONE{0};
	static constexpr uint8_t SERVO_TEST_YAW{1};
	static constexpr uint8_t SERVO_TEST_PITCH{2};
	static constexpr uint8_t SERVO_TEST_REQUEST_NONE{0};
	static constexpr uint8_t SERVO_TEST_REQUEST_STOP{1};
	static constexpr uint8_t SERVO_TEST_REQUEST_START_YAW{2};
	static constexpr uint8_t SERVO_TEST_REQUEST_START_PITCH{3};
	px4::atomic<uint8_t> _servo_test_axis{SERVO_TEST_NONE};
	px4::atomic<uint8_t> _servo_test_request{SERVO_TEST_REQUEST_NONE};
	int32_t _configured_fake_source{-1};
	int32_t _configured_fake_lat{INT32_MIN};
	int32_t _configured_fake_lon{INT32_MIN};
	int32_t _configured_fake_alt{INT32_MIN};

#if defined(__PX4_POSIX)
	// Deliberately local, non-persistent fault injection for the SITL evidence
	// path. It never changes the vehicle_global_position uORB publication.
	px4::atomic<bool> _sitl_force_global_position_invalid{false};
#endif

	float _servo_test_phase{0.f};
	uint64_t _boot_time_us{0};
	bool _startup_delay_done{false};
	bool _axis_outputs_initialized{false};
	uint64_t _last_run_us{0};
	float _prev_bearing_rad{0.f};
	float _prev_pitch_target_rad{0.f};
	bool _prev_bearing_valid{false};

	perf_counter_t _loop_perf{perf_alloc(PC_ELAPSED, MODULE_NAME": cycle")};
	perf_counter_t _loop_interval_perf{perf_alloc(PC_INTERVAL, MODULE_NAME": interval")};

	DEFINE_PARAMETERS(
		// Mode and startup
		(ParamInt<px4::params::TRK_MODE>)       _param_trk_mode,
		(ParamFloat<px4::params::TRK_STRT_DLY>) _param_trk_strt_dly,
		(ParamInt<px4::params::TRK_SERVO_TEST>) _param_trk_servo_test,
		(ParamFloat<px4::params::TRK_REF_SETTLE>) _param_trk_ref_settle,

		// Bounded trim correction gains
		(ParamFloat<px4::params::TRK_YAW_P>)    _param_trk_yaw_p,
		(ParamFloat<px4::params::TRK_YAW_I>)    _param_trk_yaw_i,
		(ParamFloat<px4::params::TRK_YAW_D>)    _param_trk_yaw_d,
		(ParamFloat<px4::params::TRK_YAW_FF>)   _param_trk_yaw_ff,
		(ParamFloat<px4::params::TRK_YAW_IMAX>) _param_trk_yaw_imax,
		(ParamFloat<px4::params::TRK_PIT_P>)    _param_trk_pit_p,
		(ParamFloat<px4::params::TRK_PIT_I>)    _param_trk_pit_i,
		(ParamFloat<px4::params::TRK_PIT_D>)    _param_trk_pit_d,
		(ParamFloat<px4::params::TRK_PIT_FF>)   _param_trk_pit_ff,
		(ParamFloat<px4::params::TRK_PIT_IMAX>) _param_trk_pit_imax,

		// Normalized output calibration
		(ParamFloat<px4::params::TRK_YAW_TRIM>) _param_trk_yaw_trim,
		(ParamFloat<px4::params::TRK_PIT_TRIM>) _param_trk_pit_trim,
		(ParamFloat<px4::params::TRK_YAW_MIN>)  _param_trk_yaw_min,
		(ParamFloat<px4::params::TRK_YAW_MAX>)  _param_trk_yaw_max,
		(ParamFloat<px4::params::TRK_PIT_MIN>)  _param_trk_pit_min,
		(ParamFloat<px4::params::TRK_PIT_MAX>)  _param_trk_pit_max,

		// Physical mechanical limits, park angles, and direction
		(ParamFloat<px4::params::TRK_YAW_RANGE>) _param_trk_yaw_range,
		(ParamFloat<px4::params::TRK_YAW_MIND>)  _param_trk_yaw_mind,
		(ParamFloat<px4::params::TRK_YAW_MAXD>)  _param_trk_yaw_maxd,
		(ParamFloat<px4::params::TRK_YAW_PARK>)  _param_trk_yaw_park,
		(ParamInt<px4::params::TRK_YAW_REV>)     _param_trk_yaw_rev,
		(ParamFloat<px4::params::TRK_PIT_MIND>)  _param_trk_pit_mind,
		(ParamFloat<px4::params::TRK_PIT_MAXD>)  _param_trk_pit_maxd,
		(ParamFloat<px4::params::TRK_PIT_PARK>)  _param_trk_pit_park,
		(ParamInt<px4::params::TRK_PIT_REV>)     _param_trk_pit_rev,

		// Output slew
		(ParamFloat<px4::params::TRK_YAW_SLEW>) _param_trk_yaw_slew,
		(ParamFloat<px4::params::TRK_PIT_SLEW>) _param_trk_pit_slew,

		// Geometry and loss policy
		(ParamFloat<px4::params::TRK_DIST_MIN>)  _param_trk_dist_min,
		(ParamInt<px4::params::TRK_ALT_SRC>)     _param_trk_alt_src,
		(ParamFloat<px4::params::TRK_SCAN_YSPD>) _param_trk_scan_yspd,
		(ParamFloat<px4::params::TRK_SCAN_PSPD>) _param_trk_scan_pspd,
		(ParamInt<px4::params::TRK_AUTO_SCAN>)   _param_trk_auto_scan,
		(ParamInt<px4::params::TRK_DEADRECK>)    _param_trk_deadreck,

		// Bench target and tracker home fallback
		(ParamInt<px4::params::TRK_TGT_LAT>)  _param_trk_tgt_lat,
		(ParamInt<px4::params::TRK_TGT_LON>)  _param_trk_tgt_lon,
		(ParamInt<px4::params::TRK_TGT_ALT>)  _param_trk_tgt_alt,
		(ParamInt<px4::params::TRK_FAKE_EN>)  _param_trk_fake_en,
		(ParamInt<px4::params::TRK_HOME_EN>)  _param_trk_home_en,
		(ParamInt<px4::params::TRK_HOME_LAT>) _param_trk_home_lat,
		(ParamInt<px4::params::TRK_HOME_LON>) _param_trk_home_lon,
		(ParamInt<px4::params::TRK_HOME_ALT>) _param_trk_home_alt,

		// MAVLink target selection
		(ParamInt<px4::params::TRK_SYSID_TGT>)  _param_trk_sysid_tgt,
		(ParamInt<px4::params::TRK_AUTO_LOCK>)  _param_trk_auto_lock,
		(ParamInt<px4::params::TRK_TIMEOUT_MS>) _param_trk_timeout_ms
	)
};
