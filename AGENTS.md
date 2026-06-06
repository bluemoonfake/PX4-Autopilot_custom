# AGENTS.md — Work Instructions for PX4 Antenna Tracker Agent

## 1. Project mission

Build a **PX4-native Antenna Tracker firmware** on top of `PX4-Autopilot v1.17.0`.

The firmware must allow a dedicated tracker flight controller to:

1. Receive UAV target position over MAVLink.
2. Estimate its own attitude/yaw using PX4 sensors/AHRS.
3. Compute bearing, pitch/elevation, and distance to the UAV.
4. Run yaw/pitch PID controllers.
5. Publish servo commands through PX4 `actuator_servos`.
6. Drive yaw and pitch servos through PWM outputs.

This project must not use Gimbal v2 as the main architecture. The target is an actual PX4 airframe/module equivalent in concept to ArduPilot AntennaTracker.

---

## 2. Hard rules for the coding agent

1. Base firmware must be `PX4-Autopilot v1.17.0`.
2. Use C++ for module implementation.
3. Do not rewrite unrelated PX4 core code.
4. Do not implement all features in one patch.
5. Build after each major step.
6. Keep the first version minimal:
   - AUTO-like tracking
   - STOP/failsafe behavior
   - servo test output
7. Do not add Gimbal v2/mount control as the main solution.
8. Do not directly output raw PWM microseconds from the tracking algorithm.
9. Publish normalized output to `actuator_servos`; let PX4 output drivers convert to PWM.
10. Avoid starting multicopter/fixed-wing controllers in the antenna tracker airframe.
11. Add parameters with clear `TRK_` prefix.
12. Keep geometry, controller, and PX4 module glue separated.

---

## 3. Expected repository state

The repo should be prepared as:

PX4_Autopilot v1.17.0

---

## 4. Proposed file structure

Create or modify the following files:

```text
PX4-Autopilot/
├── msg/
│   ├── TrackerTargetPosition.msg
│   └── TrackerStatus.msg
│
├── src/modules/antenna_tracker/
│   ├── CMakeLists.txt
│   ├── Kconfig
│   ├── antenna_tracker_main.cpp
│   ├── antenna_tracker.hpp
│   ├── tracker_geo.cpp
│   ├── tracker_geo.hpp
│   ├── tracker_controller.cpp
│   ├── tracker_controller.hpp
│   └── tracker_params.c
│
├── src/modules/mavlink/
│   ├── mavlink_receiver.cpp
│   └── mavlink_receiver.h
│
└── ROMFS/px4fmu_common/init.d/airframes/
    └── 4099_antenna_tracker
```

---

## 5. Phase-by-phase work

## Phase 1 — Module skeleton

### Goal

Create a PX4 module that builds and runs.

### Files

- [ ] `src/modules/antenna_tracker/CMakeLists.txt`
- [ ] `src/modules/antenna_tracker/Kconfig`
- [ ] `src/modules/antenna_tracker/antenna_tracker_main.cpp`
- [ ] `src/modules/antenna_tracker/antenna_tracker.hpp`

### Required behavior

PX4 shell commands must work:
- [ ] `antenna_tracker start`
- [ ] `antenna_tracker status`
- [ ] `antenna_tracker stop`

### Implementation notes

- Use PX4 module style similar to existing modules.
- Use `ModuleBase`/`ScheduledWorkItem` style if appropriate.
- Run loop at 50 Hz.
- At this phase, do not subscribe to MAVLink target yet.
- Print status showing:
  - running/not running,
  - loop rate,
  - current mode placeholder,
  - servo test disabled/enabled.

### Acceptance check

```bash
make px4_sitl gz_x500
pxh> antenna_tracker start
pxh> antenna_tracker status
pxh> antenna_tracker stop
```

---

## Phase 2 — Servo output path

### Goal

Verify that the module can drive servo outputs through PX4.

### Files

- [ ] `src/modules/antenna_tracker/antenna_tracker_main.cpp`
- [ ] `src/modules/antenna_tracker/antenna_tracker.hpp`

### Required behavior

- [ ] Publish normalized commands to `actuator_servos.control[0]` (yaw) and `actuator_servos.control[1]` (pitch) in the range `-1.0` to `+1.0`

### Servo test mode

Implement a safe test mode that sweeps:

```text
-0.5 → 0.0 → +0.5 → 0.0
```

Do not sweep full range by default on hardware.

### Acceptance check

- `listener actuator_servos` shows changing values.
- On hardware, MAIN/AUX outputs generate PWM.
- Yaw servo and pitch servo can move independently.

---

## Phase 3 — Parameters

### Goal

Add tracker-specific parameters.

### File

- [ ] `src/modules/antenna_tracker/tracker_params.c`

### Required parameters

- [ ] `TRK_SYSID_TARGET`
- [ ] `TRK_AUTO_LOCK`
- [ ] `TRK_TIMEOUT_MS`
- [ ] `TRK_YAW_P`
- [ ] `TRK_YAW_I`
- [ ] `TRK_YAW_D`
- [ ] `TRK_PIT_P`
- [ ] `TRK_PIT_I`
- [ ] `TRK_PIT_D`
- [ ] `TRK_YAW_TRIM`
- [ ] `TRK_PIT_TRIM`
- [ ] `TRK_YAW_MIN`
- [ ] `TRK_YAW_MAX`
- [ ] `TRK_PIT_MIN`
- [ ] `TRK_PIT_MAX`
- [ ] `TRK_TGT_LAT`
- [ ] `TRK_TGT_LON`
- [ ] `TRK_TGT_ALT`
- [ ] `TRK_MODE`
- [ ] `TRK_SERVO_TEST`

### Notes

- Use conservative defaults.
- Keep output limited.
- Parameters should allow a fake target before MAVLink target is implemented.

---

## Phase 4 — Geometry layer

### Goal

Implement standalone geometry functions.

### Files

- [ ] `src/modules/antenna_tracker/tracker_geo.hpp`
- [ ] `src/modules/antenna_tracker/tracker_geo.cpp`

### Required functions

- [ ] `longitude_scale(double latitude_deg)`
- [ ] `horizontal_distance_m(int32_t tracker_lat, int32_t tracker_lon, int32_t target_lat, int32_t target_lon)`
- [ ] `bearing_rad(int32_t tracker_lat, int32_t tracker_lon, int32_t target_lat, int32_t target_lon)`
- [ ] `pitch_rad(float delta_alt_m, float horizontal_distance_m)`
- [ ] `wrap_pi(float angle_rad)`
- [ ] `wrap_180_deg(float angle_deg)`

### Required formulas

```math
distance = sqrt(dlat^2 + dlng_scaled^2) * LOCATION_SCALING_FACTOR
```

```math
bearing = pi/2 + atan2(-off_y, off_x)
```

```math
pitch = atan2(delta_altitude, distance)
```

### Acceptance check

Create simple internal tests or debug output:

- target north → bearing near 0°
- target east → bearing near 90°
- target south → bearing near 180°
- target west → bearing near 270°
- target above same horizontal point → pitch near +90° if distance is very small

---

## Phase 5 — Controller layer

### Goal

Implement yaw/pitch PID controller.

### Files

- [ ] `src/modules/antenna_tracker/tracker_controller.hpp`
- [ ] `src/modules/antenna_tracker/tracker_controller.cpp`

### Required behavior

Inputs:

```text
yaw_error_rad
pitch_error_rad
dt
```

Outputs:

```text
yaw_output normalized [-1, 1]
pitch_output normalized [-1, 1]
```

### Required safety

- Clamp output to `[-1, 1]`.
- Clamp integrator.
- Reset integrator when:
  - target invalid,
  - module stopped,
  - mode STOP,
  - timeout triggered.

### First tuning suggestion

Start with P-only control:

```text
I = 0
D = 0
```

Add I/D only after basic servo direction is verified.

---

## Phase 6 — Fake target tracking

### Goal

Before MAVLink receiver is modified, track a target specified by parameters.

### Required subscriptions

```text
vehicle_attitude
vehicle_global_position
parameter_update
```

### Required logic

1. Read tracker GPS/global position.
2. Read tracker attitude/yaw.
3. Read fake target from `TRK_TGT_LAT`, `TRK_TGT_LON`, `TRK_TGT_ALT`.
4. Compute bearing/pitch.
5. Compute yaw/pitch error.
6. Run PID.
7. Publish `actuator_servos`.

### Acceptance check

- Change fake target params.
- Confirm bearing/pitch changes.
- Confirm servo output changes in correct direction.

---

## Phase 7 — Airframe

### Goal

Add a PX4 airframe for the antenna tracker.

### File

- [ ] `ROMFS/px4fmu_common/init.d/airframes/4099_antenna_tracker`

### Draft content

```sh
#!/bin/sh
#
# @name Generic Antenna Tracker
# @type Antenna Tracker
# @class Tracker
#

. ${R}etc/init.d/rc.sensors

# MAV_TYPE_ANTENNA_TRACKER is 5 in MAVLink.
# Verify PX4 accepts this value before finalizing.
param set-default MAV_TYPE 5

# Conservative PWM defaults. Verify exact parameter names in PX4 v1.17.
param set-default PWM_MAIN_MIN 1000
param set-default PWM_MAIN_MAX 2000
param set-default PWM_MAIN_CENTER 1500

# Start MAVLink. Device may differ by board.
mavlink start -d /dev/ttyS1 -b 57600

antenna_tracker start
```

### Acceptance check

- Airframe appears in QGC.
- Selecting airframe applies defaults.
- Module starts after boot.
- No unnecessary multicopter/fixed-wing controller controls the servo outputs.

---

## Phase 8 — uORB messages

### Goal

Add target and status messages.

### File: `msg/TrackerTargetPosition.msg`

- [ ] `msg/TrackerTargetPosition.msg`

Suggested fields:

```text
uint64 timestamp
uint8 target_system
bool valid

int32 lat
int32 lon
int32 alt_mm
int32 relative_alt_mm

float32 vx_m_s
float32 vy_m_s
float32 vz_m_s

uint64 last_update_us
```

### File: `msg/TrackerStatus.msg`

- [ ] `msg/TrackerStatus.msg`

Suggested fields:

```text
uint64 timestamp

bool target_valid
uint8 target_system

float32 bearing_rad
float32 pitch_rad
float32 distance_m

float32 yaw_error_rad
float32 pitch_error_rad

float32 yaw_output
float32 pitch_output

uint8 mode
uint8 state
```

### Acceptance check

```bash
listener tracker_target_position
listener tracker_status
```

works in PX4 shell after build.

---

## Phase 9 — MAVLink receiver integration

### Goal

Receive target UAV position from MAVLink and publish to uORB.

### Files

- [ ] `src/modules/mavlink/mavlink_receiver.cpp`
- [ ] `src/modules/mavlink/mavlink_receiver.h`

### Required MAVLink message

```text
GLOBAL_POSITION_INT
```

### Required filtering

- Ignore messages from:
  - GCS
  - this tracker itself
  - unrelated sysid
- If `TRK_SYSID_TARGET != 0`, only accept that sysid.
- If `TRK_SYSID_TARGET == 0` and `TRK_AUTO_LOCK == 1`, lock first valid vehicle sysid.

### Required conversion

```text
lat/lon: keep int32 degE7
alt: mm
relative_alt: mm
vx/vy/vz: cm/s → m/s
timestamp: hrt_absolute_time()
```

### Acceptance check

Use a script to send MAVLink `GLOBAL_POSITION_INT`.

Expected:

```bash
listener tracker_target_position
```

shows updated target data.

---

## Phase 10 — Timeout and failsafe

### Goal

Prevent stale target from driving the servos.

### Required behavior

If target age exceeds `TRK_TIMEOUT_MS`:

1. Mark target invalid.
2. Stop PID update.
3. Reset integrator.
4. Publish safe servo output:
   - hold current output, or
   - center output, depending on selected policy.

### Recommended first policy

Use center/neutral output for first hardware tests.

---

## Phase 11 — Simulation test plan

### Test A — Compile test

```bash
make px4_sitl gz_x500
```

Expected:

- Build succeeds.
- PX4 shell has `antenna_tracker` command.

### Test B — uORB output test

```bash
pxh> antenna_tracker start
pxh> listener actuator_servos
```

Expected:

- Servo outputs update.

### Test C — Fake target test

Set fake target params and monitor:

```bash
pxh> listener tracker_status
```

Expected:

- distance, bearing, pitch, errors update.

### Test D — MAVLink fake target

Use a companion script to send `GLOBAL_POSITION_INT`.

Expected:

```bash
pxh> listener tracker_target_position
```

updates at the chosen MAVLink rate.

### Test E — Servo hardware dry run

- FC tracker on desk
- Servo yaw/pitch connected
- No propellers, no UAV flight
- Send fake target path by MAVLink
- Confirm servo motion is smooth and correct direction

---

## 6. Definition of done for first firmware milestone

The first milestone is done when:

1. PX4 v1.17.0 builds with the new module.
2. The module starts/stops from PX4 shell.
3. The new airframe starts the module automatically.
4. The module can publish `actuator_servos`.
5. The module can track a fake target.
6. The module can receive target from MAVLink `GLOBAL_POSITION_INT`.
7. Servo yaw/pitch outputs respond to target changes.
8. Timeout prevents stale target from moving servos.

---

## 7. Notes for future expansion

After the first milestone, add:

- SCAN mode
- MANUAL mode
- SERVOTEST mode
- auto-lock target sysid
- dead reckoning using target velocity
- altitude source selection:
  - GPS absolute altitude
  - GPS relative altitude
  - barometer difference
- continuous rotation servo support
- logging improvements
- QGC parameter metadata polishing
