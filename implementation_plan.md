# PX4 Antenna Tracker Implementation Plan

## Purpose

This is the canonical roadmap for the PX4-native antenna tracker firmware. It replaces earlier phase checklists and historical test narratives that drifted from the source tree.

The target system uses **positional yaw/pitch servos** and an IMU mounted rigidly on the moving antenna assembly. It must remain compatible with **stock QGroundControl** through normal PX4 metadata, parameters, actuator configuration, events, and logs.

Read [the tracker documentation](docs/en/antenna_tracker/index.md) and use [the verification contract](validation/antenna_tracker/README.md) before changing runtime behavior.

## Status vocabulary

| State | Meaning |
|---|---|
| planned | Work is scoped but not implemented. |
| implemented | Source is present; behavior is not yet verified. |
| blocked | The test cannot currently complete because a prerequisite, setup, or required evidence is unavailable. |
| verified-sitl | Passed canonical tracker SITL with evidence. |
| verified-bench | Passed controlled bench testing on intended hardware. |
| verified-hardware | Passed deployed sensor/telemetry/mechanical testing. |
| field-ready | Repeated field and failure tests passed; known limits are published. |

## Gate 0 — Repository, documentation, and evidence normalization

**State:** implemented; final validation pending for this change set.

### Scope

- Establish `docs/en/antenna_tracker/` as the canonical human documentation.
- Establish `validation/antenna_tracker/` as the canonical test matrix and evidence schema.
- Consolidate tracker utilities under `Tools/antenna_tracker/`.
- Retire duplicate/outdated docs, generated-log heuristics, x500 transcripts, and ArduPilot-only utilities from active paths.

### Acceptance criteria

- One canonical location defines architecture, QGC policy, build procedure, safety, and verification.
- No active instructions use `gz_x500` to validate airframe 4099.
- No active instructions use obsolete `TRK_SYSID_TARGET` or the historical mismatched target port.
- Every future test can be linked to a test ID and evidence manifest.

### Evidence

- `BLD-001`, `BLD-002`, and documentation/tool validation recorded after this structural change is checked.

---

## Gate 1 — Honest stock-QGC integration

**State:** implemented. The 2026-07-19 MicoAir H743 run verifies the two-axis
MAIN1/MAIN2 mapping and standard MAVLink ARM/DISARM path. Stock-QGC GUI Events
capture and the remaining hardware safety gates are still required.

### Objective

Make the tracker clearly usable in unmodified QGroundControl without claiming custom tracker flight modes that QGC/PX4 Commander do not actually implement.

### Work

1. Update `src/modules/mavlink/mavlink_params.c`:
   - add a human-readable parameter metadata value for `MAV_TYPE=5` / Antenna Tracker;
   - correct the advertised range so the valid Antenna Tracker value is represented.
2. Keep airframe metadata as `@type Antenna Tracker` and `@class Rover` until the QGC metadata pipeline supports a tracker class natively.
3. Deprecate `TRK_QGC_MODE` and change the tracker-airframe default to disabled:
   - `TRK_MODE` is the tracker submode authority;
   - do not map PX4 Position/Altitude/Manual navigation states to AUTO/SCAN/MANUAL in new production behavior.
4. Document stock-QGC operation through Parameters, Actuators, PX4 Events, MAVLink Console, ULog, and the tracker monitor utility.
5. Keep generic Servo1/Servo2 output functions. Do not add global “Tracker Yaw/Pitch” output-function enums before hardware validation identifies a real operator problem.

### Critical files

- `src/modules/mavlink/mavlink_params.c`
- `src/modules/antenna_tracker/antenna_tracker_main.cpp`
- `src/modules/antenna_tracker/tracker_params.c`
- both `4099_antenna_tracker` airframes
- `docs/en/antenna_tracker/qgroundcontrol.md`

### Acceptance criteria

- QGC displays/selects the tracker airframe and exposes tracker parameters.
- MAVLink type 5 has readable metadata.
- Changing PX4 navigation state cannot silently select SCAN/AUTO in the production tracker path.
- QGC Actuators identifies Servo1 as yaw and Servo2 as pitch through airframe output metadata.

### Required evidence

- `SITL-001`, `SITL-002`, `BENCH-001`.

---

## Gate 1A — Commander readiness and ARM-gated tracker execution

**State:** implemented. Targeted unit checks, local tracker SITL observations,
and a MicoAir H743 standard MAVLink ARM/DISARM run exist, but `SITL-010` and
`SITL-011` remain `implemented`, not `verified-sitl`. Complete manifests, the
untested transition branches, and stock-QGC Events capture remain required.

### Objective

Allow stock QGC to arm a healthy `MAV_TYPE=5` tracker without depending on
irrelevant Position/Altitude/Manual flight-mode requirements, while ensuring a
requested `TRK_MODE` cannot run until PX4 is armed.

### Work

1. Capture the current arming rejection through Commander Events,
   `actuator_armed`, `vehicle_status`, and `health_report` before changing
   readiness logic.
2. Treat `TRK_MODE` as requested state and derive an effective tracker mode:
   - disarmed, kill, lockdown, or termination: effective STOP/PARK;
   - armed: requested STOP/AUTO/SCAN/MANUAL, subject to existing runtime safety
     checks.
3. Subscribe to `actuator_armed`, reset controller history on arm/disarm edges,
   and cancel servo test if the system arms.
4. Add a tracker-only Commander readiness policy for `MAV_TYPE=5` based on the
   recorded rejection:
   - retain calibrated IMU/attitude, power, safety, output, hard-fault, kill,
     lockdown, and termination failures;
   - do not require unrelated navigation modes, RC, mission, or local
     position/altitude merely to arm the tracker;
   - keep missing target/global-position data as explicit runtime park states.
5. Append armed/requested/effective fields and a waiting-for-arm reason to
   `TrackerStatus`; expose transitions through status, Events, and ULog.
6. Keep stock QGC and normal Commander ARM/DISARM. Do not add an external
   tracker flight mode, force-arm path, Gimbal v2 dependency, or global arming
   bypass.

### Critical files

- `src/modules/antenna_tracker/antenna_tracker_main.cpp`
- `src/modules/antenna_tracker/antenna_tracker.hpp`
- `msg/TrackerStatus.msg`
- the smallest applicable Commander health/arming requirement files identified
  by baseline evidence
- `docs/en/antenna_tracker/arming_and_modes.md`
- `validation/antenna_tracker/test_matrix.yaml`

### Acceptance criteria

- Booting with `TRK_MODE=AUTO`, SCAN, or MANUAL cannot move beyond park while
  disarmed.
- A standard successful ARM makes the requested tracker mode effective without
  rewriting `TRK_MODE`.
- AUTO still waits at park for reference settle and valid runtime inputs.
- DISARM, kill, lockdown, or termination parks and resets control state within
  one 50 Hz cycle.
- QGC Ready/Not Ready reflects tracker-relevant failures and no longer depends
  on unsupported tracker mappings to Position/Altitude/Manual navigation modes.
- Existing STOP, timeout, sensor-invalid, reference-settle, and volatile servo
  test safety behavior does not regress.

### Required evidence

- `BLD-001`, `SITL-001`, `SITL-002`, `SITL-006`, `SITL-009`, `SITL-010`,
  `SITL-011`, and `HW-004` before hardware completion is claimed.

### Current evidence snapshot

| Item | Current result | Remaining work |
|---|---|---|
| Build | `px4_sitl_default` built locally | Record `BLD-001` manifest and build log for the final revision |
| Commander requirements | Tracker-only attitude/rate requirement unit test passed locally | Retain the test log in a run manifest |
| ARM/DISARM path | H743 standard command ARM and DISARM each returned ACK result 0 and matching HEARTBEAT armed state | Capture the same flow in stock QGC with PX4 Events |
| Runtime ARM gate | H743 disarmed AUTO reported requested 1/effective 0 at 1500/1500 us; ARM made effective 1; missing position parked with reason 4; DISARM restored effective STOP | Exercise SCAN, MANUAL, kill, and lockdown; record a conforming `SITL-010` manifest |
| STOP diagnostics | Fixed: disarmed requested STOP reports IDLE/STOP; only a non-STOP request reports waiting-for-ARM | Add this hardware observation to the final SITL/QGC regression evidence |
| QGC readiness | H743 reported `ready_to_arm=true`, all health/arming warning/error flags zero, and accepted standard GCS ARM/DISARM | Retain stock-QGC Events capture and verify all retained hardware blockers |
| Hardware safety | `HW-004` passed: operator cut the independent servo rail while FC stayed live; termination and reboot produced 49/49 and 94/94 neutral samples | Keep the cutoff available for every later powered test |

---

## Gate 2 — Positional-servo control architecture

**State:** implemented; current unit tests cover positional mapping and the
continuous-rate neutral/reverse/safe-stop path. The airframe-4099 SITL smoke
test also verifies independent per-axis reporting, continuous neutral in STOP,
and rejection of the positional sweep on a continuous axis. MicoAir H743 now
selects its 50 Hz Timer 0 default in the board profile only for airframe 4099;
generic SITL no longer depends on a board timer parameter. The 50 Hz H743
retest passed physical yaw/pitch return. The follow-up selected-magnetometer
test found a 4.58x field increase under yaw movement, so compass installation
interference—not missing IMU motion—is the remaining control blocker.

### Problem to solve

A normal RC servo receives a **position** command. The current direct `PID(angle_error) -> normalized servo output` approach can return the command toward neutral once measured error is near zero, even when the desired antenna angle is not neutral.

### Target architecture

```text
target source
  -> geometry: desired world yaw / pitch
  -> mechanical planner: reachable sector, pitch limits, park/scan/manual setpoints
  -> calibrated servo mapper: physical angle -> normalized Servo1/Servo2 output
  -> bounded stateful correction and slew limit
  -> actuator_servos
```

### Work

1. Split responsibilities without duplicating PX4 utilities:
   - `tracker_target_manager.*`: target validity, timeout, source data, optional prediction;
   - `tracker_setpoint_planner.*`: geometry, AUTO/SCAN/MANUAL/PARK desired angles, yaw sector;
   - `tracker_servo_mapper.*`: physical angle to normalized command, reverse, trim, output calibration;
   - `tracker_axis_controller.*`: stateful servo command and bounded correction;
   - `tracker_events.*`: transition events.
2. Move AUTO, SCAN, MANUAL, STOP, startup, and timeout through a single angle-domain output pipeline.
3. Keep servo motion semantics separate from PX4 electrical output setup:
   - `TRK_YAW_SRV_T` / `TRK_PIT_SRV_T`: POSITION or CONTINUOUS;
   - `PWM_MAIN_TIMx` / `PWM_AUX_TIMx`: PWM protocol/rate for a shared timer group;
   - continuous safe states command neutral immediately and positional
     `servo_test` is rejected.
4. Replace the hard-coded normalized-zero safe command with configurable per-axis park angles.
5. Replace symmetric-only yaw range handling with explicit yaw min/max/park angles and a reachable-sector policy.
6. Define pitch min/max/park angles in the same physical domain.
7. Use PX4 geo, matrix-wrap, and slew-rate helpers where applicable.
8. Keep fake target and home fallback as bench/development aids; preserve deterministic behavior.

### Critical files

- `src/modules/antenna_tracker/antenna_tracker_main.cpp`
- `src/modules/antenna_tracker/antenna_tracker.hpp`
- new tracker planner/mapper/controller/event files
- `src/modules/antenna_tracker/tracker_params.c`
- `src/modules/antenna_tracker/TrackerMathTest.cpp`
- `msg/TrackerStatus.msg`

### Acceptance criteria

- A non-neutral desired antenna angle remains commanded when tracking error becomes zero.
- STOP, timeout, and invalid sensors select documented physical park positions.
- Target headings outside the mechanical sector do not command through a stop or cable-wrap limit.
- SCAN only visits validated reachable angles.
- Manual mode emits angle/rate setpoints through the same safety mapping, not raw servo passthrough.
- Analog/digital positional servos use the same POSITION semantics; changing
  PWM rate does not change tracker control type.
- Continuous axes use moving-IMU feedback and output neutral immediately on
  STOP, timeout, disarm, or invalid attitude.

### Required evidence

- `UNIT-002`, `SITL-002`, `SITL-003`, `SITL-006`, `BENCH-002`.

### Calibration contract (required before further closed-loop hardware claims)

The angle-domain mapper is only as accurate as the installed mechanics. A
passing software mapper does not prove that a commanded angle equals the angle
measured by the moving IMU. Before `HW-002` can pass, record a board/mechanism
calibration profile containing the output bank/pin, PWM min/max/disarmed/
failsafe values, physical yaw/pitch min/max/park, reversal, trim, and the
firmware artifact SHA-256.

Initial hardware acceptance limits are:

- each `+/-2 deg` yaw step moves in the requested direction and settles within
  `0.5 deg` for at least three seconds;
- each `+/-5 deg` yaw step settles within `1.0 deg` in at most 30 seconds,
  without more than `1.0 deg` overshoot beyond the requested angle;
- a return-to-park step settles within `0.5 deg`;
- apply the same limits independently to pitch after production pitch mechanics
  are installed.

An out-of-tolerance result is a calibration/mechanics blocker, not PID tuning
permission. Correct horn position, travel, PWM endpoints, trim, and reversal
before changing controller gains.

---

## Gate 3 — MAVLink target bridge hardening

**State:** implemented. Historical `HW-001` passed on the selected V6X USB
bench route (`UDP 18570 -> usb_router.py -> USB CDC MAVLink instance 2`),
including fixed-sysid and GCS-rejection evidence. Re-run it from the clean
release commit before calling the current source verified-hardware.

### Work

1. Keep only a small dispatch integration in `mavlink_receiver.cpp` and move tracker-specific target handling into a dedicated MAVLink helper/bridge.
2. Enable the bridge only for tracker operation so unrelated vehicle behavior is unaffected.
3. Validate target source in this order:
   - reject self and known GCS sources;
   - system-ID selection or validated auto-lock;
   - component/heartbeat policy;
   - valid latitude/longitude, timestamps, altitude, and velocity fields;
   - target update age.
4. Record source component, MAVLink instance, validity flags, and age in `TrackerTargetPosition` without reordering existing fields.
5. Keep dead reckoning disabled by default until velocity validity and prediction error are hardware-verified.
6. Use MSL altitude in production. Do not use target `relative_alt` as tracker height difference unless both vehicles explicitly share a reference.
7. Standardize the SITL sender and bench router port configuration; record it in each manifest.

### Critical files

- `src/modules/mavlink/mavlink_receiver.cpp`
- `src/modules/mavlink/mavlink_receiver.h`
- new MAVLink tracker bridge helper
- `msg/TrackerTargetPosition.msg`
- `Tools/antenna_tracker/send_target.py`
- `Tools/antenna_tracker/usb_router.py`

### Acceptance criteria

- A valid configured source publishes a fresh target topic.
- Wrong system IDs/components are rejected according to documented policy.
- Auto-lock does not change while an accepted source remains fresh, then releases on configured timeout.
- The test sender reaches the intended PX4 receiver port.
- Invalid/unknown velocity cannot produce unbounded prediction.

### Required evidence

- `SITL-004`, `SITL-005`, `SITL-006`, `SITL-007`, `SITL-008`, `HW-001`.

---

## Gate 4 — Board, airframe, and startup safety

**State:** implemented. Historical output mapping and yaw timeout exist, but
the current source requires a new two-axis bench record; `HW-004` startup and
cutoff evidence also remains required.

### Work

1. Audit the dedicated FMUv6C target and disable `navigator` and `control_allocator` only after confirming no required dependency; the tracker must retain exclusive Servo1/Servo2 ownership.
2. Define and document the production MAVLink serial port, baud rate, MAVLink instance, and routing policy.
3. Set conservative airframe defaults for:
   - tracker STOP mode;
   - QGC nav-state mapping disabled;
   - dead reckoning disabled;
   - scan-on-loss disabled;
   - MSL altitude source;
   - servo test disabled.
4. Configure PWM disarmed/min/max/failsafe values according to the actual mechanism.
5. Ensure prearm does not cause a motion jump: startup must command park before servo power is enabled or released.

### Critical files

- `boards/px4/fmu-v6c/antenna_tracker.px4board`
- hardware and POSIX `4099_antenna_tracker` airframes
- `src/modules/antenna_tracker/tracker_params.c`
- `docs/en/antenna_tracker/hardware_setup.md`
- `docs/en/antenna_tracker/safety.md`

### Acceptance criteria

- Dedicated build has no competing controller publisher for Servo1/Servo2.
- Boot, prearm, STOP, timeout, and sensor-invalid behavior are mechanically safe.
- Hardware serial/telemetry target path is reproducible from documentation.

### Required evidence

- `BLD-002`, `BENCH-001`, `BENCH-002`, `HW-004`.

### Deployment profiles

Keep firmware build support separate from a verified wiring profile:

| Profile | Intended link/output | Status |
|---|---|---|
| FMUv6C production candidate | TELEM1 at 57600, MAIN Servo1/Servo2 | build only; hardware route pending |
| FMUv6X bench | USB CDC router and AUX Servo1/Servo2 | ingress verified; mechanics/calibration in progress |
| MicoAir H743 | direct USB, MAIN Servo1/Servo2 | build/flash, BENCH-001, USB HW-001, and HW-004 verified; BENCH-003 blocked by 4.58x selected-compass disturbance under servo load |

Never copy a bench AUX parameter export into a production MAIN airframe
default. A deployment profile must include board, port, baud, MAVLink instance,
output bank, calibrated PWM values, mechanism limits, and evidence link.

---

## Gate 5 — Status, Events, and logging

**State:** implemented. A clean V6X firmware build at commit `41927b6fe7`
logged `tracker_status`, `tracker_target_position`, `actuator_servos`, and
Events with no dropouts. Because `TrackerStatus` has changed, the current
release still needs a fresh ULog; `HW-003` remains required for dynamic
tracking and loss/recovery acceptance.

### Work

1. Extend `TrackerStatus` by appending measured angles, desired angles, mapped commands, clipping flags, target age, and state reason.
2. Extend `TrackerTargetPosition` by appending source and validity metadata.
3. Add edge-triggered PX4 Events for target acquired/lost, source rejected, sector clipped, entering scan, and parking output.
4. Keep tracker status and target topics logged at rates sufficient for diagnosis without overloading logs.
5. Keep raw ULogs out of Git; link them from evidence manifests with checksums.

### Acceptance criteria

- A ULog explains why output moved, parked, clipped, or stopped.
- QGC can surface meaningful PX4 Events without a tracker plugin.
- Source age and source identity are available for debugging target loss.

### Required evidence

- `SITL-004`, `SITL-006`, `SITL-008`, `HW-003`.

---

## Gate 6 — Automated unit and canonical SITL regression

**State:** historical regression passed at prior commits. The current safety
revision requires fresh `UNIT-001` through `UNIT-003` and `SITL-001` through
`SITL-011` manifests before this gate is again verified-sitl.

### Work

1. Expand tracker unit coverage for geometry, mapping, angle wrap, park, sector clipping, state transitions, source filtering, prediction guards, and safe-state behavior.
2. Run tracker SITL through airframe 4099:

```bash
make px4_sitl_default
cd build/px4_sitl_default
PX4_SYS_AUTOSTART=4099 PX4_SIM_MODEL=none PX4_PARAM_SIH_VEHICLE_TYPE=5 ./bin/px4
```

3. Capture manifests and raw logs for each SITL case.
4. Treat a dynamic circular test as a directional/continuity test unless its duration/speed truly cover the claimed arc.

### Acceptance criteria

All `UNIT-*` and `SITL-*` cases in [test_matrix.yaml](validation/antenna_tracker/test_matrix.yaml) have current evidence and no unexplained failure.

---

## Gate 7 — Bench and hardware stabilization

**State:** bench/hardware validation in progress. The 2026-07-19 MicoAir H743
run passed `BENCH-001` and direct-USB `HW-001`. Setting Timer 0 to 50 Hz
resolved the pitch park failure (0.020-degree residual), but `BENCH-002` still
remains blocked because yaw attitude stopped 25.740 degrees from baseline after
its output returned to 1500 us. The operator confirmed the positional yaw head
physically returned to its initial mark, so this is now a heading/compass
measurement blocker rather than a mechanical-return failure. `BENCH-003` now
has direct selected-compass evidence: device 527625 increased from 0.412 G to
1.887 G under yaw movement and EKF raised its disturbed-field preflight flag.
A later limited 1486..1514 us retest passed with a -2.52% field change and no
EKF fault, but covered only 1.736 degrees and therefore does not clear the
broader-range blocker. A staged follow-up localized the failure: 8.039 degrees
at 1452..1548 us passed with -1.08% field change, while 17.009 degrees at
1400..1600 us crossed the 10% stop limit at -16.61% and reproduced a
25.03-degree post-neutral yaw residual. No larger stage was run after the
safety threshold was crossed.
After operator-reported compass recalibration, the bounded Stage 1 was repeated
at 1452..1548 us. Selected field fell from 0.30380 G to 0.18921 G (-37.72%)
and remained 0.19241 G (-36.66%) after STOP, with a 55.70-degree reported-yaw
residual at neutral output. Stage 2 was deliberately skipped. This confirms
that recalibration alone does not clear the physical magnetic-installation
blocker.
An internal-compass comparison then selected IST8310 device 396817 and disabled
external device 527625. The same Stage 1 reduced selected field from 0.31948 G
to 0.26669 G (-16.52%); it remained -16.66% after neutral with a 32.97-degree
reported-yaw residual. Internal selection improves on the external -37.72%
result but still fails the 10% limit, so no Stage 2 was run. The internal
priority remains persisted pending the next installation diagnostic.
A repeat with the subsequently observed priorities MAG0=75/MAG1=50 still
selected internal device 396817. Eighty selected samples were clean during the
early/mid active window (+0.23%), but the selected field was -25.62% after STOP
and remained -25.16% after a 15-second neutral dwell, with a 38.71-degree yaw
residual. This confirms a later-travel/return-dependent failure and does not
clear BENCH-003.
An explicit external-compass repeat then persisted MAG0=0/MAG1=100 and verified
device 527625 after reboot. After a stable 80-sample 0.31016 G baseline, the
early active window remained clean (+0.12%), but a late batch rose +17.73% and
triggered the stop threshold plus an EKF heading-innovation failure. Field was
still +15.49% after 15 seconds at neutral, although yaw returned within 0.30
degrees. External selection improves returned heading but does not clear field
integrity.
A subsequent operator manual rotation exposed an external-compass data-path
failure: device 527625 stopped publishing for more than 40 seconds, the sensor
validator reported `best=-1`/TOUT with two failsafe events, and QMC5883L showed
112 resets plus register/transfer errors. Internal IST8310 remained live with
zero driver errors. This run is not a valid field comparison; external I2C
cable, connector, power, ground, and strain relief now take priority before any
additional sweep.
The next bounded diagnostic preserved the current MAG0=75/MAG1=0 and
800..2200 us endpoint parameters. A volatile +/-0.07 function-201 override
reached 1549 us and released to 1500 us without writing parameters, but
reported yaw moved only 0.027 degrees. The operator subsequently confirmed
normal physical servo/mechanism motion, so the remaining discrepancy is that
`vehicle_attitude` did not represent the observed movement. Selected internal
field ended -12.65% from baseline after a 15-second neutral dwell, with a
3.32-degree yaw residual; the external compass remained timed out. This is
blocked evidence, not a BENCH-003 pass, and no larger stage was run.
After USB reconnection, a full-transition capture of the same bounded envelope
passed. The 1451..1500 and 1500..1549 us transitions produced 4.541 and 4.264
degrees of measured yaw, while selected internal-field changes were -0.62%
and -1.84%. Final field residual was -2.21%, output returned to 1500/1500 us,
all recorded heartbeats were disarmed, and no parameter was written. This
clears BENCH-003 only for the bounded 1451..1549 us envelope; the previous
1400..1600 us failure range, full endpoints, and external-compass selection
remain unqualified.
The following HW-002 static-yaw attempt used a temporary home fallback, target
system 3, `TRK_YAW_REV=1`, a +/-5-degree sector, and 1451..1549 us MAIN1
limits. Direction and magnetic safety passed, but accuracy did not: +2/-2/-5/
return errors were 0.689/0.928/1.990/0.709 degrees against 0.5/0.5/1.0/0.5
degree limits. Only +5 passed at 0.535 degrees. This is a yaw calibration,
asymmetric travel, backlash, or park/endpoint blocker; pitch remains untested.
All temporary parameters were restored after verified STOP/DISARM.
Repeating the same HW-002 sequence with every yaw/pitch PID and feed-forward
term set to zero did not clear accuracy. +2/-2/+5/return errors became
1.200/0.643/3.486/1.725 degrees; -5 steady error was 0.126 degrees but its
1.441-degree overshoot failed. Field remained within -4.33..+2.49%. The
direction- and history-dependent response rules out PID as the sole cause and
strengthens the mechanical/mapping, backlash, park, or asymmetric-travel
diagnosis. PID/FF remains zero by operator request; temporary setup parameters
were restored.
Code review found that positional feed-forward mapping applied the axis reverse
flag while its PID/FF correction did not. The correction path now applies the
same direction before summation. A normal/reversed regression test brings
`unit-TrackerMath` to 15/15 passing tests, and the MicoAir H743 build passes at
81.91% flash use with artifact SHA-256
`54a588aa1cb123d552b21b1a60469c244e2022a4d6b702dc1062279ff5d0af37`.
The artifact was then flashed to the MicoAir H743 with erase/program/verify at
100% and airframe 4099 verified after reboot. A bounded yaw retest used
`TRK_YAW_P=0.05`, zero I/D/FF, `TRK_YAW_REV=1`, a +/-5-degree sector and MAIN1
1451..1549 us. Both command signs moved measured yaw in the correct direction,
showing no overall direction regression. At P=0.05 the correction is below one
PWM microsecond, so its sign is directly covered by the 15/15 unit regression
rather than independently isolated by this hardware run. Static accuracy still failed:
+2/-2/+5 errors were 0.649/2.158/5.815 degrees, and the -5 phase stopped when
measured relative yaw crossed -7.53 degrees. HW-002 therefore remains blocked
on mechanical angle/PWM calibration and history-dependent travel, not the
correction sign. Increasing gain is not authorized. The test ended STOP,
disarmed and neutral with temporary parameters restored and PID/FF zero.
The operator then corrected the mechanical yaw definition to -90..+90 degrees.
HW-002 was repeated without narrowing the native 800..2200 us PWM endpoints,
using reverse 1, P=0.05 and a temporary four-second slew. All four static
target dwells completed: +2 degrees passed at 0.371-degree error, while -2,
+5 and -5 failed at 0.943, 1.424 and 1.494 degrees. The return phase stopped on
the unchanged yaw-rate guard at 0.871 rad/s. Output and selected field remained
safe at 1459..1541 us and -2.33..+1.75%. HW-002 remains blocked, but the
corrected mapping materially improves symmetry and replaces the invalid
-180..180 deployment setting. The board retains -90..90 and reverse 1; PID/FF
is zero and slew is restored to two seconds.
The servo's supplied full calibration is 500..2500 us for 180 degrees, making
the restricted 800..2200 us deployment window equivalent to -63..+63 degrees.
HW-002 was repeated with that paired mapping, P=0.05, an eight-second temporary
slew and one-degree ramps between static dwells. The full scenario completed
without a safety abort. +2, -2 and return passed at 0.243/0.208/0.182 degrees;
+5 and -5 failed at 1.383/1.305 degrees. Maximum yaw rate was 0.580 rad/s,
output was 1443..1556 us and selected-field change was -1.14..+1.86%. The board
retains -63..63 and reverse 1, while PID/FF is zero and slew is two seconds.
HW-002 remains blocked by five-degree accuracy and missing pitch validation.
Yaw P was then raised to 0.8 with I/D/FF zero. Under the same -63..63 mapping,
an eight-second temporary slew and one-degree inter-dwell ramps, every static
yaw case passed: +2/-2/+5/-5/return errors were
0.050/0.180/0.750/0.578/0.145 degrees. Maximum yaw rate was 0.662 rad/s, PWM
remained 1435..1563 us, selected-field change remained -1.79..+1.94%, and no
safety guard fired. This qualifies ramped yaw static accuracy, not a direct
zero-to-five-degree step or pitch. HW-002 therefore remains blocked rather than
passing. The board retains P=0.8, -63..63 and reverse 1; I/D/FF are zero and
slew is restored to two seconds.
The overall direct-step follow-up used a 12-second temporary slew without
relaxing the 0.8 rad/s guard. Yaw +2/-2/+5 passed at 0.403/0.366/0.592 degrees
maximum final error, but -5 and return failed at 1.155/0.747 degrees. Isolated
positive pitch +2/+5 also failed at 1.714/1.926 degrees, while park return
passed at 0.497 degrees. Negative pitch remains unqualified because the
installed range is 0..90 degrees. Primary runs completed without a guard;
maximum yaw/pitch rates were 0.758/0.452 rad/s and field stayed within
-2.49..+3.77%. `HW-002` is now failed overall, not merely untested. The board
ended STOP/disarmed and neutral; yaw P=0.8 is retained while pitch PID/FF is
zero.
`HW-002` remains failed until yaw calibration/park and pitch calibration pass;
`HW-003` remains blocked on HW-002.
`HW-004` passed after the operator physically cut the independent servo rail;
termination and reboot returned all recorded outputs to neutral. The current
checksummed handoff is in
`validation/antenna_tracker/runs/hardware/bench003-hw004-20260719T025023Z/`.

### Work

1. Validate Pixhawk 6C hardware output mapping, servo supply, direction, limits, reversal, park position, and no-startup-jump.
2. Verify that the IMU moves with the head and that the compass is valid under servo/motor current.
3. Verify deployed telemetry target ingress and source selection.
4. Start with static low-gain tests and bounded output, then dynamic target and loss/recovery tests.
5. Maintain an independent servo-power cutoff through the entire gate.
6. Improve bench operator UX: provide `antenna_tracker servo_test start
   yaw|pitch` and `antenna_tracker servo_test stop`, reporting the active axis
   in `status`. This is a volatile bench override that requires STOP and is
   cancelled immediately if `TRK_MODE` leaves STOP; normal tracker operation
   continues to use `TRK_MODE`. `TRK_SERVO_TEST` is legacy-only and is cleared
   rather than allowed to move hardware after boot.
7. Add a scenario runner for hardware evidence. It must timestamp target phases,
   keep HEARTBEAT and position traffic fresh throughout each dwell, record phase
   start/end in a machine-readable log, then cease target traffic for the final
   timeout dwell. It must not silently change `TRK_MODE`; the operator confirms
   final STOP. ULog is the authority for tracker response; an interactive
   console snapshot is only supporting evidence.
8. Treat `TRK_HOME_EN` as a provisioned deployment setting. Production defaults
   to disabled unless a verified tracker home is supplied; the all-zero
   latitude/longitude placeholder is rejected in every build. Missing global
   position and a requested but unprovisioned home must park with a diagnostic
   reason.
9. Make servo test a volatile, single-axis command that requires STOP. A
   persistent parameter must not sweep hardware after reboot.
10. Park and settle the servo command for configurable `TRK_REF_SETTLE` before
    capturing the moving-IMU head reference on every AUTO transition. This is
    command-settle only until a future position-feedback design exists.
11. Park when MANUAL input is stale; never substitute the midpoint of a
    mechanical range for absent input.
12. Make fake target an explicit source (`TRK_FAKE_EN`), never an automatic
    fallback after MAVLink timeout.

### Acceptance criteria

`BENCH-001` through `BENCH-003` and `HW-001` through `HW-004` are verified with evidence manifests. A V6X USB-router result does not verify the FMUv6C TELEM1 production profile.

---

## Gate 8 — Field-ready release

**State:** planned.

### Work

1. Run repeated outdoor static and dynamic tracking.
2. Validate target loss/recovery, source rejection, reachability, cable-wrap behavior, and restart safety.
3. Publish a release note containing firmware provenance, supported board/mechanics, tuning baseline, test evidence, and known limits.

### Acceptance criteria

`FIELD-001` passes repeatedly. The tracker is only then described as **field-ready**.

## Deferred until after field stability

- continuous multi-turn/encoder feedback, relay, or stepper actuator backends;
- yaw turn counting and automatic cable-wrap strategies;
- advanced PID notch/filter work;
- custom output-function enums for tracker axes;
- QGC plugin/custom tracker UI;
- dead reckoning enabled by default.
