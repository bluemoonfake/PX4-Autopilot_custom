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

**State:** implemented; SITL/bench evidence remains required before verification.

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

## Gate 2 — Positional-servo control architecture

**State:** implemented; bench validation remains required before field tracking.

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
3. Replace the hard-coded normalized-zero safe command with configurable per-axis park angles.
4. Replace symmetric-only yaw range handling with explicit yaw min/max/park angles and a reachable-sector policy.
5. Define pitch min/max/park angles in the same physical domain.
6. Use PX4 geo, matrix-wrap, and slew-rate helpers where applicable.
7. Keep fake target and home fallback as bench/development aids; preserve deterministic behavior.

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

### Required evidence

- `UNIT-002`, `SITL-002`, `SITL-003`, `SITL-006`, `BENCH-002`.

---

## Gate 3 — MAVLink target bridge hardening

**State:** implemented; SITL and deployed-link evidence remains required before verification.

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

**State:** implemented; bench and hardware startup-safety evidence remains required before verification.

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

---

## Gate 5 — Status, Events, and logging

**State:** planned.

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

**State:** planned.

### Work

1. Expand tracker unit coverage for geometry, mapping, angle wrap, park, sector clipping, state transitions, source filtering, prediction guards, and safe-state behavior.
2. Run tracker SITL through airframe 4099:

```bash
make px4_sitl_default
cd build/px4_sitl_default
PX4_SYS_AUTOSTART=4099 PX4_SIM_MODEL=none ./bin/px4
```

3. Capture manifests and raw logs for each SITL case.
4. Treat a dynamic circular test as a directional/continuity test unless its duration/speed truly cover the claimed arc.

### Acceptance criteria

All `UNIT-*` and `SITL-*` cases in [test_matrix.yaml](validation/antenna_tracker/test_matrix.yaml) have current evidence and no unexplained failure.

---

## Gate 7 — Bench and hardware stabilization

**State:** planned.

### Work

1. Validate Pixhawk 6C hardware output mapping, servo supply, direction, limits, reversal, park position, and no-startup-jump.
2. Verify that the IMU moves with the head and that the compass is valid under servo/motor current.
3. Verify deployed telemetry target ingress and source selection.
4. Start with static low-gain tests and bounded output, then dynamic target and loss/recovery tests.
5. Maintain an independent servo-power cutoff through the entire gate.
6. Improve bench operator UX: add an explicit `antenna_tracker servo_test
   start|stop` command and report the active test override in `status`. This
   must remain an explicit bench override, not a new tracking submode; normal
   tracker operation continues to use `TRK_MODE`.

### Acceptance criteria

`BENCH-001` through `BENCH-003` and `HW-001` through `HW-004` are verified with evidence manifests.

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

- continuous-rotation, relay, stepper, or encoder-specific actuator backends;
- yaw reversal/multi-turn strategies;
- advanced PID notch/filter work;
- custom output-function enums for tracker axes;
- QGC plugin/custom tracker UI;
- dead reckoning enabled by default.
