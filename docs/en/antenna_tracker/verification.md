# Verification

Verification is evidence-driven. A source file, successful compile, historical console copy, or build artifact alone does not verify a tracker behavior.

The machine-readable contract lives in `validation/antenna_tracker/`:

- `test_matrix.yaml` defines test IDs, gates, prerequisites, and acceptance criteria.
- `evidence.schema.yaml` defines one evidence manifest per run.
- `legacy/README.md` records limitations of older test transcripts.

## Evidence levels

| Level | Meaning |
|---|---|
| planned | Work is scoped but no implementation claim is made. |
| implemented | Code exists and is reviewed; no runtime claim yet. |
| blocked | A required prerequisite, setup, or evidence source prevents completion. |
| verified-sitl | Canonical tracker SITL test passed with a manifest and raw console/log attachments. |
| verified-bench | Intended flight controller and actuator mechanics passed a controlled bench test. |
| verified-hardware | Real telemetry, sensors, and closed-loop mechanics passed the required hardware case. |
| field-ready | Repeated outdoor dynamic and failure tests passed, with known limits documented. |

## Required gates

### Build and unit tests

- `BLD-001`: Build `px4_sitl_default`.
- `BLD-002`: Build `px4_fmu-v6c_antenna_tracker` and record provenance.
- `UNIT-001`: Geometry and angle-wrapping tests.
- `UNIT-002`: PID/mapping/saturation tests.
- `UNIT-003`: HEARTBEAT-qualified MAVLink target admission, source selection, and auto-lock tests.

### Canonical SITL tests

- `SITL-001`: Boot tracker airframe 4099 without Gazebo vehicle autostart.
- `SITL-002`: STOP and servo-test behavior.
- `SITL-003`: Fake target and home fallback geometry.

  The canonical SITL fixture uses `antenna_tracker test gpos-loss on` only on
  the POSIX build. It makes the tracker treat its local global-position input
  as invalid without stopping EKF2 or changing the uORB topic, so fresh
  `vehicle_attitude` remains available while the `TRK_HOME_*` fallback is
  exercised. This command is not compiled into NuttX flight-controller builds.
- `SITL-004`: Correct MAVLink ingress and static target acquisition.
- `SITL-005`: Dynamic target path with duration consistent with expected angular travel.
- `SITL-006`: Target timeout, safe state, and recovery.
- `SITL-007`: Specific system-ID filter.
- `SITL-008`: Auto-lock, competing source rejection, and lock release.
- `SITL-009`: AUTO park-settle reference capture.
- `SITL-010`: Requested/effective tracker mode and ARM/DISARM transitions.
- `SITL-011`: Tracker-specific Commander readiness and standard QGC arming semantics.

### Current ARM-gate verification status

The ARM-gate source has been built and exercised locally, including standard
MAVLink ARM/DISARM acknowledgement, HEARTBEAT arming state, disarmed AUTO park,
armed AUTO activation, target-missing park, servo-test cancellation on ARM, and
immediate park selection on the first logged tracker sample after DISARM.

These observations do not yet satisfy the formal evidence contract. Both
`SITL-010` and `SITL-011` remain `implemented` until their manifests include
raw console/ULog attachments and the stock-QGC Events view. SCAN, MANUAL, kill,
lockdown, termination, and retained hardware/safety arm blockers also require
explicit coverage. `SITL-009` must be rerun because the ARM transition now
invalidates and recaptures the AUTO head reference.

### Bench and hardware tests

- `BENCH-001`: MAIN1 yaw / MAIN2 pitch mapping and output calibration.
- `BENCH-002`: Servo direction, limited range, park pose, and startup no-jump behavior.
- `BENCH-003`: Moving-IMU yaw/pitch feedback and compass validation under servo load.
- `HW-001`: Target ingress over the deployed telemetry link.
- `HW-002`: Static bearing/elevation tracking at bounded gain/output.
- `HW-003`: Dynamic tracking and target-loss recovery.
- `HW-004`: Power-cycle, prearm, sensor-invalid, and emergency cutoff safety.
- `FIELD-001`: Repeated outdoor tracking with documented reachable sector and cable-wrap limits.

## Minimum run attachments

Each evidence manifest must identify:

- firmware commit, PX4 base tag, build target, artifact checksum, and flash use;
- board, airframe, target ingress link, and moving-IMU arrangement;
- exact relevant parameters and actuator mapping;
- test procedure and expected result;
- observed metrics and verdict;
- raw console log, parameter dump, ULog if available, and any photo/video stored outside Git.

## Required metrics

Capture measurable values rather than only `PASS` text:

- target acquisition and timeout times;
- target age at state transition;
- output range and slew;
- maximum bearing/pitch error for known static geometry;
- target source system/component;
- actuator/park values;
- ULog path and checksum when a log is available.

## Historical evidence

Older root transcripts and `logs/verification/phase1_sitl.txt` are historical notes only. They use the x500 autostart path or mismatched target ports and therefore do not satisfy the canonical SITL acceptance criteria.
