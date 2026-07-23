# Arming, Readiness, and Tracker Modes

## Purpose

This document defines the implemented arming increment: make stock QGroundControl
arming meaningful for the antenna tracker and prevent `TRK_MODE` from moving the
tracker until PX4 Commander has armed the system.

The implementation must preserve the existing tracker-mode policy:

- `TRK_MODE` is the only authority for tracker `STOP`, `AUTO`, `SCAN`, and
  `MANUAL` submodes;
- PX4 navigation states such as Position, Altitude, and Manual are not tracker
  submodes and must never select tracker motion;
- no custom QGC plugin, Gimbal v2 path, external navigation mode, or forced-arm
  workflow is introduced by this change.

## Requested mode versus effective mode

`TRK_MODE` is a persistent **requested mode**. Arming controls whether that
request may become the **effective mode**.

| PX4 actuator state | Requested `TRK_MODE` | Effective tracker behavior |
|---|---|---|
| disarmed | STOP | hold configured park pose |
| disarmed | AUTO/SCAN/MANUAL | hold park; report waiting for arm |
| armed | STOP | hold configured park pose |
| armed | AUTO | use the existing park/settle/reference sequence, then track when sensors and target are valid |
| armed | SCAN | scan only inside the configured mechanical sector |
| armed | MANUAL | accept only fresh valid manual input; otherwise park |
| kill, lockdown, or termination | any | reset controllers and select the documented safe output immediately |

Arming must not write a different value to `TRK_MODE`. Disarming must not erase
the operator's requested mode. A reboot with `TRK_MODE=AUTO` must therefore boot
at park and remain waiting for arm.

## Runtime permission source

The tracker module should subscribe to `actuator_armed` and use these fields as
the final runtime permission:

```text
armed && !kill && !lockdown && !termination
```

The permission check must run before AUTO, SCAN, or MANUAL dispatch. Target
messages may continue to be received and cached while disarmed, but they cannot
move the servos.

Required transition behavior:

### Disarmed to armed

1. Reset PID integrators, derivative history, and stale scan state.
2. Preserve the requested `TRK_MODE`.
3. For AUTO, command park and complete `TRK_REF_SETTLE` before capturing the
   moving-IMU reference.
4. Start active motion only when the existing mode-specific sensor, source, and
   target checks pass.

### Armed to disarmed

1. Stop AUTO, SCAN, or MANUAL dispatch within one 50 Hz control cycle.
2. Reset both axis controllers and target-rate history.
3. Select the configured park command and report the disarmed reason.

### Servo test exception

The volatile, one-axis servo test is a bench-maintenance exception. It remains
available only while disarmed, with `TRK_MODE=STOP`, through the explicit
`antenna_tracker servo_test` command. It must stop immediately if the system
arms, `TRK_MODE` leaves STOP, or kill/lockdown/termination becomes active.

## `COM_PREARM_MODE` and physical motion

The airframe currently uses `COM_PREARM_MODE=2`, allowing non-throttling servo
outputs while disarmed. This is necessary to command the calibrated park pose
before active operation, but it means **disarmed is not a guarantee of no servo
motion**.

The ARM gate guarantees that tracking, scanning, and manual tracker modes do not
run while disarmed. It does not replace the independent servo-power cutoff or
the calibrated PWM disarmed/failsafe values required by the safety procedure.

## Commander and stock-QGC readiness

QGC obtains armability from normal PX4 Commander health and arming state. The
tracker must continue to use the standard `MAV_CMD_COMPONENT_ARM_DISARM` path and
normal HEARTBEAT armed bit.

The baseline rejection and subsequent behavior are captured from:

```sh
commander check
listener actuator_armed
listener vehicle_status
listener health_report
```

and the corresponding PX4 Events shown by QGC.

For `MAV_TYPE=5`, the readiness policy distinguishes between:

### Checks that remain arm-blocking

- required IMU/attitude sensors are initialized and calibrated;
- tracker module and output configuration are available;
- safety switch, power, hard-fault, kill, lockdown, and termination checks;
- any fault that makes the configured park command unsafe or unavailable.

### Conditions handled as runtime safe states

- no target or stale target;
- no live global position when no provisioned home is available;
- target outside the reachable sector or below minimum distance;
- AUTO reference still settling;
- manual input absent while MANUAL is requested.

These runtime conditions may prevent movement after arming, but they should not
be disguised as unsupported Position/Altitude flight-mode failures. The module
must park and publish a specific state reason instead.

The Commander change must be narrowly conditioned on the tracker system type.
Do not disable checks globally, use force-arm, set broad circuit breakers, or
start multicopter/fixed-wing/rover controllers to make QGC display Ready.

## Status and Events contract

Append fields to `TrackerStatus` rather than reordering existing fields:

```text
bool armed
uint8 requested_mode
uint8 effective_mode
```

Add an explicit waiting-for-arm state/reason. `antenna_tracker status`, ULog,
and PX4 Events must make these cases distinguishable:

- STOP requested;
- non-STOP mode waiting for arm;
- armed but waiting for AUTO reference;
- armed but waiting for valid sensor/position/target data;
- active tracking, scanning, or manual control;
- disarmed or terminated while previously active.

## Implementation and evidence status

The source review confirms that the runtime gate, tracker-specific Commander
mode requirements, appended status fields, Events, and the POSIX SIH pedestal
default are present. The current evidence level remains `implemented`: local
build/test artifacts and the MicoAir H743 ARM-gate run are useful development
evidence, but no conforming `SITL-010` or `SITL-011` run manifest exists yet.

Observed locally on the current working tree:

- `px4_sitl_default` builds successfully;
- the tracker-specific Commander mode-requirement unit test passes;
- disarmed AUTO remains at park and reports effective STOP;
- standard MAVLink ARM/DISARM produces accepted command acknowledgements and
  matching HEARTBEAT armed state;
- on MicoAir H743, `ready_to_arm` was true, the health/arming warning and error
  masks were zero, and standard GCS ARM/DISARM acknowledgements were result 0;
- an armed tracker with no target remains armed and parks with a runtime target
  reason;
- an active servo test stops when the system arms;
- the first logged tracker sample after DISARM selects the exact configured park
  command;
- disarmed requested STOP now reports IDLE/STOP, while requested AUTO reports
  waiting-for-ARM with effective STOP.

Still required before formal verification:

- a dedicated state-transition unit test for the module glue;
- explicit SCAN and MANUAL ARM-gate cases;
- explicit kill, lockdown, and termination latency cases;
- retain disarmed STOP versus non-STOP waiting-for-ARM in final regression
  evidence;
- stock-QGC PX4 Events, command, HEARTBEAT, and Ready/Not Ready capture;
- evidence that retained tracker-relevant hardware and safety failures still
  block ARM;
- conforming `SITL-009`, `SITL-010`, and `SITL-011` manifests with checksummed
  raw attachments;
- `HW-004` before powered hardware completion is claimed.

## Implemented patch sequence

### Patch A — Baseline and observability

**Status:** partially complete. The Commander/SITL baseline was inspected, but
the formal stock-QGC rejection capture is not yet recorded.

- Reproduce the current QGC arming denial on airframe 4099.
- Record Commander Events and the relevant uORB topics.
- Add the status fields and state/reason constants needed by the tests.
- Do not change arming decisions in this patch.

### Patch B — Runtime ARM gate

**Status:** implemented and partially exercised in local SITL. Dedicated module
state-transition unit coverage and the remaining safety branches are pending.

- Add the `actuator_armed` subscription.
- Gate normal mode dispatch before AUTO/SCAN/MANUAL.
- Implement deterministic arm/disarm edge handling and servo-test cancellation.
- Add unit coverage for the state transitions. This remains open.

### Patch C — Tracker-specific Commander readiness

**Status:** implemented and partially exercised in local SITL.

- Use the Patch A evidence to identify the exact irrelevant mode requirements.
- Apply the smallest tracker-only change in Commander health/arming logic.
- Preserve all tracker-relevant hardware and safety failures.
- Verify standard ARM/DISARM command acknowledgement and HEARTBEAT state. The
  MAVLink path was observed locally; stock-QGC capture remains open.

### Patch D — Regression and evidence

**Status:** selected unit tests and local SITL scenarios passed. The formal test
matrix is not complete, so `SITL-010` and `SITL-011` remain `implemented`.

- Run `SITL-010` and `SITL-011` from the verification matrix.
- Re-run STOP, timeout, reference-settle, and servo-test regressions.
- Build the intended hardware target and complete the relevant bench/HW safety
  case before connecting servo power.

## Canonical development commands

Build and boot the tracker without the x500 autostart:

```bash
make px4_sitl_default
cd build/px4_sitl_default
PX4_SYS_AUTOSTART=4099 PX4_SIM_MODEL=none PX4_PARAM_SIH_VEHICLE_TYPE=5 ./bin/px4
```

`SIH_VEHICLE_TYPE=5` models the tracker as a ground-supported pedestal. The
same value is the airframe default; the explicit environment override also
corrects an existing SITL parameter file that still contains the old SIH
quadcopter value.

Do not use `make px4_sitl gz_x500` as tracker evidence. Build only the intended
hardware target before flash or bench validation:

```bash
make px4_fmu-v6c_antenna_tracker
make px4_fmu-v6x_antenna_tracker
make micoair_h743_antenna_tracker
```

Record build logs, commit and submodule state, artifact checksum, parameter
dump, console output, QGC Events, and ULog according to the validation schema.
