# PX4 Antenna Tracker Firmware

This fork turns a dedicated PX4 flight controller into a two-axis antenna tracker. It is a **PX4-native airframe and module**, not a Gimbal v2 or mount-control integration.

::: warning
The firmware is a SITL-capable and limited bench-test candidate. It is **not field-ready** until the hardware, safety, and validation gates in [the implementation plan](../../../implementation_plan.md) have passed with recorded evidence.
:::

## What the firmware does today

- Receives a target UAV position from MAVLink `GLOBAL_POSITION_INT`.
- Publishes the target through `tracker_target_position`.
- Reads tracker attitude and global position from PX4 estimator topics.
- Computes bearing, elevation, distance, timeout state, and diagnostic status.
- Publishes normalized yaw and pitch commands through `actuator_servos`.
- Maps Servo 1 to yaw and Servo 2 to pitch through the PX4 output-function pipeline.
- Selects positional or continuous-rotation semantics independently per axis,
  while leaving PWM protocol/rate to the native PX4 output configuration.
- Provides a dedicated airframe, `4099_antenna_tracker`, and hardware build targets including `px4_fmu-v6c_antenna_tracker`, `px4_fmu-v6x_antenna_tracker`, and `micoair_h743_antenna_tracker`.

## Safety-critical architecture constraint

The current closed-loop design uses `vehicle_attitude` as the measured antenna orientation. Therefore the flight controller and its IMU **must be mounted rigidly on, and rotate with, the pan/tilt antenna assembly**.

If the FC remains fixed on the tripod, its attitude does not describe the antenna heading. Do not use AUTO closed-loop tracking in that arrangement without an external yaw/pitch encoder or a separately designed and calibrated open-loop controller.

## Documentation

- [Architecture](architecture.md) — current flow, target architecture, and source ownership.
- [Arming and tracker modes](arming_and_modes.md) — requested/effective mode state machine, Commander readiness, and validation status.
- [QGroundControl compatibility](qgroundcontrol.md) — the stock-QGC integration contract and limits.
- [Hardware setup](hardware_setup.md) — Pixhawk 6C, actuator mapping, IMU orientation, and telemetry.
- [Safety](safety.md) — required mechanical, power, timeout, and prearm precautions.
- [Build and flash](build_and_flash.md) — canonical SITL, firmware build, and artifact provenance procedures.
- [Verification](verification.md) — evidence gates and required test coverage.
- [MicoAir H743 validation](h743_validation.md) — board-specific bench, hardware, and field runbook.
- [ArduPilot gap reference](reference/ardupilot_gap.md) — useful comparison only; it is not a porting plan.

## Source entry points

| Layer | Primary source |
|---|---|
| Tracker module | `src/modules/antenna_tracker/` |
| MAVLink target bridge | `src/modules/mavlink/mavlink_receiver.cpp` and `.h` |
| Target/status uORB messages | `msg/TrackerTargetPosition.msg`, `msg/TrackerStatus.msg` |
| Hardware airframe | `ROMFS/px4fmu_common/init.d/airframes/4099_antenna_tracker` |
| POSIX/SITL airframe | `ROMFS/px4fmu_common/init.d-posix/airframes/4099_antenna_tracker` |
| FMUv6C target | `boards/px4/fmu-v6c/antenna_tracker.px4board` |
| MicoAir H743 target | `boards/micoair/h743/antenna_tracker.px4board` |
| Tracker utilities | `Tools/antenna_tracker/` |
| Test evidence contract | `validation/antenna_tracker/` |

## Status terminology

Do not treat a source file, a build artifact, or an unchecked historical transcript as verification. Every feature uses one of these states:

- **planned** — scoped but not implemented.
- **implemented** — source exists and is reviewed.
- **blocked** — a required prerequisite, setup, or evidence source is unavailable.
- **verified-sitl** — tested in the canonical tracker SITL setup with recorded evidence.
- **verified-bench** — tested with the intended controller and actuator setup on a controlled bench.
- **verified-hardware** — tested on the intended hardware with recorded evidence.
- **field-ready** — passed repeated hardware and field gates, including failure handling.

The canonical roadmap is [implementation_plan.md](../../../implementation_plan.md).
