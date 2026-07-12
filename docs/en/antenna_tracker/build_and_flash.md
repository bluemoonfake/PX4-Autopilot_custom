# Build, SITL, and Flash

## Prerequisites

This fork is based on PX4 v1.17.0. Initialize submodules and use the tracker branch before building. Record the exact commit and submodule state for any test or release candidate.

## Canonical tracker SITL boot

Build the normal POSIX target:

```bash
make px4_sitl_default
```

Boot the tracker airframe directly:

```bash
cd build/px4_sitl_default
PX4_SYS_AUTOSTART=4099 PX4_SIM_MODEL=none ./bin/px4
```

This is the canonical tracker firmware boot path. Do **not** use `make px4_sitl gz_x500` as the pass criterion for airframe 4099: the Gazebo x500 model applies its own multicopter autostart and does not validate the tracker airframe.

In the PX4 shell, confirm:

```sh
param show SYS_AUTOSTART
param show MAV_TYPE
param show TRK_MODE
param show TRK_QGC_MODE
antenna_tracker status
listener vehicle_attitude
listener vehicle_global_position
listener tracker_target_position
listener tracker_status
listener actuator_servos
```

Expected baseline:

```text
SYS_AUTOSTART = 4099
MAV_TYPE = 5
antenna_tracker is running
vehicle_attitude and vehicle_global_position are fresh
```

## Send a SITL target

Use the tracker-specific utility:

```bash
python3 Tools/antenna_tracker/send_target.py \
  --port udpout:127.0.0.1:18570 \
  --mode static \
  --sysid 2 \
  --duration 30
```

The script sends `GLOBAL_POSITION_INT`. The port must match the PX4 MAVLink receiver instance used by the tracker SITL setup. Capture the selected port and the actual console startup message in the test evidence manifest.

For circular tests, do not claim one complete orbit unless duration and speed cover it. With the current defaults, `radius=200 m` and `speed=15 m/s` take roughly 84 seconds for a full revolution.

## Build the hardware firmware

```bash
make px4_fmu-v6c_antenna_tracker
```

Expected artifact:

```text
build/px4_fmu-v6c_antenna_tracker/px4_fmu-v6c_antenna_tracker.px4
```

Before flash, capture:

```bash
git rev-parse HEAD
git submodule status
sha256sum build/px4_fmu-v6c_antenna_tracker/px4_fmu-v6c_antenna_tracker.px4
```

Also retain the build output reporting flash use. A stale artifact copied from a previous checkout is not valid release evidence.

## Flash and first boot

Use the PX4/QGC firmware upload procedure appropriate for the board. On first hardware boot:

1. leave servo power disabled or mechanically decouple the servos;
2. verify `SYS_AUTOSTART=4099`, `MAV_TYPE=5`, and PWM functions;
3. calibrate sensors and verify board orientation;
4. configure PWM output min/max/disarmed/failsafe values;
5. perform safe actuator mapping before enabling target tracking.

## Debug order

When tracking does not behave as expected, inspect the pipeline in this order:

```text
vehicle_attitude
vehicle_global_position
tracker_target_position
tracker_status
actuator_servos
actuator_outputs
physical PWM/servo motion
```

This isolates estimator, target ingress, tracker logic, PX4 output mapping, and hardware issues without making assumptions from a single symptom.