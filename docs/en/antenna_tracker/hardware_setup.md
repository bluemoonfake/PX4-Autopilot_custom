# Hardware Setup

## Supported initial target

Supported hardware builds are:

```bash
# Pixhawk 6C
make px4_fmu-v6c_antenna_tracker

# Pixhawk 6X
make px4_fmu-v6x_antenna_tracker

# MicoAir H743 (v1)
make micoair_h743_antenna_tracker
```

The output artifact is expected under:

```text
build/px4_fmu-v6c_antenna_tracker/px4_fmu-v6c_antenna_tracker.px4
build/px4_fmu-v6x_antenna_tracker/px4_fmu-v6x_antenna_tracker.px4
build/micoair_h743_antenna_tracker/micoair_h743_antenna_tracker.px4
```

Do not treat an existing artifact as proof that the current checkout still builds. Record the commit, build command, artifact checksum, and flash use for every release candidate.

## Required moving-IMU arrangement

Mount the Pixhawk and all attitude sensors rigidly to the pan/tilt head such that:

- tracker forward axis matches antenna boresight after calibration;
- yaw rotation of the antenna rotates the IMU yaw;
- pitch rotation of the antenna rotates the IMU pitch;
- the compass is sufficiently separated from servo motors, high-current wiring, and ferrous material;
- moving wires cannot bind or wrap through the full allowed yaw sector.

If the Pixhawk is fixed to the tripod while only the antenna moves, the current closed loop has no antenna-angle feedback. Stop and add external angle sensing or use a separately engineered open-loop solution.

## Output mapping

The initial hardware airframe maps:

| Axis | Output | PX4 function |
|---|---|---:|
| yaw | MAIN1 | Servo 1 / 201 |
| pitch | MAIN2 | Servo 2 / 202 |

Validate wiring, rail power, and physical direction with the antenna/load disconnected or mechanically decoupled first.

The MAIN mapping is the production-candidate default. The current FMUv6X bench
uses AUX1/AUX2 because its MAIN bank is unavailable; that is a separate bench
profile and must not be copied into a production MAIN configuration.

## Required calibration record

Before AUTO, create an evidence-linked calibration record for the exact board
and mechanism: firmware SHA-256, output bank/pin, PWM min/max/disarmed/failsafe,
physical yaw/pitch min/max/park, trim, reversal, and servo supply. First prove
`+/-2 deg` steps; then prove `+/-5 deg` and return-to-park. Do not use PID gains
to compensate for an incorrect horn, endpoint, reversal, or mechanical travel.

## Bring-up sequence

1. Inspect the mechanism for physical stops, cable routing, backlash, and a safe park pose.
2. Add an independently accessible servo-power cutoff or emergency stop.
3. Power the FC and calibrate accelerometer, gyro, compass, and level horizon.
4. Set `SENS_BOARD_ROT` to the physical FC orientation.
5. With servo command trim at zero, verify in `antenna_tracker status` that rotating the assembly clockwise increases yaw and that pitch sign matches physical movement.
6. Configure PWM output min/max/disarmed/failsafe values conservatively in QGC Actuators. The airframe defaults MAIN1 to 1000/2000 us with 1500 us disarmed (yaw park), and MAIN2 to 1000/2000 us with 1000 us disarmed (0° pitch park). If either park angle is changed, set its PWM disarmed value to the calibrated park PWM before connecting servo power.
7. Run actuator tests with no RF load and enough clearance for all motion.
8. Verify yaw/pitch direction, neutral, and mechanical range one axis at a time.
9. Verify STOP and target-timeout behavior before enabling AUTO.
10. Only then test static target tracking at low gain and limited output range.

For the bounded tracker sweep, use `antenna_tracker servo_test start` and
`antenna_tracker servo_test stop` in the MAVLink console. The command resets
the sweep to its known start point and shows `Servo-test override: ACTIVE` in
`antenna_tracker status`. It is a bench override of `TRK_SERVO_TEST`, not a
tracker mode; stop it before using `TRK_MODE=AUTO`.

## Orientation versus trim

| Issue | Correct adjustment |
|---|---|
| FC mounted in a different cardinal orientation | `SENS_BOARD_ROT` |
| Small horizon offset | level-horizon calibration / board offsets |
| Small servo horn neutral error | tracker servo trim after orientation is correct |
| Servo moves opposite direction | output reversal or future tracker axis reversal parameter |

Do not compensate for a 90°/180° board orientation error by adding large tracker trim.

## Telemetry target link

The production airframe reserves **TELEM1** for MAVLink instance 0 in Normal mode at **57600 baud** (`MAV_0_CONFIG=101`, `MAV_0_MODE=0`, `SER_TEL1_BAUD=57600`). This maps to `/dev/ttyS5` on FMUv6C, `/dev/ttyS6` on FMUv6X, and `/dev/ttyS0` on MicoAir H743; verify the board label before wiring. QGC and the target UAV may share this routed telemetry link. For USB bench testing, use `Tools/antenna_tracker/usb_router.py`; it owns the USB device and injects test target traffic on UDP port 18570.

A target must arrive on the normal MAVLink receiver path with a periodic `HEARTBEAT` and `GLOBAL_POSITION_INT` from the same `(sysid, compid)`. The first production policy accepts component `MAV_COMP_ID_AUTOPILOT1`, requires a fresh non-GCS heartbeat, and then applies `TRK_SYSID_TGT` or validated auto-lock. The source system ID and heartbeat are routing/qualification mechanisms, not authentication. If the link must resist spoofing, plan MAVLink signing and transport security separately.

`TRK_HOME_EN` is disabled for production unless a verified tracker home has been
provisioned. Firmware rejects the unprovisioned `(TRK_HOME_LAT,TRK_HOME_LON) =
(0,0)` placeholder and parks if live global position is unavailable. A real
site may be on the equator or prime meridian: only the all-zero pair is
rejected. Use `antenna_tracker set_home` only while the tracker has a valid
global position, then run `param save`.

## Positional servo assumption

The controller commands calibrated physical positions, respects mechanical sectors, and parks safely. Continuous-rotation, relay, stepper, or encoder-based axes require their own actuator/feedback design and are outside the first stable milestone.
