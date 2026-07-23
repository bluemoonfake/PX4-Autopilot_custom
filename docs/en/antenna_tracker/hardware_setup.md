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

For the bounded tracker sweep, first set `TRK_MODE=STOP`, then use
`antenna_tracker servo_test start yaw` (or `pitch`) and
`antenna_tracker servo_test stop` in the MAVLink console. The command is a
volatile one-axis bench override: it is cleared by reboot and the other axis
holds park. `antenna_tracker status` reports the selected axis. The legacy
`TRK_SERVO_TEST` parameter is automatically cleared and cannot start a sweep
after boot.

`TRK_REF_SETTLE` is the minimum time the *commanded* yaw and pitch outputs
must remain at park before AUTO captures its moving-IMU reference. Start with
the conservative default of 1.0 s and increase it for a slow or heavily loaded
mechanism. The firmware has no servo-position feedback, so this is not a
substitute for verifying physical park on the bench.

## Servo type and PWM rate

Use `TRK_YAW_SRV_T` and `TRK_PIT_SRV_T` for motion semantics:

- `0` (`POSITION`, default): output represents a physical servo position.
- `1` (`CONTINUOUS`): output represents rotation rate; `TRK_*_TRIM` is neutral/stop.

Use QGC Actuators `PWM_MAIN_TIMx`/`PWM_AUX_TIMx` for the electrical protocol and
rate. MAIN1 and MAIN2 use Timer 0 on the MicoAir H743 profile, so they cannot
have different PWM rates while they remain in that timer group. The tracker
board profile defaults `PWM_MAIN_TIM0=50` when MicoAir H743 boots airframe
4099; the generic airframe does not assume a timer layout. Changing the timer
setting requires reboot. Analog and digital positional servos both remain
`POSITION`. Select 100/200/400 Hz only when every active servo on the timer
group explicitly supports that rate.

For a continuous axis, set the corresponding PWM disarmed and failsafe values
to the measured neutral pulse (normally near 1500 us) before applying servo
power. It requires valid moving-IMU feedback for AUTO/SCAN/MANUAL, stops
immediately on a safe-state transition, and cannot use the bounded positional
`servo_test` sweep. Wrapped yaw does not provide turn counting or cable-wrap
protection.

`TRK_FAKE_EN` defaults to `0`. Set it to `1` only for SITL or a controlled
bench; while it is enabled, fake target parameters are the sole target source.
Disable it before any MAVLink ingress, timeout, or hardware tracking test.

`TRK_ALT_SRC` must remain `0` (GPS MSL). Any reserved/unverified value is
rejected and parks the tracker with `REASON_SOURCE_REJECTED`; it is never
silently interpreted as relative altitude.

## Orientation versus trim

| Issue | Correct adjustment |
|---|---|
| FC mounted in a different cardinal orientation | `SENS_BOARD_ROT` |
| Small horizon offset | level-horizon calibration / board offsets |
| Small servo horn neutral error | tracker servo trim after orientation is correct |
| Servo moves opposite direction | `TRK_YAW_REV` / `TRK_PIT_REV` or PX4 output reversal, but not both |

Do not compensate for a 90°/180° board orientation error by adding large tracker trim.

## Telemetry target link

The production airframe reserves **TELEM1** for MAVLink instance 0 in Normal mode at **57600 baud** (`MAV_0_CONFIG=101`, `MAV_0_MODE=0`, `SER_TEL1_BAUD=57600`). This maps to `/dev/ttyS5` on FMUv6C, `/dev/ttyS6` on FMUv6X, and `/dev/ttyS0` on MicoAir H743; verify the board label before wiring. QGC and the target UAV may share this routed telemetry link. For USB bench testing, use `Tools/antenna_tracker/usb_router.py`; it owns the USB device and injects test target traffic on UDP port 18570.

A target arrives on the normal MAVLink receiver path as `GLOBAL_POSITION_INT`. The initial policy accepts only messages whose source system ID equals `TRK_SYSID_TARGET`; a value of zero disables MAVLink target acceptance. System ID filtering is routing, not authentication. If the link must resist spoofing, plan MAVLink signing and transport security separately.

`TRK_HOME_EN` is disabled for production unless a verified tracker home has been
provisioned. Firmware rejects the unprovisioned `(TRK_HOME_LAT,TRK_HOME_LON) =
(0,0)` placeholder and parks if live global position is unavailable. A real
site may be on the equator or prime meridian: only the all-zero pair is
rejected. Use `antenna_tracker set_home` only while the tracker has a valid
global position, then run `param save`.

## Supported actuator semantics

Positional servos are the production default and command calibrated physical
positions. Continuous-rotation PWM servos are available per axis as an
explicit, feedback-dependent mode; they stop at neutral instead of returning
to a physical park pose. Relay, stepper, and encoder-bus actuators still require
separate drivers/feedback designs and are outside this firmware interface.
