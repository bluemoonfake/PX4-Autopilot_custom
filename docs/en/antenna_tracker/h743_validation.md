# MicoAir H743 validation runbook

This is the hardware runbook for the MicoAir H743 tracker profile. The current
pre-field run is recorded in
[`pre_field_summary.md`](../../../validation/antenna_tracker/runs/hardware/pre-field-20260719T000555Z/pre_field_summary.md).
The follow-up pitch retest is recorded in
[`retest_summary.md`](../../../validation/antenna_tracker/runs/hardware/pre-field-resume-20260719T005335Z/retest_summary.md).
The second pitch retest is recorded in
[`retest_summary.md`](../../../validation/antenna_tracker/runs/hardware/pre-field-resume2-20260719T010429Z/retest_summary.md).
The 50 Hz PWM retest is recorded in
[`pwm50_summary.md`](../../../validation/antenna_tracker/runs/hardware/pre-field-pwm50-20260719T012434Z/pwm50_summary.md).
The selected-compass and independent-cutoff run is recorded in
[`bench003_hw004_summary.md`](../../../validation/antenna_tracker/runs/hardware/bench003-hw004-20260719T025023Z/bench003_hw004_summary.md).
The limited-output BENCH-003 retest is recorded in
[`bench003_retest_summary.md`](../../../validation/antenna_tracker/runs/hardware/bench003-retest-20260719T050450Z/bench003_retest_summary.md).
The staged-amplitude follow-up is recorded in
[`bench003_staged_summary.md`](../../../validation/antenna_tracker/runs/hardware/bench003-staged-20260719T051236Z/bench003_staged_summary.md).
The post-calibration Stage 1 retest is recorded in
[`bench003_postcal_summary.md`](../../../validation/antenna_tracker/runs/hardware/bench003-postcal-20260719T052802Z/bench003_postcal_summary.md).
The internal-compass comparison is recorded in
[`bench003_internal_mag_summary.md`](../../../validation/antenna_tracker/runs/hardware/bench003-internalmag-20260719T054023Z/bench003_internal_mag_summary.md).
The internal-compass repeat is recorded in
[`bench003_internal_mag_repeat_summary.md`](../../../validation/antenna_tracker/runs/hardware/bench003-internalmag-repeat-20260719T060859Z/bench003_internal_mag_repeat_summary.md).
The explicit external-compass repeat is recorded in
[`bench003_external_mag_repeat_summary.md`](../../../validation/antenna_tracker/runs/hardware/bench003-externalmag-repeat-20260719T063044Z/bench003_external_mag_repeat_summary.md).
The post-manual-rotation compass-timeout diagnostic is recorded in
[`manual_rotation_summary.md`](../../../validation/antenna_tracker/runs/hardware/manual-rotation-externalmag-20260719T065009Z/manual_rotation_summary.md).
The unchanged-parameter bounded direct-step diagnostic is recorded in
[`currentparams_direct_step_summary.md`](../../../validation/antenna_tracker/runs/hardware/currentparams-directstep-20260719T070147Z/currentparams_direct_step_summary.md).
The full-transition bounded BENCH-003 retest is recorded in
[`bench003_fullcapture_summary.md`](../../../validation/antenna_tracker/runs/hardware/bench003-fullcapture-20260719T072059Z/bench003_fullcapture_summary.md).
The bounded static-yaw HW-002 attempt is recorded in
[`hw002_static_bounded_summary.md`](../../../validation/antenna_tracker/runs/hardware/hw002-static-bounded-20260719T074548Z/hw002_static_bounded_summary.md).
The zero-PID HW-002 repeat is recorded in
[`hw002_pidzero_summary.md`](../../../validation/antenna_tracker/runs/hardware/hw002-pidzero-20260719T081331Z/hw002_pidzero_summary.md).
The build-only positional PID reverse fix is recorded in
[`pid_reverse_build_summary.md`](../../../validation/antenna_tracker/runs/hardware/pid-reverse-build-20260719T083021Z/pid_reverse_build_summary.md).
Its hardware flash is recorded in
[`flash_summary.md`](../../../validation/antenna_tracker/runs/hardware/pid-reverse-flash-20260719T083913Z/flash_summary.md).
The initial rate-guard attempt and bounded P=0.05 retest are recorded in
[`hw002_pid_small_rate_abort_summary.md`](../../../validation/antenna_tracker/runs/hardware/hw002-pid-small-20260719T084708Z/hw002_pid_small_rate_abort_summary.md)
and
[`hw002_pid_small_summary.md`](../../../validation/antenna_tracker/runs/hardware/hw002-pid-small-20260719T084840Z/hw002_pid_small_summary.md).
The corrected -90..90 yaw-range preliminary and primary reruns are recorded in
[`hw002_yaw90_rate_abort_summary.md`](../../../validation/antenna_tracker/runs/hardware/hw002-yaw90-p005-20260719T090619Z/hw002_yaw90_rate_abort_summary.md)
and
[`hw002_yaw90_summary.md`](../../../validation/antenna_tracker/runs/hardware/hw002-yaw90-p005-slew4-20260719T090754Z/hw002_yaw90_summary.md).
The PWM-limited -63..63 preliminary and complete ramped runs are recorded in
[`hw002_yaw63_rate_abort_summary.md`](../../../validation/antenna_tracker/runs/hardware/hw002-yaw63-p005-slew8-20260719T091618Z/hw002_yaw63_rate_abort_summary.md)
and
[`hw002_yaw63_ramped_summary.md`](../../../validation/antenna_tracker/runs/hardware/hw002-yaw63-p005-ramped-20260719T091806Z/hw002_yaw63_ramped_summary.md).
The P=0.8 ramped yaw qualification is recorded in
[`hw002_yaw63_p080_summary.md`](../../../validation/antenna_tracker/runs/hardware/hw002-yaw63-p080-ramped-20260719T092728Z/hw002_yaw63_p080_summary.md).
The overall direct-step yaw and positive-pitch qualification is recorded in
[`hw002_overall_summary.md`](../../../validation/antenna_tracker/runs/hardware/hw002-full-pitch-positive-p080-slew12-20260719T094253Z/hw002_overall_summary.md).
The board has been flashed and the USB target path is verified, but closed-loop
pointing fails the direct static-accuracy gate on yaw return/minus travel and
positive pitch; earlier runs also retain compass-installation limits outside
the qualified bounded envelope.

The preceding per-axis servo-type candidate built successfully as artifact SHA-256
`00f70dd6a9fae553bb4d3c4904e45e33df2f47510586f45a343c7f24f43726a4`
at 81.91% flash usage, with 14/14 tracker unit tests and the per-axis SITL
smoke test passing. Its MicoAir board default selects 50 Hz only for airframe
4099, while generic SITL remains independent of board timer parameters. This
artifact was uploaded, verified, rebooted, and used for the earlier
BENCH-003/HW-004 evidence below. The current flashed artifact is identified in
the BLD-004 row and post-fix evidence.

## Current hardware result — 2026-07-19

| Gate | Result | Evidence |
|---|---|---|
| BLD-004 | pass | Current H743 artifact SHA-256 `54a588aa...af37`, 81.91% flash use, upload/verify/reboot completed |
| BENCH-001 | pass | Servo1 produced 30 us PWM / 1.801 deg yaw span; Servo2 produced 21 us / 0.772 deg pitch span; inactive output stayed at 1500 us |
| BENCH-002 | blocked, not a mechanical fail | 50 Hz fixed pitch park (0.020 deg residual); operator confirmed yaw physically returned to its initial mark, while reported attitude retained a 25.740 deg offset |
| BENCH-003 | pass in bounded envelope only | Full-transition capture at 1451..1549 us measured 4.541/4.264 deg yaw spans, -0.62%/-1.84% active field changes, and -2.21% final field residual; larger travel and external-compass selection remain unqualified |
| HW-001 | pass for USB | wrong sysid rejected; matching sysid accepted into `tracker_target_position` |
| HW-002 | fail overall | Direct yaw passed +2/-2/+5 but failed -5 and park return at 1.155/0.747 deg max error; positive pitch +2/+5 failed at 1.714/1.926 deg max error; negative pitch remains unqualified by the 0..90 deg profile |
| HW-003 | blocked | Requires passing calibrated static tracking on both required axes |
| HW-004 | pass | operator cut the independent servo rail while FC remained live; termination forced 49/49 samples to 1500/1500 us and reboot cleared the latch with 94/94 neutral samples |
| FIELD-001 | **not authorized** | BENCH-003, HW-002 and HW-003 are not passed |

The final PX4 parameter dump contains 876/876 records. All temporary test
settings were restored; the only before/final difference is the automatic
`COM_FLIGHT_UUID` counter changing from 0 to 3 after ARM test sessions.

Timer 0 at 50 Hz resolved the pitch park failure, and the operator confirmed
the positional yaw mechanism returned to its initial physical mark. The new
selected-magnetometer run now explains the 25.740-degree residual: device
527625 measured 0.412 G at baseline, 1.887 G during yaw movement, and 0.291 G
after STOP. The moving IMU produced a 161.42-degree unwrapped yaw span, so the
remaining blocker is magnetic installation/interference, not missing IMU
motion. Relocate the compass away from servo motors and high-current wiring,
correct power routing/shielding, recalibrate, and repeat BENCH-003. Do not use
the non-selected `HIGHRES_IMU` magnetic stream as the compass verdict.

HW-004 is complete and ended with servo power cut off. The current normalized
test sweep can produce 1164..1836 us with the persisted 800..2200 us yaw
endpoints, so powered retests must temporarily narrow the output range.

The operator subsequently re-enabled servo power for a limited BENCH-003
retest. Temporary 1470..1530 us MAIN1 endpoints yielded an actual
1486..1514 us sweep and 1.736 degrees of measured yaw movement. Selected field
changed from 0.26304 G to 0.25642 G (-2.52%); heading test ratio stayed at or
below 0.09101, disturbed-field and heading-failure flags stayed false, and the
quaternion reset count did not change. This subtest passes, but it does not
clear BENCH-003 because the broader range that produced the 4.58x disturbance
was not exercised. Endpoints were restored to 800..2200 us after STOP.

The next staged run increased amplitude with a 10% selected-field stop limit.
The 1452..1548 us stage passed: yaw span was 8.039 degrees and field changed
only -1.08%. The 1400..1600 us stage failed: yaw span was 17.009 degrees and
field changed -16.61%. After 20 seconds at 1500 us, selected field was still
-15.71% from baseline and yaw settled 25.03 degrees away from its baseline.
PX4 did not raise a disturbed-field flag, so explicit field-norm and returned
heading limits are mandatory. No larger stage was run.

After the operator recalibrated the compass and reconnected USB and servo
power, the bounded Stage 1 was repeated. Baseline selected field was 0.30380 G,
but it fell to 0.18921 G immediately after the 1452..1548 us sweep (-37.72%)
and remained at 0.19241 G (-36.66%) after settling. Reported yaw retained a
55.70-degree residual while the outputs were neutral. EKF fault and disturbed
field flags remained clear, again demonstrating that the explicit field-norm
limit is required. Stage 2 was not run. Recalibration alone therefore does not
clear the physical compass/servo-power installation blocker.

The next comparison disabled external device 527625 and selected internal
IST8310 device 396817 (`CAL_MAG0_PRIO=100`, `CAL_MAG1_PRIO=0`). Selection was
confirmed after reboot by `vehicle_magnetometer`, both EKF instances, and
`sensors status`. Internal field was 0.31948 G at baseline and 0.26669 G after
the same Stage 1 (-16.52%); after settling it remained at 0.26626 G (-16.66%)
with a 32.97-degree reported-yaw residual. The internal compass is better than
the external result but still fails. Stage 2 was skipped, and internal priority
remains persisted for the next diagnostic.

The internal-mag Stage 1 was then repeated after priorities had changed to
`CAL_MAG0_PRIO=75` and `CAL_MAG1_PRIO=50`; device 396817 remained selected.
An 80-sample active window was clean (+0.23%, complete range
0.31618..0.32575 G), but the later travel produced a 37.894-degree reported-yaw
span. Immediately after STOP the selected field was -25.62%, and after 15
seconds at neutral it remained -25.16% with a 38.71-degree yaw residual. This
confirms that a short clean window does not clear BENCH-003 and localizes the
failure to later travel/return rather than a constant current offset.

External priority was then made unambiguous with `CAL_MAG0_PRIO=0` and
`CAL_MAG1_PRIO=100`, followed by reboot and device-ID verification. After one
isolated 0.79719 G neutral spike, an 80-sample baseline stabilized at 0.31016 G.
The first 80 active samples were clean (+0.12%), but a later batch reached
0.36515 G (+17.73%) and triggered STOP. EKF heading innovation failed
immediately after STOP. After 15 seconds at neutral the field remained +15.49%,
although yaw returned within 0.30 degrees. The external compass therefore
improves returned heading but still fails field integrity.

The following manual-rotation diagnostic exposed a more direct fault. Selected
external device 527625 stopped publishing for more than 40 seconds, while
internal device 396817 remained current. The sensor validator reported
`best=-1`, two failsafe events, `TOUT`, and zero confidence. The QMC5883L driver
showed 112 resets, two bad-register events, and three bad-transfer events;
IST8310 errors remained zero. EKF quaternion reset count increased to two and
the event log reported no valid data from Compass 1. This invalidates the run as
a field-norm comparison and prioritizes external I2C cable/connector/power/
ground continuity and strain relief before any further sweep.

The latest diagnostic deliberately preserved the operator's current parameters:
`CAL_MAG0_PRIO=75`, `CAL_MAG1_PRIO=0`, and MAIN1 endpoints 800..2200 us. The
standard +/-0.5 tracker sweep was unsafe with those endpoints, so volatile
`actuator_test` commands were limited to +/-0.07 and released automatically
after five seconds. The positive command was verified at 1549 us and the
operator subsequently confirmed normal servo/mechanism motion, but reported
yaw spanned only 0.027 degrees and did not represent the observed movement.
Selected internal-field changes were -4.31% for the negative step and -9.93%
for the positive step; after 15 seconds back at 1500 us, the field was still
-12.65% with a 3.32-degree yaw residual. The FC ended DISARMED/STOP at
1500/1500 us. No parameter was written, no larger command was attempted, and
BENCH-003 remains blocked. Verify that the FC/IMU rotates rigidly with the
antenna head before interpreting `vehicle_attitude` as the mechanical angle.

After USB was reconnected, the next retest captured telemetry before both
commands. Baseline field was stable at 0.33184 G (0.474% CV). Volatile -0.07
and +0.07 steps produced verified 1451..1500 and 1500..1549 us transitions,
with 4.541 and 4.264 degrees of moving-IMU yaw. Selected internal-field changes
were -0.62% and -1.84%; after 15 seconds neutral and final verification the
residual was -2.21%. The 10% stop threshold was never crossed, all commands
were accepted, and the run ended disarmed at 1500/1500 us. This passes the
bounded 1451..1549 us BENCH-003 envelope. It does not qualify 1400..1600 us,
the full persisted endpoints, or the external compass, which was healthy at
preflight but disabled by priority.

HW-002 then used a temporary synthetic home because GPS had no fix, target
system 3, a +/-5-degree yaw sector, `TRK_YAW_REV=1`, and MAIN1 limited to
1451..1549 us. Direction was correct and the selected-field rolling change
remained within -6.48..+1.42%. The +5-degree phase passed with 0.535-degree
error, but +2, -2, -5, and return-to-park errors were respectively 0.689,
0.928, 1.990, and 0.709 degrees, exceeding their 0.5/0.5/1.0/0.5-degree
limits. This is a calibration/mechanics failure, not permission to increase
PID gain. Pitch was parked and remains untested. STOP at 1500/1500 us was
verified while armed, then the FC was disarmed and every temporary parameter
was restored and read back.

At the operator's request, the same static-yaw sequence was repeated with all
yaw and pitch PID/FF terms zero. This did not clear the error. Steady +2, -2,
+5, and return errors were 1.200, 0.643, 3.486, and 1.725 degrees. The -5 phase
reached 0.126-degree steady error but overshot by 1.441 degrees. Field rolling
change remained safe at -4.33..+2.49%, and outputs remained 1451..1548 us with
pitch fixed at 1500 us. This direction- and history-dependent result excludes
PID as the sole cause and keeps mechanical/mapping calibration as the HW-002
blocker. Temporary setup parameters were restored; PID/FF remains zero as
requested.

Source review then identified a firmware defect in the positional correction
path: physical-angle mapping honored the axis reverse parameter, but PID/FF
correction did not. The correction now uses the same direction as the mapper,
and a normal/reverse regression test was added. `unit-TrackerMath` passes 15/15
and the H743 target builds at 81.91% flash use. The new build-only artifact is
SHA-256 `54a588aa1cb123d552b21b1a60469c244e2022a4d6b702dc1062279ff5d0af37`.
It was subsequently flashed: board ID 1166 was erased, programmed and verified
at 100%, then rebooted with airframe 4099. The first P=0.05 attempt safely
stopped on a conservative 0.5 rad/s transition guard. A second attempt inserted
zero-degree phases. Both signs responded correctly with no overall direction
regression, but the sub-microsecond P=0.05 correction cannot be isolated from
the position command in hardware; its sign is covered directly by the 15/15
unit regression. The +2, -2 and +5 steady errors were 0.649, 2.158 and 5.815
degrees. The -5 phase was aborted when measured relative yaw reached -7.53
degrees. Output
remained within 1451..1536 us, pitch stayed at 1500 us, and field change stayed
below the 10% threshold. This leaves mechanical angle/PWM mapping and
history-dependent travel as the HW-002 blocker; increasing gain is not
authorized. The final independent check found STOP/disarmed, 1500/1500 us,
all temporary parameters restored and PID/FF zero.

The operator then clarified that the physical positional-yaw range is only
-90..+90 degrees. The rerun retained the real MAIN1 800..2200 us endpoints
instead of rescaling the entire normalized range into 1451..1549 us. With
P=0.05 and a temporary four-second slew, all four static target dwells
completed. +2 degrees passed with 0.371-degree error; -2, +5 and -5 failed with
0.943, 1.424 and 1.494-degree errors. Output remained 1459..1541 us, pitch was
1500 us and field change stayed within -2.33..+1.75%. The return phase stopped
on the unchanged 0.8 rad/s guard at 0.871 rad/s. STOP later settled within
0.303 degrees, but cannot replace the incomplete armed return criterion.
HW-002 therefore remains failed. The board safely retains the corrected
`TRK_YAW_MIND=-90`, `TRK_YAW_MAXD=90` and `TRK_YAW_REV=1`; PID/FF is zero,
slew is restored to two seconds, and outputs are disarmed at 1500/1500 us.

The operator then supplied the servo's full calibration: 500..2500 us spans
180 degrees, while the installation intentionally limits PWM to 800..2200 us.
The matching reachable range is therefore -63..+63 degrees, not -90..+90.
With this calibrated pair, P=0.05, an eight-second temporary slew and one-degree
ramps between static dwells, the complete scenario finished without a safety
abort. +2, -2 and return passed at 0.243, 0.208 and 0.182-degree error. +5 and
-5 still failed at 1.383 and 1.305-degree error. Maximum yaw rate was 0.580
rad/s, output remained 1443..1556 us, pitch stayed at 1500 us, and field change
remained -1.14..+1.86%. This supersedes the retained -90..90 profile above.
The board now safely retains -63..63 and reverse 1, while PID/FF is zero and
slew is restored to two seconds. HW-002 remains failed on five-degree accuracy
and missing pitch validation.

Yaw P was then increased to 0.8 while I/D/FF remained zero. With the calibrated
-63..63 range, eight-second temporary slew and one-degree ramps, every static
yaw dwell passed: +2, -2, +5, -5 and return errors were 0.050, 0.180, 0.750,
0.578 and 0.145 degrees. Maximum yaw rate was 0.662 rad/s, output remained
1435..1563 us, field change stayed within -1.79..+1.94%, and no guard fired.
This qualifies ramped yaw static accuracy only. Direct zero-to-five-degree step
behavior is not qualified because the ramps deliberately avoid the earlier
rate abort, and pitch is still untested. HW-002 therefore changes from failed
yaw accuracy to blocked on these remaining cases, not to pass. The board
retains P=0.8, range -63..63 and reverse 1 while STOP/disarmed; slew is restored
to two seconds and I/D/FF remain zero.

The overall follow-up then used direct park-referenced targets. With a
12-second temporary slew, yaw +2, -2 and +5 passed at 0.403, 0.366 and 0.592
degrees maximum final error. Yaw -5 failed at 1.155 degrees and return-to-park
failed at 0.747 degrees. Maximum yaw rate was 0.758 rad/s, output stayed within
1439..1568 us and selected-field change stayed within -1.93..+3.77%. A prior
8-second-slew attempt had safely aborted at 0.852 rad/s while returning from +5
degrees; the guard was retained rather than relaxed.

Pitch was then isolated with temporary P=0.8 and 12-second slew. The reachable
+2 and +5 targets failed with 1.714 and 1.926 degrees maximum final error;
return-to-park passed at 0.497 degrees. Servo2 stayed within 1500..1548 us,
maximum pitch rate was 0.452 rad/s, and selected-field change stayed within
-2.49..+2.22%. Negative pitch was not attempted because the installed profile
starts at `TRK_PIT_MIND=0`. HW-002 is therefore **failed**, while HW-003 and
FIELD-001 remain blocked. The board ended STOP/disarmed at 1500/1500 us with
all temporary values restored; yaw P=0.8 remains, while pitch PID/FF remains
zero.

## Fixed deployment profile

Use the exact firmware artifact built with:

```bash
make micoair_h743_antenna_tracker
```

The airframe is `4099_antenna_tracker` and the production mapping is:

| Function | PX4 output | Function ID |
|---|---|---:|
| yaw | MAIN1 / Servo1 | 201 |
| pitch | MAIN2 / Servo2 | 202 |

The operator-confirmed yaw mechanism profile on this installation is:

| Parameter | Value |
|---|---:|
| `TRK_YAW_MIND` | -63 deg |
| `TRK_YAW_MAXD` | +63 deg |
| `TRK_YAW_PARK` | 0 deg |
| `TRK_YAW_REV` | 1 |
| `TRK_YAW_P` | 0.8 |
| `PWM_MAIN_MIN1` | 800 us |
| `PWM_MAIN_MAX1` | 2200 us |
| `PWM_MAIN_DIS1` / `PWM_MAIN_FAIL1` | 1500 us |

The -63..63 limits correspond to the restricted 800..2200 us deployment
window. Only use -90..90 if MAIN1 is deliberately expanded to the servo's
verified 500..2500 us full calibration and all mechanical/cable limits permit
that travel.

The default H743 TELEM1 path is `/dev/ttyS0` at 57600 baud. Confirm the actual
connector and routing on the installed board before using a radio or USB
router. Do not copy the historical V6X AUX1/AUX2 profile to this board.

Before every run, record `git rev-parse HEAD`, the firmware SHA-256, board
serial, airframe, output mapping, PWM min/max/disarmed/failsafe, mechanical
limits, park angles, `SENS_BOARD_ROT`, `TRK_SYSID_TARGET`, and `TRK_TIMEOUT_MS`.

## Gate order

Keep the independent servo-power cutoff accessible throughout all tests.
Servo power remains off until the preceding row has passed.

| Order | Test | Procedure | Pass evidence |
|---:|---|---|---|
| 1 | BENCH-001 | Confirm MAIN1/Servo1 yaw and MAIN2/Servo2 pitch mapping with bounded one-axis commands | parameter dump, PWM and moving-IMU measurement |
| 2 | BENCH-002 | STOP; verify park, bounded one-axis `servo_test`, direction, ±2° then ±5° steps, return-to-park, startup no-jump and timeout park | observed physical angles, PWM record, console, ULog |
| 3 | BENCH-003 | Rotate the complete antenna head by hand with IMU attached; verify `vehicle_attitude` follows yaw/pitch; repeat with servo load and check compass health | angle comparison, `tracker_status`, ULog, video |
| 4 | HW-001 | On H743's selected ingress, send `GLOBAL_POSITION_INT` from `TRK_SYSID_TARGET`; verify `tracker_target_position` updates and a wrong source ID is ignored | link config, console, target topic/ULog |
| 5 | HW-002 | Set low P-only gains, narrow reachable sector, and one static target phase at a time; run north/east/reachable-sector cases, then STOP | target bearing/elevation versus measured head angle; ULog and video |
| 6 | HW-003 | Run a bounded moving target; stop target traffic for more than `TRK_TIMEOUT_MS`; verify park, integrator reset, then recovery with a fresh matching source ID | timeout latency, recovery latency, ULog and video |
| 7 | HW-004 | Power-cycle with servo rail isolated, test invalid attitude/global position, and exercise the independent cutoff | no unsafe motion, safe state reason, cutoff record/video |
| 8 | FIELD-001 | Repeat static, dynamic, loss/recovery and reachable-sector tests outdoors; publish cable-wrap and wind limits | repeated run manifests, ULogs, video, release note |

## H743 bench procedure

1. Set `TRK_MODE=0`, `TRK_FAKE_EN=0`, `TRK_DEADRECK=0`, `TRK_AUTO_SCAN=0`,
   `TRK_ALT_SRC=0`, and a specific `TRK_SYSID_TARGET` for operational tests.
2. Verify `PWM_MAIN_FUNC1=201` and `PWM_MAIN_FUNC2=202`. Set the disarmed and
   failsafe values to the measured physical park positions before powering the
   servos.
3. With the linkage disconnected or clear of stops, use:

   ```text
   antenna_tracker status
   antenna_tracker servo_test start yaw
   antenna_tracker servo_test stop
   antenna_tracker servo_test start pitch
   antenna_tracker servo_test stop
   ```

   Only one axis is swept and the command requires STOP. If direction is
   reversed, correct `TRK_YAW_REV`/`TRK_PIT_REV` or wiring before applying any
   gain change.
4. Set the calibrated home with `antenna_tracker set_home`, then `param save`.
   Keep `TRK_HOME_EN=0` when live global position is the production source.
5. Use `TRK_REF_SETTLE=1.0` s or longer as needed. On every STOP→AUTO
   transition, verify the head is physically parked before the reference is
   captured.
6. For static target phases, use the existing scenario tool. It sends the
   required heartbeat and position traffic but does not change tracker mode:

   ```bash
   python3 Tools/antenna_tracker/run_static_bearing_scenario.py \
     --port <selected-mavlink-ingress> --sysid <uav-sysid> --compid 1 --rate 5 \
     --center-lat <tracker-lat> --center-lon <tracker-lon> \
     --center-alt <tracker-msl-alt> \
     --phase north:<bearing>:<distance>:<alt-offset>:20 \
     --phase east:<bearing>:<distance>:<alt-offset>:20 \
     --timeout-dwell 8 --log <artifact-dir>/hw002-static.jsonl
   ```

   Replace `north`/`east` with reachable bearings from the actual mechanical
   sector. Stop the run immediately if a cable or hard stop is approached.

## Acceptance measurements

Use the measured IMU/head angle, not only normalized `actuator_servos` values.
The initial gate limits are:

- ±2° step: requested direction is correct and settles within 0.5° for 3 s;
- ±5° step: settles within 1.0° in at most 30 s, with no more than 1.0°
  overshoot;
- return-to-park: settles within 0.5°;
- timeout: output reaches the calibrated park command after the configured
  timeout, with target invalid and controller integrators reset;
- recovery: a fresh qualified source restores tracking without a startup jump.

An out-of-limit result is a mechanics/calibration blocker. Do not tune PID to
hide incorrect horn position, endpoint, reversal, board rotation, or compass
interference.

## Evidence handoff

Create one manifest under
`validation/antenna_tracker/runs/gate-7/<TEST-ID>/<run-id>.yaml`. Include the
raw console log, parameter dump, scenario JSONL, ULog and video/photo checksum
where applicable. A verbal “signal received” result should be recorded as a
partial HW-001 observation until the H743 port, source identity and target
topic/ULog are attached. Do not mark HW-002 or FIELD-001 passed from MAVLink
ingress alone.
