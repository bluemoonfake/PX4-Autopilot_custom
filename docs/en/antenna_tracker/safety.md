# Safety

::: warning
`COM_PREARM_MODE=2` is used so the tracker can drive non-throttling servo outputs while not armed. Treat every boot and parameter change as a potential actuator-motion event.
:::

## ARM permission versus servo power

The implemented ARM gate prevents AUTO, SCAN, and MANUAL tracker behavior while
disarmed. It does not promise zero physical motion before ARM: with
`COM_PREARM_MODE=2`, the output pipeline may move or hold the mechanism at its
calibrated park pose.

On disarm, kill, lockdown, or termination, active mode dispatch must stop, both
controllers must reset, and the configured safe command must be selected within
one 50 Hz cycle. Continue to treat the independent servo-power cutoff as the
final safety boundary.

## Required physical safeguards

Before any powered servo test:

- use a hardware-accessible servo-power cutoff or emergency stop;
- set a current limit appropriate for the servo supply;
- remove RF payload/load or decouple the linkage for initial tests;
- establish clearance from stops, people, cables, and the tripod;
- verify cable routing through the complete permitted yaw range;
- do not rely on PX4 disarmed state alone to prevent movement.

## Safe-state model

The implementation defines per-axis physical park angles and maps them through calibrated servo outputs. Normalized zero is never used as an assumed safe command:

| Condition | Required behavior |
|---|---|
| boot/startup delay | remain at configured park pose |
| STOP | move or hold at configured park pose |
| target timeout | park by default; scan only when explicitly configured and mechanically validated |
| invalid attitude/global position | park and report the condition |
| target too close/unreachable | park according to the configured physical park pose |
| PWM failsafe | output the calibrated yaw/pitch park PWM values |
| emergency actuator cutoff | remove servo rail power independently of firmware |

## Mechanical limits and cable wrap

A target bearing may have more than one mathematically equivalent yaw representation. The controller must choose only an angle inside the actual reachable sector. It must not wrap through a stop or cable limit just to minimize angular distance.

Before AUTO or SCAN is used on hardware, document:

- yaw minimum and maximum physical angles;
- pitch minimum and maximum physical angles;
- yaw/pitch park angles;
- cable-wrap limit and allowed turns;
- output PWM min/max/reverse;
- output PWM disarmed and failsafe values, both matching the calibrated park pose;
- corresponding normalized servo range.

## Compass and feedback validation

Compass/heading error points the antenna at the wrong azimuth even when GPS and geometry are correct. Validate yaw in the final mechanical configuration, including servo power, current load, motors, brackets, and cable routing. Repeat calibration or add mitigation when current-dependent heading errors appear.

## Target data safety

- Set `TRK_SYSID_TARGET` to the target UAV system ID before operational use; a value of zero disables MAVLink target acceptance.
- Verify that only `GLOBAL_POSITION_INT` from that system ID updates `tracker_target_position`.
- Treat system ID filtering as routing, not identity proof.
- Use absolute MSL altitude until a common relative-altitude reference is proven.
- Keep dead reckoning disabled until target velocity validity and prediction error are verified.
- Test target loss, recovery, invalid positions, and wrong source IDs before a field test.

## Test escalation

Advance only in this order:

1. output mapping with no mechanical load;
2. one-axis motion and park behavior;
3. moving-IMU orientation verification;
4. static target at bounded output/gain;
5. dynamic bench target;
6. outdoor static target;
7. outdoor dynamic target and link-loss tests.

Every transition requires a recorded evidence manifest defined in [verification](verification.md).
