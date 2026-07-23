# QGroundControl Compatibility

## Goal

The tracker must work with an unmodified QGroundControl (QGC) build. Compatibility means QGC can identify the vehicle, select the airframe, edit parameters, configure/test outputs, receive normal PX4 telemetry, and display PX4 events. It does **not** mean QGC gains a custom antenna-tracker control page or new flight modes without a QGC extension.

## Vehicle and airframe metadata

The tracker airframe uses:

```text
@name Generic Antenna Tracker
@type Antenna Tracker
@class Rover
MAV_TYPE = 5
```

`MAV_TYPE=5` is MAVLink's Antenna Tracker type. `@class Rover` is presently a QGC airframe-metadata compatibility shim; it does not make the firmware run rover controllers. The dedicated airframe sets `VEHICLE_TYPE=antenna_tracker`, avoiding the standard multicopter, fixed-wing, and rover application startup paths.

PX4 parameter metadata explicitly exposes `MAV_TYPE=5` as **Antenna Tracker**. QGC therefore receives both the correct MAVLink vehicle type and a readable parameter value.

## Stock-QGC operator surface

| Need | Stock-QGC mechanism |
|---|---|
| Choose airframe | Airframe metadata / airframe configuration |
| Configure tracker | Parameters in group **Antenna Tracker** |
| Set output mapping, PWM min/max/reverse | Actuators view |
| Verify yaw/pitch channels | Actuator Test, with mechanical safeguards |
| Inspect health/state | MAVLink Console, PX4 Events, normal telemetry/logging |
| Inspect tracking data | ULog topics, PX4 listener, or tracker monitor utility |

## Tracker mode policy

Tracker `STOP`, `AUTO`, `SCAN`, and `MANUAL` are application submodes, not PX4 vehicle flight modes. The project must not reinterpret unrelated PX4 navigation states as tracker actions in the production path.

`TRK_QGC_MODE` is retained only for compatibility with older parameter files and is disabled by the airframe. It has no production navigation-mode mapping: Commander navigation-state changes cannot select tracker AUTO, SCAN, or MANUAL.

For stock QGC, use `TRK_MODE` and the tracker status/event surface. This is more honest and safer than advertising unsupported custom flight-mode semantics.

## Arming and Ready state

QGC ARM/DISARM continues to use PX4 Commander and the standard MAVLink command;
it does not select `TRK_MODE`. `TRK_MODE` may be configured before arming, but
AUTO, SCAN, or MANUAL becomes effective only after Commander reports the system
armed. Disarming returns the tracker to park without overwriting the requested
mode.

The tracker must not register a custom/external navigation mode merely to avoid
Position, Altitude, Manual, or RC requirements. Instead, Commander readiness
for `MAV_TYPE=5` must retain tracker-relevant sensor, power, safety, and output
checks while treating missing target/position data as a reported runtime park
state. Broad circuit breakers and force-arm are not an acceptable production
solution.

For formal acceptance, record `commander check`, `actuator_armed`,
`vehicle_status`, `health_report`, command acknowledgement, HEARTBEAT armed
state, and the QGC PX4 Events before and after each ARM/DISARM transition. Local
MAVLink/SITL observations exist, but the stock-QGC capture is still pending.
The development contract is documented in [Arming, Readiness, and Tracker
Modes](arming_and_modes.md).

## Actuator presentation

The initial output contract deliberately uses the existing PX4 generic functions:

```text
Servo 1 (function 201) = yaw
Servo 2 (function 202) = pitch
```

The airframe `@output` labels make this visible in airframe metadata. The project will not add new global output-function enum values for “Tracker Yaw” and “Tracker Pitch” until hardware validation shows that generic Servo1/Servo2 causes real operator confusion. Avoiding new global functions keeps upstream merge impact low.

## Parameters

All tracker parameters use the `TRK_` prefix and the **Antenna Tracker** parameter group. Parameters must describe physical behavior precisely:

- angle limits and park positions;
- target source/filter policy;
- timeout and loss policy;
- output calibration/reversal;
- per-axis motion semantics through `TRK_YAW_SRV_T` and `TRK_PIT_SRV_T`;
- only validated prediction or scan behavior.

Do not use servo trim to compensate for incorrect FC orientation. Set `SENS_BOARD_ROT` and perform level-horizon calibration before setting mechanical trim.

`TRK_*_SRV_T` selects `POSITION` versus `CONTINUOUS`; it does not select an
analog/digital servo rate. Configure PWM50/100/200/400 with the native QGC
Actuators output-protocol field (`PWM_MAIN_TIMx`/`PWM_AUX_TIMx`). All channels
in one hardware timer group share that setting, and changing it requires reboot.

## Events and logs

The tracker emits standard PX4 Events on meaningful state edges, including:

- target acquired or lost;
- source rejected by MAVLink admission policy;
- target outside reachable yaw sector;
- entry to scan mode;
- parking outputs due to STOP, timeout, or invalid sensors.

QGC can display PX4 Events without a tracker-specific plugin. `tracker_status`, `tracker_target_position`, and `actuator_servos` are the primary log evidence for diagnostics.

## What requires a QGC fork or plugin

The following are intentionally out of scope for stock-QGC compatibility:

- a dedicated pan/tilt visual control page;
- tracker-specific flight-mode labels in the QGC mode selector;
- graphical sector/cable-wrap editing;
- live tracker plots beyond generic telemetry/log analysis.

Those can be considered only after the stock-QGC firmware workflow is stable and hardware-validated.
