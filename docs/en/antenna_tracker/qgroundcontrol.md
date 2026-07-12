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

The firmware roadmap includes adding the missing human-readable Antenna Tracker value to PX4's `MAV_TYPE` parameter metadata. Until then, some QGC views may render the numeric type as unknown even though MAVLink type 5 is correct.

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

The current `TRK_QGC_MODE` compatibility behavior maps PX4 Position, Altitude, and Manual navigation states to tracker modes. This is fragile: Commander may change navigation state as part of health or estimator fallback, causing unrequested tracker behavior. The roadmap therefore deprecates this mapping and makes direct tracker mode selection the default.

For stock QGC, use `TRK_MODE` and the tracker status/event surface. This is more honest and safer than advertising unsupported custom flight-mode semantics.

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
- only validated prediction or scan behavior.

Do not use servo trim to compensate for incorrect FC orientation. Set `SENS_BOARD_ROT` and perform level-horizon calibration before setting mechanical trim.

## Events and logs

Future firmware phases must emit standard PX4 Events on meaningful state edges, including:

- target acquired or lost;
- source rejected or invalid;
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