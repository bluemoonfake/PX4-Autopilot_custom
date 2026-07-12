# ArduPilot Comparison Reference

This page uses ArduPilot AntennaTracker as a feature reference only. The PX4 tracker remains a PX4-native airframe/module and must follow PX4 uORB, output-function, parameter, event, and logging conventions.

## Already present in the PX4 fork

- target position ingest through MAVLink `GLOBAL_POSITION_INT`;
- dedicated target/status uORB messages;
- bearing, elevation, distance, and angle wrapping;
- STOP, AUTO, SCAN, MANUAL, and servo-test paths;
- target timeout and source system-ID selection/auto-lock;
- generic Servo1/Servo2 output functions;
- FMUv6C tracker build target and airframe 4099;
- optional tracker logging topics.

Presence in source is not the same as hardware validation. Refer to the [verification matrix](../../../../validation/antenna_tracker/test_matrix.yaml) for the required evidence level.

## Gaps that matter before a stable positional-servo tracker

| Gap | Why it matters | Roadmap response |
|---|---|---|
| Position-servo command model | Error-to-position output can collapse toward neutral at zero error | Angle-domain planner, mapper, and stateful axis command |
| Reachable yaw sector | Shortest wrapped heading can cross a stop or cable limit | Explicit yaw min/max/park and sector planner |
| Safe park semantics | Normalized zero is not always mechanically safe | Per-axis park angles and calibrated output mapping |
| Source trust | System-ID lock selects messages but does not authenticate them | Source/component/heartbeat policy, optional signing design |
| Altitude reference | Target relative altitude may use a different home datum | Use MSL until a shared reference is explicit |
| Hardware feedback | Fixed FC attitude is not antenna attitude | Moving IMU or external encoder architecture |
| Observability | Raw status fields are insufficient for reproducible tuning | Enriched tracker uORB status, Events, and evidence manifests |

## Deliberately deferred features

Do not add these before the core positional-servo architecture and safety gates pass:

- continuous-rotation or on/off actuator backends;
- yaw reversal strategies for multi-turn/limited mechanics;
- advanced PID filters/notches;
- custom QGC tracker pages or flight modes;
- ONVIF/camera integration;
- dead reckoning as a production default.

The historical ArduPilot parameter export is stored under `Tools/antenna_tracker/legacy/ardupilot/` for comparison only. It must never be applied to PX4.