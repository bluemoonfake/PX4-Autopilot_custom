# Antenna Tracker Utilities

These utilities belong to the PX4 antenna tracker workflow. Run commands from the repository root unless a command says otherwise.

## PX4-native utilities

| Utility | Purpose |
|---|---|
| `send_target.py` | Send simulated target `HEARTBEAT` and `GLOBAL_POSITION_INT` traffic to tracker SITL or a bench router. |
| `geometry_reference.py` | Independently check geometry cases used by tracker tests. |
| `monitor_tracker.py` | Read generic MAVLink tracker telemetry for bench diagnosis. |
| `usb_router.py` | Forward one Pixhawk USB serial link to QGC and an injected target UDP endpoint. |

Use `--help` before a first run. `send_target.py` emits a 1 Hz non-GCS heartbeat from the same system/component as each position stream because the production bridge requires it. Use `--no-heartbeat` only for negative policy tests. The default destination is `udpout:127.0.0.1:18570`; that is only correct when the target PX4 MAVLink receiver is listening on that port. Record the actual link configuration in the validation evidence manifest.

## Legacy reference

`legacy/ardupilot/` contains historical ArduPilot materials. They are reference-only and must not be used to operate or validate this PX4 firmware.

## Dependencies

- `pymavlink` for target sending and MAVLink monitoring;
- `pyserial` for the USB router.

Install them in a dedicated development environment; do not treat a utility as firmware test evidence without a corresponding manifest under `validation/antenna_tracker/`.