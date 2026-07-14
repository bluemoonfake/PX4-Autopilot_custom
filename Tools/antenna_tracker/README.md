# Antenna Tracker Utilities

These utilities belong to the PX4 antenna tracker workflow. Run commands from the repository root unless a command says otherwise.

## PX4-native utilities

| Utility | Purpose |
|---|---|
| `send_target.py` | Send simulated target `HEARTBEAT` and `GLOBAL_POSITION_INT` traffic to tracker SITL or a bench router. |
| `run_static_bearing_scenario.py` | Send timestamped static bearing/elevation phases and write a JSONL audit log for reproducible hardware evidence. |
| `geometry_reference.py` | Independently check geometry cases used by tracker tests. |
| `monitor_tracker.py` | Read generic MAVLink tracker telemetry for bench diagnosis. |
| `usb_router.py` | Forward one Pixhawk USB serial link to QGC and an injected target UDP endpoint. |

Use `--help` before a first run. `send_target.py` emits a 1 Hz non-GCS heartbeat from the same system/component as each position stream because the production bridge requires it. Use `--no-heartbeat` only for negative policy tests. The default destination is `udpout:127.0.0.1:18570`; that is only correct when the target PX4 MAVLink receiver is listening on that port. Record the actual link configuration in the validation evidence manifest.

## Static hardware scenario

`run_static_bearing_scenario.py` is the repeatable sender for `HW-002` static
bearing/elevation and the target-loss portion of `HW-003`. It requires a real,
provisioned tracker home; `(0,0)` is rejected before any MAVLink message is
sent. It records every phase boundary, heartbeat, and position in the required
JSONL audit log. Pair that log with the ULog: the ULog is authoritative for
`tracker_status` and `actuator_servos` response.

The tool deliberately has **no authority to set `TRK_MODE` or PWM output**. An
operator must configure bounded limits and an independent servo-power cutoff,
select AUTO explicitly, then set STOP when the run ends. This prevents an
injection utility from silently becoming a control path.

```bash
python3 Tools/antenna_tracker/run_static_bearing_scenario.py \\
  --port udpout:127.0.0.1:18570 --sysid 2 --compid 1 --rate 5 \\
  --center-lat <TRK_HOME_LAT/1e7> --center-lon <TRK_HOME_LON/1e7> \\
  --center-alt <TRK_HOME_ALT/1000> \\
  --phase north:0:100:0:10 --phase east:90:100:0:10 \\
  --phase south:180:100:0:10 --phase west:270:100:0:10 \\
  --timeout-dwell 5 --log evidence-logs/hw002-static.jsonl
```

Use the calibrated, reachable bearing sector rather than blindly using all four
cardinals. Stop the run immediately if the head approaches a mechanical limit.

## Legacy reference

`legacy/ardupilot/` contains historical ArduPilot materials. They are reference-only and must not be used to operate or validate this PX4 firmware.

## Dependencies

- `pymavlink` for target sending and MAVLink monitoring;
- `pyserial` for the USB router.

Install them in a dedicated development environment; do not treat a utility as firmware test evidence without a corresponding manifest under `validation/antenna_tracker/`.
