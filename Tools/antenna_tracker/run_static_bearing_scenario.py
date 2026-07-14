#!/usr/bin/env python3
"""Run a reproducible static-target scenario for antenna-tracker bench evidence.

The program sends qualified MAVLink HEARTBEAT and GLOBAL_POSITION_INT traffic
for a sequence of known bearing/elevation targets and writes a JSON-lines audit
log. It has no authority to change tracker parameters or modes: the operator
must put the tracker in STOP before wiring, then deliberately select AUTO for a
run, and return it to STOP after the run. This keeps a test-data sender from
becoming an unreviewed control path.

Example (through usb_router.py's UDP injection port):

  python3 Tools/antenna_tracker/run_static_bearing_scenario.py \\
    --port udpout:127.0.0.1:18570 \\
    --center-lat 10.0000000 --center-lon 106.0000000 --center-alt 10 \\
    --phase north:0:100:0:10 --phase east:90:100:0:10 \\
    --phase south:180:100:0:10 --phase west:270:100:0:10 \\
    --timeout-dwell 5 --log evidence-logs/hw002-static.jsonl

Each --phase is NAME:BEARING_DEG:DISTANCE_M:ALTITUDE_OFFSET_M:DURATION_S.
The optional final timeout dwell sends no heartbeat or position messages, which
lets the configured TRK_TIMEOUT_MS expiry be observed in the ULog.
"""

import argparse
import json
import math
import sys
import time
from pathlib import Path

try:
    from pymavlink import mavutil
except ImportError:
    print("ERROR: pymavlink not installed. Run: pip install pymavlink", file=sys.stderr)
    sys.exit(1)


EARTH_RADIUS_M = 6371000.0


def parse_phase(value):
    fields = value.split(":")
    if len(fields) != 5 or not fields[0]:
        raise argparse.ArgumentTypeError(
            "phase must be NAME:BEARING_DEG:DISTANCE_M:ALTITUDE_OFFSET_M:DURATION_S")

    try:
        name = fields[0]
        bearing_deg, distance_m, altitude_offset_m, duration_s = map(float, fields[1:])
    except ValueError as error:
        raise argparse.ArgumentTypeError(f"invalid phase '{value}': {error}") from error

    if not 0.0 <= bearing_deg < 360.0:
        raise argparse.ArgumentTypeError("phase bearing must be in [0, 360)")
    if distance_m <= 0.0 or duration_s <= 0.0:
        raise argparse.ArgumentTypeError("phase distance and duration must be positive")

    return {
        "name": name,
        "bearing_deg": bearing_deg,
        "distance_m": distance_m,
        "altitude_offset_m": altitude_offset_m,
        "duration_s": duration_s,
    }


class AuditLog:
    def __init__(self, path):
        self._file = path.open("w", encoding="utf-8")

    def write(self, event, **fields):
        record = {
            "event": event,
            "utc": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "monotonic_s": round(time.monotonic(), 6),
            **fields,
        }
        self._file.write(json.dumps(record, sort_keys=True) + "\n")
        self._file.flush()

    def close(self):
        self._file.close()


def target_position(center_lat, center_lon, center_alt, phase):
    bearing_rad = math.radians(phase["bearing_deg"])
    north_m = phase["distance_m"] * math.cos(bearing_rad)
    east_m = phase["distance_m"] * math.sin(bearing_rad)
    latitude = center_lat + math.degrees(north_m / EARTH_RADIUS_M)
    longitude = center_lon + math.degrees(east_m / (EARTH_RADIUS_M * math.cos(math.radians(center_lat))))
    return {
        "lat_e7": int(round(latitude * 1e7)),
        "lon_e7": int(round(longitude * 1e7)),
        "alt_mm": int(round((center_alt + phase["altitude_offset_m"]) * 1000.0)),
        "relative_alt_mm": int(round(phase["altitude_offset_m"] * 1000.0)),
        "north_m": north_m,
        "east_m": east_m,
    }


def send_heartbeat(mav):
    mav.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_QUADROTOR,
        mavutil.mavlink.MAV_AUTOPILOT_PX4,
        0,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )


def send_position(mav, target, run_elapsed_s):
    mav.mav.global_position_int_send(
        int(run_elapsed_s * 1000.0),
        target["lat_e7"],
        target["lon_e7"],
        target["alt_mm"],
        target["relative_alt_mm"],
        0,
        0,
        0,
        0,
    )


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--port", default="udpout:127.0.0.1:18570",
                        help="MAVLink output connection (default: udpout:127.0.0.1:18570)")
    parser.add_argument("--sysid", type=int, default=2, help="simulated vehicle system ID (default: 2)")
    parser.add_argument("--compid", type=int, default=1, help="simulated vehicle component ID (default: 1)")
    parser.add_argument("--rate", type=float, default=5.0, help="position rate in Hz (default: 5)")
    parser.add_argument("--heartbeat-rate", type=float, default=1.0, help="heartbeat rate in Hz (default: 1)")
    parser.add_argument("--center-lat", type=float, required=True, help="provisioned tracker latitude, degrees")
    parser.add_argument("--center-lon", type=float, required=True, help="provisioned tracker longitude, degrees")
    parser.add_argument("--center-alt", type=float, required=True, help="provisioned tracker MSL altitude, metres")
    parser.add_argument("--phase", type=parse_phase, action="append", required=True,
                        help="NAME:BEARING_DEG:DISTANCE_M:ALTITUDE_OFFSET_M:DURATION_S; repeat as needed")
    parser.add_argument("--timeout-dwell", type=float, default=0.0,
                        help="seconds of target silence after the final phase (default: 0)")
    parser.add_argument("--log", type=Path, required=True, help="required JSONL audit-log path")
    args = parser.parse_args()

    if not -90.0 <= args.center_lat <= 90.0 or not -180.0 <= args.center_lon <= 180.0:
        parser.error("center latitude/longitude out of range")
    if args.center_lat == 0.0 and args.center_lon == 0.0:
        parser.error("center (0,0) is a placeholder, not a provisioned tracker home")
    if not 1 <= args.sysid <= 255 or not 1 <= args.compid <= 255:
        parser.error("sysid and compid must be in 1..255")
    if args.rate <= 0.0 or args.heartbeat_rate <= 0.0:
        parser.error("rate and heartbeat-rate must be positive")
    if args.timeout_dwell < 0.0:
        parser.error("timeout-dwell must not be negative")

    args.log.parent.mkdir(parents=True, exist_ok=True)
    audit = AuditLog(args.log)
    mav = mavutil.mavlink_connection(args.port, source_system=args.sysid,
                                     source_component=args.compid, input=False)
    started = time.monotonic()
    sent_positions = 0
    sent_heartbeats = 0
    outcome = "completed"

    audit.write("run_start", port=args.port, sysid=args.sysid, compid=args.compid,
                rate_hz=args.rate, heartbeat_rate_hz=args.heartbeat_rate,
                center_lat=args.center_lat, center_lon=args.center_lon, center_alt_m=args.center_alt,
                phases=args.phase, timeout_dwell_s=args.timeout_dwell)
    print(f"Scenario source {args.sysid}/{args.compid} -> {args.port}; audit log: {args.log}")
    print("Safety: verify the independent servo-power cutoff; this utility does not set TRK_MODE.")

    try:
        position_period = 1.0 / args.rate
        heartbeat_period = 1.0 / args.heartbeat_rate
        next_heartbeat = started

        for phase in args.phase:
            target = target_position(args.center_lat, args.center_lon, args.center_alt, phase)
            phase_start = time.monotonic()
            phase_end = phase_start + phase["duration_s"]
            next_position = phase_start
            audit.write("phase_start", phase=phase, target=target,
                        run_elapsed_s=round(phase_start - started, 3))
            print(f"START {phase['name']}: bearing={phase['bearing_deg']:.1f} deg "
                  f"distance={phase['distance_m']:.1f} m alt_offset={phase['altitude_offset_m']:.1f} m")

            while time.monotonic() < phase_end:
                now = time.monotonic()
                elapsed = now - started
                if now >= next_heartbeat:
                    send_heartbeat(mav)
                    sent_heartbeats += 1
                    audit.write("heartbeat", phase=phase["name"], run_elapsed_s=round(elapsed, 3))
                    next_heartbeat += heartbeat_period
                if now >= next_position:
                    send_position(mav, target, elapsed)
                    sent_positions += 1
                    audit.write("position", phase=phase["name"], run_elapsed_s=round(elapsed, 3), target=target)
                    next_position += position_period
                time.sleep(min(0.01, max(0.0, min(next_heartbeat, next_position) - time.monotonic())))

            audit.write("phase_end", phase=phase["name"], run_elapsed_s=round(time.monotonic() - started, 3))
            print(f"END   {phase['name']}")

        if args.timeout_dwell > 0.0:
            audit.write("target_silence_start", duration_s=args.timeout_dwell,
                        run_elapsed_s=round(time.monotonic() - started, 3))
            print(f"TARGET SILENCE: {args.timeout_dwell:.1f}s (observe timeout/park in ULog)")
            time.sleep(args.timeout_dwell)
            audit.write("target_silence_end", run_elapsed_s=round(time.monotonic() - started, 3))

    except KeyboardInterrupt:
        outcome = "interrupted"
        print("Interrupted; no more target traffic will be sent.")
    except Exception as error:
        outcome = "error"
        audit.write("error", message=str(error))
        raise
    finally:
        audit.write("run_end", outcome=outcome, duration_s=round(time.monotonic() - started, 3),
                    positions_sent=sent_positions, heartbeats_sent=sent_heartbeats,
                    operator_action="Set TRK_MODE=STOP and preserve the ULog before removing servo power.")
        audit.close()

    print(f"{outcome}: {sent_positions} positions, {sent_heartbeats} heartbeats")
    print("Set TRK_MODE=STOP manually, save/download the ULog, and attach this JSONL file to evidence.")


if __name__ == "__main__":
    main()
