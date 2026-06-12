#!/usr/bin/env python3
"""
test_mavlink_target.py — Send fake GLOBAL_POSITION_INT to test antenna tracker.

Simulates a UAV flying in a circle (or straight line) around the tracker,
sending GLOBAL_POSITION_INT messages via MAVLink UDP.

Usage:
    # Circular orbit around SITL default position (47.3977°N, 8.5456°E)
    python3 scripts/test_mavlink_target.py --mode circular --radius 200 --alt 100

    # Linear flyover (West to East)
    python3 scripts/test_mavlink_target.py --mode linear --alt 80

    # Specific sysid for sysid filter test
    python3 scripts/test_mavlink_target.py --sysid 2 --mode circular

    # Short burst then stop (for timeout test)
    python3 scripts/test_mavlink_target.py --duration 10 --mode circular

Requirements:
    pip install pymavlink
"""

import argparse
import math
import time
import sys

try:
    from pymavlink import mavutil
except ImportError:
    print("ERROR: pymavlink not installed. Run: pip install pymavlink")
    sys.exit(1)


def main():
    parser = argparse.ArgumentParser(description="Antenna Tracker MAVLink target simulator")
    parser.add_argument("--port", type=str, default="udp:127.0.0.1:14550",
                        help="MAVLink connection string (default: udp:127.0.0.1:14550)")
    parser.add_argument("--sysid", type=int, default=2,
                        help="MAVLink system ID of the simulated UAV (default: 2)")
    parser.add_argument("--compid", type=int, default=1,
                        help="MAVLink component ID (default: 1)")
    parser.add_argument("--rate", type=float, default=5.0,
                        help="Message rate in Hz (default: 5)")
    parser.add_argument("--duration", type=float, default=60.0,
                        help="Duration in seconds (default: 60, 0=infinite)")
    parser.add_argument("--mode", type=str, default="circular",
                        choices=["circular", "linear", "static"],
                        help="Flight pattern (default: circular)")
    parser.add_argument("--radius", type=float, default=200.0,
                        help="Circle radius in meters (default: 200)")
    parser.add_argument("--alt", type=float, default=100.0,
                        help="Altitude above tracker in meters (default: 100)")
    parser.add_argument("--speed", type=float, default=15.0,
                        help="Ground speed in m/s for circular/linear (default: 15)")
    parser.add_argument("--center-lat", type=float, default=47.397742,
                        help="Tracker center latitude (default: SITL 47.397742)")
    parser.add_argument("--center-lon", type=float, default=8.545594,
                        help="Tracker center longitude (default: SITL 8.545594)")
    parser.add_argument("--center-alt", type=float, default=488.0,
                        help="Tracker altitude MSL in meters (default: SITL 488)")
    args = parser.parse_args()

    print(f"=== Antenna Tracker MAVLink Target Simulator ===")
    print(f"  Connection:  {args.port}")
    print(f"  SysID:       {args.sysid}")
    print(f"  Mode:        {args.mode}")
    print(f"  Rate:        {args.rate} Hz")
    print(f"  Duration:    {args.duration}s" if args.duration > 0 else "  Duration:    infinite")
    print(f"  Radius:      {args.radius}m" if args.mode == "circular" else "")
    print(f"  Alt above:   {args.alt}m")
    print(f"  Center:      {args.center_lat:.6f}, {args.center_lon:.6f}")
    print()

    # Connect
    mav = mavutil.mavlink_connection(
        args.port,
        source_system=args.sysid,
        source_component=args.compid,
        input=False  # output-only
    )

    # Constants
    DEG_TO_E7 = 1e7
    M_TO_MM = 1000.0
    MPS_TO_CMPS = 100.0
    EARTH_RADIUS = 6371000.0

    center_lat_rad = math.radians(args.center_lat)
    target_alt_msl_mm = int((args.center_alt + args.alt) * M_TO_MM)
    relative_alt_mm = int(args.alt * M_TO_MM)

    dt = 1.0 / args.rate
    t_start = time.time()
    t = 0.0
    msg_count = 0

    print("Sending GLOBAL_POSITION_INT messages... (Ctrl+C to stop)")

    try:
        while True:
            elapsed = time.time() - t_start

            if args.duration > 0 and elapsed > args.duration:
                print(f"\nDuration reached ({args.duration}s). Stopping.")
                break

            # Compute target position based on mode
            if args.mode == "circular":
                # Angular velocity: v = omega * r → omega = v/r
                omega = args.speed / args.radius
                angle = omega * elapsed  # radians

                # Offset in meters from center
                dx = args.radius * math.sin(angle)   # East
                dy = args.radius * math.cos(angle)   # North

                # Velocity components (tangent to circle)
                vx_mps = args.speed * math.cos(angle)   # North velocity
                vy_mps = args.speed * math.sin(angle)    # East velocity → actually we want: v_north, v_east
                # For circular: v_north = -omega*r*sin(angle), v_east = omega*r*cos(angle)
                vx_mps = -args.speed * math.sin(angle)   # North
                vy_mps = args.speed * math.cos(angle)     # East

            elif args.mode == "linear":
                # Fly West to East
                dx = -500.0 + args.speed * elapsed  # Start 500m West
                dy = 0.0
                vx_mps = 0.0
                vy_mps = args.speed

            else:  # static
                dx = args.radius  # Static point East of center
                dy = 0.0
                vx_mps = 0.0
                vy_mps = 0.0

            # Convert offset to lat/lon
            dlat = dy / EARTH_RADIUS * (180.0 / math.pi)
            dlon = dx / (EARTH_RADIUS * math.cos(center_lat_rad)) * (180.0 / math.pi)

            lat_e7 = int((args.center_lat + dlat) * DEG_TO_E7)
            lon_e7 = int((args.center_lon + dlon) * DEG_TO_E7)

            # Heading (deg * 100, 0=North CW)
            heading_deg = math.degrees(math.atan2(vy_mps, vx_mps))
            if heading_deg < 0:
                heading_deg += 360.0
            hdg_cdeg = int(heading_deg * 100)

            # Send GLOBAL_POSITION_INT
            mav.mav.global_position_int_send(
                int(elapsed * 1000),          # time_boot_ms
                lat_e7,                        # lat (degE7)
                lon_e7,                        # lon (degE7)
                target_alt_msl_mm,             # alt (mm MSL)
                relative_alt_mm,               # relative_alt (mm)
                int(vx_mps * MPS_TO_CMPS),     # vx (cm/s, NED North)
                int(vy_mps * MPS_TO_CMPS),     # vy (cm/s, NED East)
                0,                             # vz (cm/s, NED Down)
                hdg_cdeg                       # hdg (cdeg)
            )

            msg_count += 1
            bearing_deg = math.degrees(math.atan2(dx, dy))
            if bearing_deg < 0:
                bearing_deg += 360.0
            dist = math.sqrt(dx * dx + dy * dy)

            if msg_count % int(args.rate) == 0:  # Print every second
                print(f"  [{elapsed:6.1f}s] msg #{msg_count:4d} | "
                      f"lat={lat_e7} lon={lon_e7} | "
                      f"bearing={bearing_deg:5.1f}° dist={dist:6.1f}m | "
                      f"v=({vx_mps:+5.1f}, {vy_mps:+5.1f}) m/s")

            time.sleep(dt)

    except KeyboardInterrupt:
        print(f"\nStopped. Sent {msg_count} messages in {time.time() - t_start:.1f}s")


if __name__ == "__main__":
    main()
