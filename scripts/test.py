#!/usr/bin/env python3
"""
test_mavlink_target_fixed.py

Send a fake UAV target to an ArduPilot AntennaTracker through a UDP injector.
Default setup matches this bench topology:

    Mission Planner <-> mavlink_usb_router_mp_fixed.py <-> Pixhawk tracker USB
    Fake target ----udpout:127.0.0.1:18570----> router ----> Pixhawk tracker

Important:
- --sysid must match tracker parameter SYSID_TARGET.
- --center-lat/--center-lon should be the tracker/base GPS location.
- This script sends HEARTBEAT + GLOBAL_POSITION_INT + GPS_RAW_INT.
"""

import argparse
import math
import sys
import time

try:
    from pymavlink import mavutil
except ImportError:
    print("ERROR: pymavlink not installed. Run: pip install pymavlink")
    sys.exit(1)

EARTH_RADIUS_M = 6371000.0
DEG_TO_E7 = 1e7
M_TO_MM = 1000.0
MPS_TO_CMPS = 100.0


def offset_to_latlon(center_lat_deg: float, center_lon_deg: float, north_m: float, east_m: float):
    center_lat_rad = math.radians(center_lat_deg)
    dlat_deg = north_m / EARTH_RADIUS_M * (180.0 / math.pi)
    dlon_deg = east_m / (EARTH_RADIUS_M * math.cos(center_lat_rad)) * (180.0 / math.pi)
    return center_lat_deg + dlat_deg, center_lon_deg + dlon_deg


def compute_motion(args, elapsed: float):
    """Return north_m, east_m, vn_mps, ve_mps."""
    if args.mode == "circular":
        if args.radius <= 0:
            raise ValueError("radius must be > 0 for circular mode")
        omega = args.speed / args.radius
        angle = omega * elapsed

        # Position: starts North of tracker, then moves clockwise.
        north_m = args.radius * math.cos(angle)
        east_m = args.radius * math.sin(angle)

        # Velocity in NED horizontal axes: derivative of position.
        vn_mps = -args.speed * math.sin(angle)
        ve_mps = args.speed * math.cos(angle)

    elif args.mode == "linear":
        # Fly West -> East through the tracker latitude.
        north_m = 0.0
        east_m = -args.line_half_length + args.speed * elapsed
        vn_mps = 0.0
        ve_mps = args.speed

    else:  # static
        north_m = args.static_north
        east_m = args.static_east
        vn_mps = 0.0
        ve_mps = 0.0

    return north_m, east_m, vn_mps, ve_mps


def heading_cdeg(vn_mps: float, ve_mps: float) -> int:
    # MAVLink GLOBAL_POSITION_INT hdg: centi-degrees, 0 = North, clockwise.
    if abs(vn_mps) < 1e-6 and abs(ve_mps) < 1e-6:
        return 0
    deg = math.degrees(math.atan2(ve_mps, vn_mps))
    if deg < 0:
        deg += 360.0
    return int(deg * 100.0)


def bearing_deg_from_tracker(north_m: float, east_m: float) -> float:
    deg = math.degrees(math.atan2(east_m, north_m))
    if deg < 0:
        deg += 360.0
    return deg


def send_heartbeat(mav):
    mav.mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_QUADROTOR,
        mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
        0,  # base_mode
        0,  # custom_mode
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )


def send_gps_raw_int(mav, lat_e7: int, lon_e7: int, alt_msl_mm: int, vn_mps: float, ve_mps: float, hdg_cdeg: int):
    ground_speed_cmps = int(math.hypot(vn_mps, ve_mps) * MPS_TO_CMPS)
    mav.mav.gps_raw_int_send(
        int(time.time() * 1e6),  # time_usec
        3,                       # fix_type: 3D fix
        lat_e7,
        lon_e7,
        alt_msl_mm,
        100,                     # eph, cm
        100,                     # epv, cm
        ground_speed_cmps,        # vel, cm/s
        hdg_cdeg,                # cog, cdeg
        12,                      # satellites_visible
    )


def send_global_position_int(mav, elapsed: float, lat_e7: int, lon_e7: int,
                             alt_msl_mm: int, rel_alt_mm: int,
                             vn_mps: float, ve_mps: float, hdg_cdeg: int):
    mav.mav.global_position_int_send(
        int(elapsed * 1000) & 0xFFFFFFFF,  # time_boot_ms
        lat_e7,
        lon_e7,
        alt_msl_mm,
        rel_alt_mm,
        int(vn_mps * MPS_TO_CMPS),  # vx: North cm/s
        int(ve_mps * MPS_TO_CMPS),  # vy: East cm/s
        0,                          # vz: Down cm/s
        hdg_cdeg,
    )


def main():
    parser = argparse.ArgumentParser(description="Fake MAVLink UAV target for AntennaTracker")
    parser.add_argument("--port", default="udpout:127.0.0.1:18570",
                        help="MAVLink output connection. Use router inject port. Default: udpout:127.0.0.1:18570")
    parser.add_argument("--sysid", type=int, default=1,
                        help="Fake UAV MAVLink system ID. Must match tracker SYSID_TARGET. Default: 1")
    parser.add_argument("--compid", type=int, default=1, help="Fake UAV component ID. Default: 1")
    parser.add_argument("--rate", type=float, default=5.0, help="GLOBAL_POSITION_INT rate in Hz. Default: 5")
    parser.add_argument("--duration", type=float, default=0.0, help="Seconds to run. 0 = infinite. Default: 0")
    parser.add_argument("--mode", choices=["circular", "linear", "static"], default="circular")
    parser.add_argument("--radius", type=float, default=150.0, help="Circle radius in meters. Default: 150")
    parser.add_argument("--speed", type=float, default=10.0, help="Target horizontal speed in m/s. Default: 10")
    parser.add_argument("--alt", type=float, default=80.0, help="Target altitude above tracker in meters. Default: 80")
    parser.add_argument("--center-lat", type=float, default=10.7776481,
                        help="Tracker/base latitude. Replace with your tracker GPS latitude.")
    parser.add_argument("--center-lon", type=float, default=106.6899469,
                        help="Tracker/base longitude. Replace with your tracker GPS longitude.")
    parser.add_argument("--center-alt", type=float, default=0.0,
                        help="Tracker/base MSL altitude in meters. Yaw test can use 0. Default: 0")
    parser.add_argument("--line-half-length", type=float, default=300.0,
                        help="Linear mode starts this many meters West. Default: 300")
    parser.add_argument("--static-north", type=float, default=0.0,
                        help="Static target North offset in meters. Default: 0")
    parser.add_argument("--static-east", type=float, default=150.0,
                        help="Static target East offset in meters. Default: 150")
    args = parser.parse_args()

    if args.rate <= 0:
        print("ERROR: --rate must be > 0")
        sys.exit(2)

    print("=== Fake MAVLink Target for AntennaTracker ===")
    print(f"Connection : {args.port}")
    print(f"SysID      : {args.sysid}  (must equal tracker SYSID_TARGET)")
    print(f"Mode       : {args.mode}")
    print(f"Center     : {args.center_lat:.7f}, {args.center_lon:.7f}, alt={args.center_alt:.1f}m")
    print(f"Rate       : {args.rate:.1f} Hz")
    print("Duration   : infinite" if args.duration <= 0 else f"Duration   : {args.duration:.1f}s")
    print()

    mav = mavutil.mavlink_connection(
        args.port,
        source_system=args.sysid,
        source_component=args.compid,
        input=False,
        dialect="ardupilotmega",
    )

    period = 1.0 / args.rate
    t_start = time.time()
    last_hb = -999.0
    last_gps = -999.0
    msg_count = 0

    try:
        while True:
            elapsed = time.time() - t_start
            if args.duration > 0 and elapsed >= args.duration:
                print("Duration reached. Stopping.")
                break

            north_m, east_m, vn_mps, ve_mps = compute_motion(args, elapsed)
            lat, lon = offset_to_latlon(args.center_lat, args.center_lon, north_m, east_m)
            lat_e7 = int(lat * DEG_TO_E7)
            lon_e7 = int(lon * DEG_TO_E7)
            alt_msl_mm = int((args.center_alt + args.alt) * M_TO_MM)
            rel_alt_mm = int(args.alt * M_TO_MM)
            hdg = heading_cdeg(vn_mps, ve_mps)

            # HEARTBEAT helps AntennaTracker identify this sysid as a vehicle.
            if elapsed - last_hb >= 1.0:
                send_heartbeat(mav)
                last_hb = elapsed

            # GPS_RAW_INT is not always required, but it makes the fake vehicle look more like a real UAV.
            if elapsed - last_gps >= 0.5:
                send_gps_raw_int(mav, lat_e7, lon_e7, alt_msl_mm, vn_mps, ve_mps, hdg)
                last_gps = elapsed

            send_global_position_int(mav, elapsed, lat_e7, lon_e7, alt_msl_mm, rel_alt_mm, vn_mps, ve_mps, hdg)
            msg_count += 1

            if msg_count % max(1, int(args.rate)) == 0:
                bearing = bearing_deg_from_tracker(north_m, east_m)
                dist = math.hypot(north_m, east_m)
                print(f"[{elapsed:6.1f}s] sysid={args.sysid} lat={lat:.7f} lon={lon:.7f} "
                      f"bearing={bearing:6.1f} deg dist={dist:6.1f}m "
                      f"vel_N/E=({vn_mps:+5.1f},{ve_mps:+5.1f}) m/s")

            time.sleep(period)

    except KeyboardInterrupt:
        print(f"\nStopped. Sent {msg_count} GLOBAL_POSITION_INT messages.")


if __name__ == "__main__":
    main()