#!/usr/bin/env python3
"""
test_geometry.py — Verify antenna tracker geometry calculations.

Computes expected bearing, pitch, and distance for given tracker and target
positions, so you can compare with SITL tracker_status output.

Usage:
    # Default: SITL position with target ~100m North, 50m above
    python3 scripts/test_geometry.py

    # Custom positions
    python3 scripts/test_geometry.py \
        --tracker-lat 47.397742 --tracker-lon 8.545594 --tracker-alt 488 \
        --target-lat 47.398642 --target-lon 8.545594 --target-alt 538

    # From degE7 integers (as used in PX4 params)
    python3 scripts/test_geometry.py \
        --tracker-lat-e7 473977420 --tracker-lon-e7 85455940 --tracker-alt 488 \
        --target-lat-e7 473986420 --target-lon-e7 85455940 --target-alt-mm 538000
"""

import argparse
import math


def longitude_scale(lat_e7: int) -> float:
    """Cosine of latitude for longitude scaling."""
    return math.cos(lat_e7 * 1e-7 * math.pi / 180.0)


def horizontal_distance_m(tracker_lat_e7: int, tracker_lon_e7: int,
                           target_lat_e7: int, target_lon_e7: int) -> float:
    """Flat-earth horizontal distance in meters."""
    METERS_PER_DEGE7 = 0.0111319  # ~111319m per degree / 1e7

    dlat = float(target_lat_e7 - tracker_lat_e7)
    dlon = float(target_lon_e7 - tracker_lon_e7) * longitude_scale(tracker_lat_e7)

    north_m = dlat * METERS_PER_DEGE7
    east_m = dlon * METERS_PER_DEGE7

    return math.sqrt(north_m * north_m + east_m * east_m)


def bearing_rad(tracker_lat_e7: int, tracker_lon_e7: int,
                target_lat_e7: int, target_lon_e7: int) -> float:
    """Compass bearing from tracker to target (0=North, CW)."""
    METERS_PER_DEGE7 = 0.0111319

    dlat = float(target_lat_e7 - tracker_lat_e7)
    dlon = float(target_lon_e7 - tracker_lon_e7) * longitude_scale(tracker_lat_e7)

    north_m = dlat * METERS_PER_DEGE7
    east_m = dlon * METERS_PER_DEGE7

    b = math.atan2(east_m, north_m)
    if b < 0:
        b += 2.0 * math.pi
    return b


def pitch_rad(delta_alt_m: float, horizontal_dist_m: float) -> float:
    """Elevation angle from tracker to target."""
    if horizontal_dist_m < 0.1:
        if delta_alt_m > 0:
            return math.pi / 2.0
        elif delta_alt_m < 0:
            return -math.pi / 2.0
        else:
            return 0.0
    return math.atan2(delta_alt_m, horizontal_dist_m)


def run_test_case(name: str,
                  tracker_lat_e7: int, tracker_lon_e7: int, tracker_alt_m: float,
                  target_lat_e7: int, target_lon_e7: int, target_alt_m: float):
    """Run a single geometry test case and print results."""
    dist = horizontal_distance_m(tracker_lat_e7, tracker_lon_e7,
                                  target_lat_e7, target_lon_e7)
    bear = bearing_rad(tracker_lat_e7, tracker_lon_e7,
                       target_lat_e7, target_lon_e7)
    delta_alt = target_alt_m - tracker_alt_m
    pit = pitch_rad(delta_alt, dist)

    print(f"  [{name}]")
    print(f"    Tracker:  lat={tracker_lat_e7/1e7:.7f}° lon={tracker_lon_e7/1e7:.7f}° alt={tracker_alt_m:.1f}m")
    print(f"    Target:   lat={target_lat_e7/1e7:.7f}° lon={target_lon_e7/1e7:.7f}° alt={target_alt_m:.1f}m")
    print(f"    Distance: {dist:.2f} m")
    print(f"    Bearing:  {math.degrees(bear):.2f}° ({bear:.5f} rad)")
    print(f"    Pitch:    {math.degrees(pit):.2f}° ({pit:.5f} rad)")
    print(f"    ΔAlt:     {delta_alt:.1f} m")
    print()


def main():
    parser = argparse.ArgumentParser(description="Antenna Tracker geometry verifier")
    parser.add_argument("--tracker-lat", type=float, default=None, help="Tracker lat in degrees")
    parser.add_argument("--tracker-lon", type=float, default=None, help="Tracker lon in degrees")
    parser.add_argument("--tracker-lat-e7", type=int, default=None, help="Tracker lat in degE7")
    parser.add_argument("--tracker-lon-e7", type=int, default=None, help="Tracker lon in degE7")
    parser.add_argument("--tracker-alt", type=float, default=488.0, help="Tracker alt MSL (m)")
    parser.add_argument("--target-lat", type=float, default=None, help="Target lat in degrees")
    parser.add_argument("--target-lon", type=float, default=None, help="Target lon in degrees")
    parser.add_argument("--target-lat-e7", type=int, default=None, help="Target lat in degE7")
    parser.add_argument("--target-lon-e7", type=int, default=None, help="Target lon in degE7")
    parser.add_argument("--target-alt", type=float, default=None, help="Target alt MSL (m)")
    parser.add_argument("--target-alt-mm", type=int, default=None, help="Target alt MSL (mm)")
    args = parser.parse_args()

    print("=" * 60)
    print("  Antenna Tracker Geometry Verification")
    print("=" * 60)
    print()

    # Default SITL position
    default_tracker_lat_e7 = 473977420
    default_tracker_lon_e7 = 85455940

    # Handle user input
    if args.tracker_lat_e7 is not None:
        tracker_lat_e7 = args.tracker_lat_e7
    elif args.tracker_lat is not None:
        tracker_lat_e7 = int(args.tracker_lat * 1e7)
    else:
        tracker_lat_e7 = default_tracker_lat_e7

    if args.tracker_lon_e7 is not None:
        tracker_lon_e7 = args.tracker_lon_e7
    elif args.tracker_lon is not None:
        tracker_lon_e7 = int(args.tracker_lon * 1e7)
    else:
        tracker_lon_e7 = default_tracker_lon_e7

    tracker_alt = args.tracker_alt

    if args.target_lat is not None or args.target_lat_e7 is not None:
        # Custom single target
        if args.target_lat_e7 is not None:
            target_lat_e7 = args.target_lat_e7
        else:
            target_lat_e7 = int(args.target_lat * 1e7)

        if args.target_lon_e7 is not None:
            target_lon_e7 = args.target_lon_e7
        elif args.target_lon is not None:
            target_lon_e7 = int(args.target_lon * 1e7)
        else:
            target_lon_e7 = tracker_lon_e7

        if args.target_alt_mm is not None:
            target_alt = args.target_alt_mm / 1000.0
        elif args.target_alt is not None:
            target_alt = args.target_alt
        else:
            target_alt = tracker_alt + 100

        run_test_case("Custom", tracker_lat_e7, tracker_lon_e7, tracker_alt,
                      target_lat_e7, target_lon_e7, target_alt)
    else:
        # Standard 4-direction test suite
        print("  Standard 4-direction test suite (same as SITL test)")
        print()

        # Target alt: 538m MSL (50m above tracker at 488m)
        target_alt = 538.0

        # North: +9000 degE7 ≈ 100m
        run_test_case("NORTH (+100m N, +50m up)",
                      tracker_lat_e7, tracker_lon_e7, tracker_alt,
                      tracker_lat_e7 + 9000, tracker_lon_e7, target_alt)

        # East: +10000 degE7 lon
        run_test_case("EAST (+10000 dE7 E, +50m up)",
                      tracker_lat_e7, tracker_lon_e7, tracker_alt,
                      tracker_lat_e7, tracker_lon_e7 + 10000, target_alt)

        # South: -9000 degE7
        run_test_case("SOUTH (-9000 dE7 S, +50m up)",
                      tracker_lat_e7, tracker_lon_e7, tracker_alt,
                      tracker_lat_e7 - 9000, tracker_lon_e7, target_alt)

        # West: -10000 degE7 lon
        run_test_case("WEST (-10000 dE7 W, +50m up)",
                      tracker_lat_e7, tracker_lon_e7, tracker_alt,
                      tracker_lat_e7, tracker_lon_e7 - 10000, target_alt)

        # Additional: target directly above
        run_test_case("ABOVE (directly above, +100m up)",
                      tracker_lat_e7, tracker_lon_e7, tracker_alt,
                      tracker_lat_e7, tracker_lon_e7, tracker_alt + 100)

        # Additional: target same altitude, far away
        run_test_case("FAR EAST (1km East, same alt)",
                      tracker_lat_e7, tracker_lon_e7, tracker_alt,
                      tracker_lat_e7, tracker_lon_e7 + 130000, tracker_alt)


if __name__ == "__main__":
    main()
