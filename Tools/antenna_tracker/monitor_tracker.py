#!/usr/bin/env python3
"""Monitor generic MAVLink telemetry from a PX4 antenna tracker.

This utility observes MAVLink messages that a stock PX4/QGC link normally
exposes: GLOBAL_POSITION_INT, ATTITUDE, and SERVO_OUTPUT_RAW. It cannot read
tracker_status or tracker_target_position directly because those are custom
uORB topics, not standard MAVLink messages. Use PX4 `listener` or ULog for
those topics.

Examples from the PX4-Autopilot repository root:
    python3 Tools/antenna_tracker/monitor_tracker.py --connection udpin:0.0.0.0:14550
    python3 Tools/antenna_tracker/monitor_tracker.py --connection /dev/ttyACM0 --baud 921600

Requirements:
    pip install pymavlink
"""

import argparse
import math
import time

from pymavlink import mavutil


def format_position(message):
    return (
        f"[TRACKER POS] lat={message.lat / 1e7:.7f} "
        f"lon={message.lon / 1e7:.7f} alt={message.alt / 1000.0:.2f} m"
    )


def format_attitude(message):
    yaw_deg = math.degrees(message.yaw) % 360.0
    return (
        f"[TRACKER ATT] roll={math.degrees(message.roll):.1f} "
        f"pitch={math.degrees(message.pitch):.1f} yaw={yaw_deg:.1f} deg"
    )


def format_servos(message):
    return (
        f"[TRACKER SERVO] s1={message.servo1_raw} s2={message.servo2_raw} "
        f"s3={message.servo3_raw} s4={message.servo4_raw}"
    )


def main():
    parser = argparse.ArgumentParser(description="Monitor generic MAVLink telemetry from a PX4 antenna tracker")
    parser.add_argument("--connection", default="udpin:0.0.0.0:14550", help="pymavlink connection string")
    parser.add_argument("--baud", type=int, help="baud rate for serial connections")
    parser.add_argument("--sysid", type=int, help="only display this MAVLink system ID")
    parser.add_argument("--duration", type=float, default=0.0, help="seconds to monitor; 0 means until Ctrl-C")
    args = parser.parse_args()

    connection = mavutil.mavlink_connection(args.connection, baud=args.baud)
    print(f"Monitoring {args.connection}" + (f" for sysid {args.sysid}" if args.sysid is not None else ""))
    print("Use PX4 listener/ULog for tracker_status and tracker_target_position.")

    started = time.monotonic()
    message_types = ["GLOBAL_POSITION_INT", "ATTITUDE", "SERVO_OUTPUT_RAW"]

    try:
        while args.duration <= 0.0 or time.monotonic() - started < args.duration:
            message = connection.recv_match(type=message_types, blocking=True, timeout=1.0)

            if message is None:
                continue

            if args.sysid is not None and message.get_srcSystem() != args.sysid:
                continue

            if message.get_type() == "GLOBAL_POSITION_INT":
                print(format_position(message))
            elif message.get_type() == "ATTITUDE":
                print(format_attitude(message))
            elif message.get_type() == "SERVO_OUTPUT_RAW":
                print(format_servos(message))
    except KeyboardInterrupt:
        pass
    finally:
        connection.close()


if __name__ == "__main__":
    main()
