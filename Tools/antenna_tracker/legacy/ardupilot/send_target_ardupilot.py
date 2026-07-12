#!/usr/bin/env python3
"""
test_mavlink_target.py

Fake MAVLink UAV target for ArduPilot AntennaTracker.

Servo mapping:
    AUX2 = SERVO10 = Yaw
    AUX3 = SERVO11 = Pitch

Important features:
    --yaw-offset -60 / -30 / 30 / 60

The script will:
    1. Read tracker GLOBAL_POSITION_INT
    2. Wait tracker AUTO + ARMED
    3. Drain old MAVLink messages
    4. Read fresh ATTITUDE yaw
    5. Compute target_bearing = current_yaw + yaw_offset
    6. Send fake GLOBAL_POSITION_INT target

Example:

python3 test.py \
  --port udpout:127.0.0.1:18571 \
  --tracker-rx udpin:0.0.0.0:14650 \
  --auto-center-from-tracker \
  --tracker-sysid 2 \
  --sysid 1 \
  --mode static \
  --yaw-offset -30 \
  --bearing-radius 200 \
  --alt-mode fixed \
  --alt 50 \
  --rate 5 \
  --yaw-output 10 \
  --pitch-output 11
"""

import argparse
import math
import socket
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


# ============================================================
# Utility functions
# ============================================================

def wrap_360(deg: float) -> float:
    while deg < 0.0:
        deg += 360.0
    while deg >= 360.0:
        deg -= 360.0
    return deg


def wrap_180(deg: float) -> float:
    deg = wrap_360(deg)
    if deg > 180.0:
        deg -= 360.0
    return deg


def bearing_to_ne(bearing_deg: float, radius_m: float):
    """
    Bearing convention:
        0 deg   = North
        90 deg  = East
        180 deg = South
        270 deg = West

    Return:
        north_m, east_m
    """
    b = math.radians(bearing_deg)
    north_m = radius_m * math.cos(b)
    east_m = radius_m * math.sin(b)
    return north_m, east_m


def offset_to_latlon(center_lat_deg: float, center_lon_deg: float,
                     north_m: float, east_m: float):
    center_lat_rad = math.radians(center_lat_deg)

    dlat_deg = north_m / EARTH_RADIUS_M * (180.0 / math.pi)

    cos_lat = math.cos(center_lat_rad)
    if abs(cos_lat) < 1e-9:
        raise ValueError("Invalid latitude for lon offset calculation")

    dlon_deg = east_m / (EARTH_RADIUS_M * cos_lat) * (180.0 / math.pi)

    return center_lat_deg + dlat_deg, center_lon_deg + dlon_deg


def bearing_deg_from_tracker(north_m: float, east_m: float) -> float:
    deg = math.degrees(math.atan2(east_m, north_m))
    return wrap_360(deg)


def heading_cdeg(vn_mps: float, ve_mps: float) -> int:
    """
    MAVLink GLOBAL_POSITION_INT hdg:
        centi-degrees
        0 = North
        clockwise positive
    """
    if abs(vn_mps) < 1e-6 and abs(ve_mps) < 1e-6:
        return 0

    deg = math.degrees(math.atan2(ve_mps, vn_mps))
    deg = wrap_360(deg)

    return int(deg * 100.0)


# ============================================================
# UDP writer for fake UAV target
# ============================================================

class UDPOut:
    """
    Simple UDP writer for pymavlink MAVLink encoder.
    """

    def __init__(self, host: str, port: int):
        self.addr = (host, port)
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.bytes_sent = 0

    def write(self, data):
        if isinstance(data, str):
            data = data.encode("latin1")
        n = self.sock.sendto(data, self.addr)
        self.bytes_sent += n
        return n

    def flush(self):
        pass

    def close(self):
        self.sock.close()


def parse_udpout(conn: str):
    """
    Accept:
        udpout:127.0.0.1:18571
        udp:127.0.0.1:18571
        127.0.0.1:18571
    """
    if conn.startswith("udpout:"):
        conn = conn[len("udpout:"):]
    elif conn.startswith("udp:"):
        conn = conn[len("udp:"):]

    if ":" not in conn:
        raise ValueError(
            "Port must look like udpout:HOST:PORT, "
            "example udpout:127.0.0.1:18571"
        )

    host, port_s = conn.rsplit(":", 1)
    return host, int(port_s)


# ============================================================
# Tracker MAVLink readback
# ============================================================

class TrackerState:
    def __init__(self):
        self.heartbeat = None
        self.nav = None
        self.att = None
        self.servo = None
        self.pos = None
        self.last_print = 0.0


def open_tracker_rx(rx_conn: str):
    """
    Example:
        udpin:0.0.0.0:14650
    """
    print(f"Opening tracker RX: {rx_conn}")
    return mavutil.mavlink_connection(rx_conn)


def drain_rx(rx, max_count=2000):
    """
    Remove old MAVLink messages from RX buffer.

    This is important because recv_match() may return old ATTITUDE messages.
    If we use an old yaw to compute yaw-offset, the test becomes wrong.
    """
    count = 0

    while count < max_count:
        msg = rx.recv_match(blocking=False)
        if msg is None:
            break
        count += 1

    return count


def wait_tracker_position(rx, tracker_sysid=2, timeout_s=10.0):
    """
    Read tracker GLOBAL_POSITION_INT and return:
        lat_deg, lon_deg, alt_m
    """
    print(f"Waiting for tracker GLOBAL_POSITION_INT from sysid={tracker_sysid} ...")
    t0 = time.time()

    while time.time() - t0 < timeout_s:
        msg = rx.recv_match(type="GLOBAL_POSITION_INT", blocking=True, timeout=1.0)

        if msg is None:
            continue

        if msg.get_srcSystem() != tracker_sysid:
            continue

        lat_deg = msg.lat / 1e7
        lon_deg = msg.lon / 1e7
        alt_m = msg.alt / 1000.0

        print(
            f"Tracker position: "
            f"lat={lat_deg:.7f}, lon={lon_deg:.7f}, alt={alt_m:.2f} m"
        )

        return lat_deg, lon_deg, alt_m

    raise RuntimeError("Timeout: cannot read tracker GLOBAL_POSITION_INT")


def wait_tracker_auto_armed(rx, tracker_sysid=2, timeout_s=20.0):
    """
    Wait until tracker heartbeat says:
        custom_mode = 10
        armed = True

    For ArduPilot AntennaTracker:
        custom_mode 10 = AUTO
    """
    print(f"Waiting for tracker AUTO + ARMED from sysid={tracker_sysid} ...")

    t0 = time.time()

    while time.time() - t0 < timeout_s:
        msg = rx.recv_match(type="HEARTBEAT", blocking=True, timeout=1.0)

        if msg is None:
            continue

        if msg.get_srcSystem() != tracker_sysid:
            continue

        mode = msg.custom_mode
        armed = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

        print(f"Tracker heartbeat: mode={mode}, armed={armed}, base_mode={msg.base_mode}")

        if mode == 10 and armed:
            print("Tracker is AUTO + ARMED.")
            return True

    raise RuntimeError("Timeout: tracker is not AUTO + ARMED")


def wait_latest_tracker_attitude(rx, tracker_sysid=2, timeout_s=5.0, sample_window_s=0.7):
    """
    Drain old messages, then collect ATTITUDE messages for sample_window_s.
    Return the latest attitude, not an old buffered one.
    """
    drained = drain_rx(rx)
    print(f"Drained {drained} old MAVLink messages before reading ATTITUDE.")

    print(f"Reading fresh tracker ATTITUDE from sysid={tracker_sysid} ...")

    t0 = time.time()
    first_att_time = None
    last_att = None

    while time.time() - t0 < timeout_s:
        msg = rx.recv_match(type="ATTITUDE", blocking=True, timeout=0.2)

        if msg is None:
            continue

        if msg.get_srcSystem() != tracker_sysid:
            continue

        last_att = msg

        if first_att_time is None:
            first_att_time = time.time()

        if time.time() - first_att_time >= sample_window_s:
            break

    if last_att is None:
        raise RuntimeError("Timeout: cannot read fresh tracker ATTITUDE")

    yaw_deg = wrap_360(math.degrees(last_att.yaw))
    pitch_deg = math.degrees(last_att.pitch)
    roll_deg = math.degrees(last_att.roll)

    print(
        f"Fresh tracker attitude: "
        f"yaw={yaw_deg:.1f} deg, "
        f"pitch={pitch_deg:.1f} deg, "
        f"roll={roll_deg:.1f} deg"
    )

    return yaw_deg, pitch_deg, roll_deg


def get_servo_raw(msg, index: int):
    """
    index = 1..16
    Return msg.servo{index}_raw if it exists.
    """
    return getattr(msg, f"servo{index}_raw", None)


def poll_tracker_debug(
    rx,
    state: TrackerState,
    tracker_sysid=2,
    print_period_s=1.0,
    yaw_output=10,
    pitch_output=11,
):
    """
    Non-blocking read of useful tracker messages.
    Call this inside main loop.
    """
    while True:
        msg = rx.recv_match(
            type=[
                "HEARTBEAT",
                "NAV_CONTROLLER_OUTPUT",
                "ATTITUDE",
                "SERVO_OUTPUT_RAW",
                "GLOBAL_POSITION_INT",
            ],
            blocking=False,
        )

        if msg is None:
            break

        if msg.get_srcSystem() != tracker_sysid:
            continue

        name = msg.get_type()

        if name == "HEARTBEAT":
            state.heartbeat = msg

        elif name == "NAV_CONTROLLER_OUTPUT":
            state.nav = msg

        elif name == "ATTITUDE":
            state.att = msg

        elif name == "SERVO_OUTPUT_RAW":
            state.servo = msg

        elif name == "GLOBAL_POSITION_INT":
            state.pos = msg

    now = time.time()
    if now - state.last_print < print_period_s:
        return

    state.last_print = now

    parts = []

    if state.heartbeat is not None:
        hb = state.heartbeat
        armed = bool(hb.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)

        parts.append(
            f"HB: mode={hb.custom_mode} "
            f"armed={armed} "
            f"base_mode={hb.base_mode}"
        )

    if state.nav is not None:
        nav = state.nav
        parts.append(
            f"NAV: bearing={nav.nav_bearing:.1f} "
            f"target={nav.target_bearing:.1f} "
            f"dist={nav.wp_dist} "
            f"pitch={nav.nav_pitch:.1f} "
            f"alt_err={nav.alt_error:.1f}"
        )

    if state.att is not None:
        att = state.att
        yaw_deg = wrap_360(math.degrees(att.yaw))
        pitch_deg = math.degrees(att.pitch)
        roll_deg = math.degrees(att.roll)

        parts.append(
            f"ATT: yaw={yaw_deg:.1f} "
            f"pitch={pitch_deg:.1f} "
            f"roll={roll_deg:.1f}"
        )

    if state.nav is not None and state.att is not None:
        target_yaw = state.nav.target_bearing
        actual_yaw = wrap_360(math.degrees(state.att.yaw))
        yaw_error = wrap_180(target_yaw - actual_yaw)

        target_pitch = state.nav.nav_pitch
        actual_pitch = math.degrees(state.att.pitch)
        pitch_error = target_pitch - actual_pitch

        parts.append(
            f"ERR: yaw={yaw_error:+.1f} "
            f"pitch={pitch_error:+.1f}"
        )

    if state.servo is not None:
        servo = state.servo

        yaw_pwm = get_servo_raw(servo, yaw_output)
        pitch_pwm = get_servo_raw(servo, pitch_output)

        parts.append(
            f"SERVO: "
            f"YAW=s{yaw_output}={yaw_pwm} "
            f"PITCH=s{pitch_output}={pitch_pwm} "
            f"| raw: "
            f"s1={get_servo_raw(servo, 1)} "
            f"s2={get_servo_raw(servo, 2)} "
            f"s9={get_servo_raw(servo, 9)} "
            f"s10={get_servo_raw(servo, 10)} "
            f"s11={get_servo_raw(servo, 11)} "
            f"s12={get_servo_raw(servo, 12)}"
        )

    if state.pos is not None:
        pos = state.pos
        parts.append(
            f"POS: lat={pos.lat / 1e7:.7f} "
            f"lon={pos.lon / 1e7:.7f} "
            f"alt={pos.alt / 1000.0:.1f}m "
            f"rel={pos.relative_alt / 1000.0:.1f}m"
        )

    if parts:
        print("[TRACKER] " + " | ".join(parts))


# ============================================================
# Fake UAV motion and altitude
# ============================================================

def compute_motion(args, elapsed: float):
    """
    Return:
        north_m, east_m, vn_mps, ve_mps
    """
    if args.mode == "circular":
        if args.radius <= 0:
            raise ValueError("radius must be > 0 for circular mode")

        omega = args.speed / args.radius
        angle = omega * elapsed

        # Starts North of tracker, then moves clockwise.
        north_m = args.radius * math.cos(angle)
        east_m = args.radius * math.sin(angle)

        # NED horizontal velocity.
        vn_mps = -args.speed * math.sin(angle)
        ve_mps = args.speed * math.cos(angle)

    elif args.mode == "linear":
        # Fly West -> East through tracker latitude.
        north_m = 0.0
        east_m = -args.line_half_length + args.speed * elapsed
        vn_mps = 0.0
        ve_mps = args.speed

    else:
        # Static target at fixed offset.
        north_m = args.static_north
        east_m = args.static_east
        vn_mps = 0.0
        ve_mps = 0.0

    return north_m, east_m, vn_mps, ve_mps


def compute_altitude(args, elapsed: float):
    """
    Return:
        alt_rel_m: altitude above tracker/home in meters
        v_up_mps : vertical speed, positive = up
    """
    if args.alt_mode == "fixed":
        return args.alt, 0.0

    if args.alt_period <= 0:
        raise ValueError("--alt-period must be > 0")

    alt_min = min(args.alt_min, args.alt_max)
    alt_max = max(args.alt_min, args.alt_max)

    if args.alt_mode == "sine":
        mean_alt = 0.5 * (alt_min + alt_max)
        amp = 0.5 * (alt_max - alt_min)
        omega = 2.0 * math.pi / args.alt_period

        alt_rel_m = mean_alt + amp * math.sin(omega * elapsed)
        v_up_mps = amp * omega * math.cos(omega * elapsed)

        return alt_rel_m, v_up_mps

    if args.alt_mode == "triangle":
        phase = (elapsed % args.alt_period) / args.alt_period

        if phase < 0.5:
            k = phase / 0.5
            alt_rel_m = alt_min + k * (alt_max - alt_min)
            v_up_mps = 2.0 * (alt_max - alt_min) / args.alt_period
        else:
            k = (phase - 0.5) / 0.5
            alt_rel_m = alt_max - k * (alt_max - alt_min)
            v_up_mps = -2.0 * (alt_max - alt_min) / args.alt_period

        return alt_rel_m, v_up_mps

    return args.alt, 0.0


# ============================================================
# MAVLink send functions
# ============================================================

def send_heartbeat(mav):
    base_mode = (
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED |
        mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
    )

    mav.heartbeat_send(
        mavutil.mavlink.MAV_TYPE_QUADROTOR,
        mavutil.mavlink.MAV_AUTOPILOT_ARDUPILOTMEGA,
        base_mode,
        0,
        mavutil.mavlink.MAV_STATE_ACTIVE,
    )


def send_gps_raw_int(mav, lat_e7: int, lon_e7: int, alt_msl_mm: int,
                     vn_mps: float, ve_mps: float, hdg_cdeg: int):
    ground_speed_cmps = int(math.hypot(vn_mps, ve_mps) * MPS_TO_CMPS)

    mav.gps_raw_int_send(
        int(time.time() * 1e6),
        3,
        lat_e7,
        lon_e7,
        alt_msl_mm,
        100,
        100,
        ground_speed_cmps,
        hdg_cdeg,
        12,
    )


def send_global_position_int(mav, elapsed: float, lat_e7: int, lon_e7: int,
                             alt_msl_mm: int, rel_alt_mm: int,
                             vn_mps: float, ve_mps: float, vd_mps: float,
                             hdg_cdeg: int):
    """
    GLOBAL_POSITION_INT:
        vx: North cm/s
        vy: East cm/s
        vz: Down cm/s, positive = down
    """
    mav.global_position_int_send(
        int(elapsed * 1000) & 0xFFFFFFFF,
        lat_e7,
        lon_e7,
        alt_msl_mm,
        rel_alt_mm,
        int(vn_mps * MPS_TO_CMPS),
        int(ve_mps * MPS_TO_CMPS),
        int(vd_mps * MPS_TO_CMPS),
        hdg_cdeg,
    )


# ============================================================
# Arguments
# ============================================================

def build_arg_parser():
    parser = argparse.ArgumentParser(
        description="Fake MAVLink UAV target for ArduPilot AntennaTracker"
    )

    parser.add_argument(
        "--port",
        default="udpout:127.0.0.1:18571",
        help="MAVLink output connection. Default: udpout:127.0.0.1:18571",
    )

    parser.add_argument(
        "--sysid",
        type=int,
        default=1,
        help="Fake UAV MAVLink system ID. Must match tracker SYSID_TARGET. Default: 1",
    )

    parser.add_argument(
        "--compid",
        type=int,
        default=1,
        help="Fake UAV component ID. Default: 1",
    )

    parser.add_argument(
        "--tracker-rx",
        default="",
        help="Read tracker MAVLink from this connection, example: udpin:0.0.0.0:14650",
    )

    parser.add_argument(
        "--tracker-sysid",
        type=int,
        default=2,
        help="Tracker MAVLink sysid. Default: 2",
    )

    parser.add_argument(
        "--auto-center-from-tracker",
        action="store_true",
        help="Read tracker GLOBAL_POSITION_INT and use it as center_lat/center_lon",
    )

    parser.add_argument(
        "--tracker-print-period",
        type=float,
        default=1.0,
        help="Tracker debug print period in seconds. Default: 1",
    )

    parser.add_argument(
        "--yaw-output",
        type=int,
        default=10,
        help="SERVO output index for yaw. AUX2 = SERVO10. Default: 10",
    )

    parser.add_argument(
        "--pitch-output",
        type=int,
        default=11,
        help="SERVO output index for pitch. AUX3 = SERVO11. Default: 11",
    )

    parser.add_argument(
        "--rate",
        type=float,
        default=5.0,
        help="GLOBAL_POSITION_INT rate in Hz. Default: 5",
    )

    parser.add_argument(
        "--duration",
        type=float,
        default=0.0,
        help="Seconds to run. 0 = infinite. Default: 0",
    )

    parser.add_argument(
        "--mode",
        choices=["circular", "linear", "static"],
        default="static",
        help="Horizontal motion mode. Default: static",
    )

    parser.add_argument(
        "--radius",
        type=float,
        default=150.0,
        help="Circle radius in meters for circular mode. Default: 150",
    )

    parser.add_argument(
        "--speed",
        type=float,
        default=10.0,
        help="Horizontal speed in m/s. Default: 10",
    )

    parser.add_argument(
        "--center-lat",
        type=float,
        default=10.7776481,
        help="Tracker/base latitude.",
    )

    parser.add_argument(
        "--center-lon",
        type=float,
        default=106.6899469,
        help="Tracker/base longitude.",
    )

    parser.add_argument(
        "--center-alt",
        type=float,
        default=0.0,
        help="Tracker/base MSL altitude in meters. Default: 0",
    )

    parser.add_argument(
        "--line-half-length",
        type=float,
        default=300.0,
        help="Linear mode starts this many meters West. Default: 300",
    )

    parser.add_argument(
        "--static-north",
        type=float,
        default=0.0,
        help="Static target North offset in meters. Default: 0",
    )

    parser.add_argument(
        "--static-east",
        type=float,
        default=50.0,
        help="Static target East offset in meters. Default: 50",
    )

    parser.add_argument(
        "--target-bearing",
        type=float,
        default=None,
        help=(
            "Absolute target bearing in degrees. "
            "0=N, 90=E, 180=S, 270=W. "
            "Only for static mode. Overrides --static-north/east."
        ),
    )

    parser.add_argument(
        "--yaw-offset",
        type=float,
        default=None,
        help=(
            "Relative yaw offset in degrees from fresh tracker ATT.yaw. "
            "Example: --yaw-offset -30 or --yaw-offset 60. "
            "Requires --tracker-rx and tracker AUTO+ARMED. "
            "Only for static mode."
        ),
    )

    parser.add_argument(
        "--bearing-radius",
        type=float,
        default=200.0,
        help=(
            "Target distance in meters when using --target-bearing "
            "or --yaw-offset. Default: 200"
        ),
    )

    parser.add_argument(
        "--hold-latlon",
        action="store_true",
        help=(
            "Force fake UAV lat/lon exactly equal to center-lat/center-lon. "
            "Not recommended for yaw/pitch test because distance = 0."
        ),
    )

    parser.add_argument(
        "--alt",
        type=float,
        default=80.0,
        help="Fixed target altitude above tracker in meters. Used when --alt-mode fixed. Default: 80",
    )

    parser.add_argument(
        "--alt-mode",
        choices=["fixed", "sine", "triangle"],
        default="sine",
        help="Altitude mode. Default: sine",
    )

    parser.add_argument(
        "--alt-min",
        type=float,
        default=20.0,
        help="Minimum altitude above tracker in meters for sine/triangle mode. Default: 20",
    )

    parser.add_argument(
        "--alt-max",
        type=float,
        default=80.0,
        help="Maximum altitude above tracker in meters for sine/triangle mode. Default: 80",
    )

    parser.add_argument(
        "--alt-period",
        type=float,
        default=20.0,
        help="Altitude up/down period in seconds. Default: 20",
    )

    return parser


# ============================================================
# Main
# ============================================================

def main():
    parser = build_arg_parser()
    args = parser.parse_args()

    if args.rate <= 0:
        print("ERROR: --rate must be > 0")
        sys.exit(2)

    if args.tracker_print_period <= 0:
        print("ERROR: --tracker-print-period must be > 0")
        sys.exit(2)

    if not (1 <= args.yaw_output <= 16):
        print("ERROR: --yaw-output must be 1..16")
        sys.exit(2)

    if not (1 <= args.pitch_output <= 16):
        print("ERROR: --pitch-output must be 1..16")
        sys.exit(2)

    if args.target_bearing is not None and args.yaw_offset is not None:
        print("ERROR: Use only one of --target-bearing or --yaw-offset, not both.")
        sys.exit(2)

    if args.bearing_radius <= 0:
        print("ERROR: --bearing-radius must be > 0")
        sys.exit(2)

    if args.yaw_offset is not None and not args.tracker_rx:
        print("ERROR: --yaw-offset requires --tracker-rx to read current ATT.yaw")
        sys.exit(2)

    if args.yaw_offset is not None and args.mode != "static":
        print("ERROR: --yaw-offset only supports --mode static")
        sys.exit(2)

    if args.target_bearing is not None and args.mode != "static":
        print("ERROR: --target-bearing only supports --mode static")
        sys.exit(2)

    try:
        host, port = parse_udpout(args.port)
    except Exception as e:
        print(f"ERROR: invalid --port: {e}")
        sys.exit(2)

    # ------------------------------------------------------------
    # Optional tracker RX
    # ------------------------------------------------------------
    rx = None
    tracker_state = TrackerState()

    if args.tracker_rx:
        rx = open_tracker_rx(args.tracker_rx)

        if args.auto_center_from_tracker:
            try:
                lat0, lon0, alt0 = wait_tracker_position(
                    rx,
                    tracker_sysid=args.tracker_sysid,
                    timeout_s=10.0,
                )
            except Exception as e:
                print(f"ERROR: cannot auto-center from tracker: {e}")
                print("Check that your router forwards tracker telemetry to --tracker-rx port.")
                sys.exit(3)

            args.center_lat = lat0
            args.center_lon = lon0
            args.center_alt = alt0

            print(
                f"Auto center from tracker: "
                f"{args.center_lat:.7f}, {args.center_lon:.7f}, alt={args.center_alt:.2f} m"
            )

        if args.yaw_offset is not None:
            try:
                wait_tracker_auto_armed(
                    rx,
                    tracker_sysid=args.tracker_sysid,
                    timeout_s=20.0,
                )

                current_yaw_deg, current_pitch_deg, current_roll_deg = wait_latest_tracker_attitude(
                    rx,
                    tracker_sysid=args.tracker_sysid,
                    timeout_s=5.0,
                    sample_window_s=0.7,
                )

            except Exception as e:
                print(f"ERROR: cannot prepare tracker attitude for --yaw-offset: {e}")
                print("Make sure tracker is in AUTO and ARMED before running yaw-offset test.")
                sys.exit(4)

            target_bearing = wrap_360(current_yaw_deg + args.yaw_offset)
            north_m, east_m = bearing_to_ne(target_bearing, args.bearing_radius)

            args.static_north = north_m
            args.static_east = east_m

            print()
            print("=== Auto yaw-offset target ===")
            print(f"Current tracker yaw : {current_yaw_deg:.1f} deg")
            print(f"Requested yaw offset: {args.yaw_offset:+.1f} deg")
            print(f"Target bearing      : {target_bearing:.1f} deg")
            print(f"Target radius       : {args.bearing_radius:.1f} m")
            print(f"Computed north/east : north={north_m:.1f} m, east={east_m:.1f} m")
            print()

        elif args.target_bearing is not None:
            target_bearing = wrap_360(args.target_bearing)
            north_m, east_m = bearing_to_ne(target_bearing, args.bearing_radius)

            args.static_north = north_m
            args.static_east = east_m

            print()
            print("=== Absolute target-bearing mode ===")
            print(f"Target bearing      : {target_bearing:.1f} deg")
            print(f"Target radius       : {args.bearing_radius:.1f} m")
            print(f"Computed north/east : north={north_m:.1f} m, east={east_m:.1f} m")
            print()

    else:
        if args.target_bearing is not None:
            target_bearing = wrap_360(args.target_bearing)
            north_m, east_m = bearing_to_ne(target_bearing, args.bearing_radius)

            args.static_north = north_m
            args.static_east = east_m

            print()
            print("=== Absolute target-bearing mode ===")
            print(f"Target bearing      : {target_bearing:.1f} deg")
            print(f"Target radius       : {args.bearing_radius:.1f} m")
            print(f"Computed north/east : north={north_m:.1f} m, east={east_m:.1f} m")
            print()

    # ------------------------------------------------------------
    # Fake UAV TX
    # ------------------------------------------------------------
    writer = UDPOut(host, port)
    mav = mavutil.mavlink.MAVLink(
        writer,
        srcSystem=args.sysid,
        srcComponent=args.compid,
    )

    period = 1.0 / args.rate
    t_start = time.time()

    last_hb = -999.0
    last_gps = -999.0
    msg_count = 0

    initial_bearing = bearing_deg_from_tracker(args.static_north, args.static_east)
    initial_distance = math.hypot(args.static_north, args.static_east)

    print()
    print("=== Fake MAVLink Target for AntennaTracker ===")
    print(f"TX Connection : udpout:{host}:{port}")
    print(f"RX Connection : {args.tracker_rx if args.tracker_rx else 'disabled'}")
    print(f"Fake SysID    : {args.sysid}  (must equal tracker SYSID_TARGET)")
    print(f"Fake CompID   : {args.compid}")
    print(f"Tracker SysID : {args.tracker_sysid}")
    print(f"Yaw Output    : SERVO{args.yaw_output}")
    print(f"Pitch Output  : SERVO{args.pitch_output}")
    print(f"Mode          : {args.mode}")
    print(f"Center        : {args.center_lat:.7f}, {args.center_lon:.7f}, alt={args.center_alt:.1f} m")
    print(f"Static offset : north={args.static_north:.1f} m, east={args.static_east:.1f} m")
    print(f"Static bearing: {initial_bearing:.1f} deg")
    print(f"Static dist   : {initial_distance:.1f} m")
    print(f"Alt mode      : {args.alt_mode}")
    print(f"Alt fixed     : {args.alt:.1f} m")
    print(f"Alt min/max   : {args.alt_min:.1f} m / {args.alt_max:.1f} m")
    print(f"Alt period    : {args.alt_period:.1f} s")
    print(f"Rate          : {args.rate:.1f} Hz")
    print("Duration      : infinite" if args.duration <= 0 else f"Duration      : {args.duration:.1f} s")
    print()

    if args.hold_latlon:
        print("WARNING: --hold-latlon makes horizontal distance = 0.")
        print("         For pitch/yaw test, prefer --mode static with bearing/radius.")
        print()

    try:
        while True:
            now = time.time()
            elapsed = now - t_start

            if args.duration > 0 and elapsed >= args.duration:
                print("Duration reached. Stopping.")
                break

            north_m, east_m, vn_mps, ve_mps = compute_motion(args, elapsed)

            if args.hold_latlon:
                north_m = 0.0
                east_m = 0.0
                vn_mps = 0.0
                ve_mps = 0.0
                lat = args.center_lat
                lon = args.center_lon
            else:
                lat, lon = offset_to_latlon(
                    args.center_lat,
                    args.center_lon,
                    north_m,
                    east_m,
                )

            alt_rel_m, v_up_mps = compute_altitude(args, elapsed)

            # MAVLink NED: vz positive down.
            vd_mps = -v_up_mps

            lat_e7 = int(lat * DEG_TO_E7)
            lon_e7 = int(lon * DEG_TO_E7)

            alt_msl_m = args.center_alt + alt_rel_m
            alt_msl_mm = int(alt_msl_m * M_TO_MM)
            rel_alt_mm = int(alt_rel_m * M_TO_MM)

            hdg = heading_cdeg(vn_mps, ve_mps)

            # Send fake heartbeat at 1 Hz.
            if elapsed - last_hb >= 1.0:
                send_heartbeat(mav)
                last_hb = elapsed

            # Send fake GPS_RAW_INT at 2 Hz.
            if elapsed - last_gps >= 0.5:
                send_gps_raw_int(
                    mav,
                    lat_e7,
                    lon_e7,
                    alt_msl_mm,
                    vn_mps,
                    ve_mps,
                    hdg,
                )
                last_gps = elapsed

            # Send fake GLOBAL_POSITION_INT at args.rate.
            send_global_position_int(
                mav,
                elapsed,
                lat_e7,
                lon_e7,
                alt_msl_mm,
                rel_alt_mm,
                vn_mps,
                ve_mps,
                vd_mps,
                hdg,
            )

            msg_count += 1

            # Read tracker output if enabled.
            if rx is not None:
                poll_tracker_debug(
                    rx,
                    tracker_state,
                    tracker_sysid=args.tracker_sysid,
                    print_period_s=args.tracker_print_period,
                    yaw_output=args.yaw_output,
                    pitch_output=args.pitch_output,
                )

            # Print fake target status at about 1 Hz.
            if msg_count % max(1, int(args.rate)) == 0:
                bearing = bearing_deg_from_tracker(north_m, east_m)
                dist = math.hypot(north_m, east_m)

                print(
                    f"[FAKE {elapsed:6.1f}s] "
                    f"sysid={args.sysid} "
                    f"lat={lat:.7f} lon={lon:.7f} "
                    f"bearing={bearing:6.1f} deg "
                    f"dist={dist:6.1f} m "
                    f"alt_rel={alt_rel_m:6.1f} m "
                    f"alt_msl={alt_msl_m:6.1f} m "
                    f"v_up={v_up_mps:+5.1f} m/s "
                    f"vel_N/E=({vn_mps:+5.1f},{ve_mps:+5.1f}) m/s "
                    f"bytes={writer.bytes_sent}"
                )

            time.sleep(period)

    except KeyboardInterrupt:
        print(f"\nStopped. Sent {msg_count} GLOBAL_POSITION_INT messages.")
        print(f"UDP bytes sent: {writer.bytes_sent}")

    finally:
        writer.close()


if __name__ == "__main__":
    main()
