#!/usr/bin/env python3
"""
mavlink_usb_router_mp_fixed.py

Windows/Linux-friendly MAVLink USB router for bench testing one Pixhawk tracker.
The router owns the Pixhawk USB COM port. Mission Planner must connect by UDP,
not directly to COM8.

Topology:
    Pixhawk tracker USB <-> this router <-> Mission Planner UDP
    fake target script ----udpout----> this router ----> Pixhawk tracker USB

Recommended Mission Planner setup:
    1. Close Mission Planner before starting the router.
    2. Start this router with --device COM8 on Windows or /dev/ttyACM0 on Linux.
    3. Open Mission Planner and connect using UDP on port 14550.
    4. Run test_mavlink_target_fixed.py to inject the fake target.
"""

import argparse
import socket
import sys
import threading
import time

try:
    import serial
except ImportError:
    print("ERROR: pyserial not installed. Run: pip install pyserial")
    sys.exit(1)


def parse_endpoint(value: str):
    host, port = value.rsplit(":", 1)
    return host, int(port)


def bind_udp(endpoint):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    sock.bind(endpoint)
    sock.settimeout(0.2)
    return sock


def open_serial(device: str, baud: int):
    # For USB CDC the baud value is usually not critical, but it must match for UART adapters.
    try:
        return serial.Serial(device, baudrate=baud, timeout=0.02, write_timeout=0.2, exclusive=True)
    except TypeError:
        return serial.Serial(device, baudrate=baud, timeout=0.02, write_timeout=0.2)


class RouterState:
    def __init__(self, gcs_addr):
        self.stop_event = threading.Event()
        self.serial_lock = threading.Lock()
        self.gcs_addr_lock = threading.Lock()
        self.gcs_addr = gcs_addr
        self.counters = {
            "fc_to_gcs": 0,
            "gcs_to_fc": 0,
            "inject_to_fc": 0,
            "inject_to_gcs": 0,
        }

    def get_gcs_addr(self):
        with self.gcs_addr_lock:
            return self.gcs_addr

    def set_gcs_addr(self, addr):
        with self.gcs_addr_lock:
            self.gcs_addr = addr

    def add_counter(self, name, n):
        self.counters[name] += n


def serial_to_gcs_loop(ser, gcs_sock, state: RouterState):
    while not state.stop_event.is_set():
        try:
            data = ser.read(4096)
            if data:
                gcs_sock.sendto(data, state.get_gcs_addr())
                state.add_counter("fc_to_gcs", len(data))
        except Exception as exc:
            print(f"ERROR serial->gcs: {exc}")
            state.stop_event.set()
            break


def udp_to_serial_loop(name, sock, ser, state: RouterState, learn_gcs_addr=False, mirror_to_gcs_sock=None, mirror_name=None):
    while not state.stop_event.is_set():
        try:
            data, addr = sock.recvfrom(8192)
        except socket.timeout:
            continue
        except Exception as exc:
            print(f"ERROR {name}: {exc}")
            state.stop_event.set()
            break

        if not data:
            continue

        if learn_gcs_addr:
            # Mission Planner may use a dynamic UDP port. Learn it from incoming packets.
            state.set_gcs_addr(addr)

        try:
            with state.serial_lock:
                ser.write(data)
            state.add_counter(name, len(data))
        except Exception as exc:
            print(f"ERROR write serial from {name}: {exc}")
            state.stop_event.set()
            break

        if mirror_to_gcs_sock is not None:
            try:
                mirror_to_gcs_sock.sendto(data, state.get_gcs_addr())
                if mirror_name:
                    state.add_counter(mirror_name, len(data))
            except Exception as exc:
                print(f"WARN mirror inject to GCS failed: {exc}")


def status_loop(state: RouterState):
    last = time.time()
    last_counts = dict(state.counters)
    while not state.stop_event.is_set():
        time.sleep(2.0)
        now = time.time()
        dt = max(1e-6, now - last)
        diffs = {k: state.counters[k] - last_counts.get(k, 0) for k in state.counters}
        print(
            "bytes/s: fc->gcs={:.0f} gcs->fc={:.0f} inject->fc={:.0f} inject->gcs={:.0f} | GCS={}:{}".format(
                diffs["fc_to_gcs"] / dt,
                diffs["gcs_to_fc"] / dt,
                diffs["inject_to_fc"] / dt,
                diffs["inject_to_gcs"] / dt,
                state.get_gcs_addr()[0],
                state.get_gcs_addr()[1],
            )
        )
        last = now
        last_counts = dict(state.counters)


def main():
    parser = argparse.ArgumentParser(description="MAVLink USB router for Mission Planner + fake target injection")
    parser.add_argument("--device", default="COM8" if sys.platform.startswith("win") else "/dev/ttyACM0",
                        help="Pixhawk USB device, e.g. COM8 or /dev/ttyACM0")
    parser.add_argument("--baud", type=int, default=115200,
                        help="Serial baud. USB CDC usually accepts 115200. Try 57600 if needed.")
    parser.add_argument("--gcs-out", default="127.0.0.1:14550",
                        help="Mission Planner UDP receive endpoint. Default: 127.0.0.1:14550")
    parser.add_argument("--gcs-bind", default="127.0.0.1:14540",
                        help="Router UDP bind for Mission Planner replies. Default: 127.0.0.1:14540")
    parser.add_argument("--inject-bind", default="127.0.0.1:18570",
                        help="Router UDP bind for fake target injection. Default: 127.0.0.1:18570")
    parser.add_argument("--mirror-inject", action="store_true",
                        help="Also forward fake target packets to Mission Planner for easier debugging.")
    args = parser.parse_args()

    gcs_out = parse_endpoint(args.gcs_out)
    gcs_bind = parse_endpoint(args.gcs_bind)
    inject_bind = parse_endpoint(args.inject_bind)

    print("=== MAVLink USB Router for Mission Planner ===")
    print(f"Serial        : {args.device} @ {args.baud}")
    print(f"GCS out       : udpout:{gcs_out[0]}:{gcs_out[1]}")
    print(f"GCS bind      : udp:{gcs_bind[0]}:{gcs_bind[1]}")
    print(f"Inject bind   : udp:{inject_bind[0]}:{inject_bind[1]}")
    print(f"Mirror inject : {args.mirror_inject}")
    print()
    print("IMPORTANT: Mission Planner must NOT open the COM port. Connect Mission Planner by UDP port 14550.")
    print("Fake target should use: udpout:%s:%d" % inject_bind)
    print()

    try:
        ser = open_serial(args.device, args.baud)
    except serial.SerialException as exc:
        print(f"ERROR: cannot open serial {args.device}: {exc}")
        print("Close Mission Planner/other apps that are using the COM port, then run again.")
        sys.exit(2)

    gcs_sock = bind_udp(gcs_bind)
    inject_sock = bind_udp(inject_bind)
    state = RouterState(gcs_out)

    threads = [
        threading.Thread(target=serial_to_gcs_loop, args=(ser, gcs_sock, state), daemon=True),
        threading.Thread(target=udp_to_serial_loop, args=("gcs_to_fc", gcs_sock, ser, state, True, None, None), daemon=True),
        threading.Thread(
            target=udp_to_serial_loop,
            args=("inject_to_fc", inject_sock, ser, state, False,
                  gcs_sock if args.mirror_inject else None,
                  "inject_to_gcs" if args.mirror_inject else None),
            daemon=True,
        ),
        threading.Thread(target=status_loop, args=(state,), daemon=True),
    ]

    for th in threads:
        th.start()

    try:
        while not state.stop_event.is_set():
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\nStopping router.")
        state.stop_event.set()
    finally:
        try:
            ser.close()
        except Exception:
            pass
        gcs_sock.close()
        inject_sock.close()


if __name__ == "__main__":
    main()