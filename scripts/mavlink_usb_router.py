#!/usr/bin/env python3
"""
Minimal MAVLink USB router for bench testing a single Pixhawk.

It owns the USB serial port and forwards:
- Pixhawk -> QGC UDP endpoint, default 127.0.0.1:14550
- QGC -> Pixhawk, using the UDP source port from this router
- Fake target script -> Pixhawk, default listen port 127.0.0.1:18570

Example:
    python3 scripts/mavlink_usb_router.py --device /dev/ttyACM0

Then run QGC without opening /dev/ttyACM0 directly, and send fake target with:
    python3 scripts/test_mavlink_target.py --port udpout:127.0.0.1:18570 --sysid 2
"""

import argparse
import select
import socket
import sys
import time

try:
    import serial
except ImportError:
    print("ERROR: pyserial not installed. Run: pip install pyserial")
    sys.exit(1)


def parse_endpoint(value):
    host, port = value.rsplit(":", 1)
    return host, int(port)


def make_bound_udp(endpoint):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    sock.bind(endpoint)
    return sock


def open_serial(device, baud):
    try:
        return serial.Serial(device, baud, timeout=0, exclusive=True)
    except TypeError:
        # Older pyserial versions do not support exclusive locks.
        return serial.Serial(device, baud, timeout=0)


def main():
    parser = argparse.ArgumentParser(description="Minimal MAVLink USB router")
    parser.add_argument("--device", default="/dev/ttyACM0", help="Pixhawk USB device")
    parser.add_argument("--baud", type=int, default=921600, help="Serial baud rate")
    parser.add_argument("--qgc-out", default="127.0.0.1:14550", help="QGC UDP receive endpoint")
    parser.add_argument("--qgc-bind", default="127.0.0.1:14540", help="UDP socket for QGC replies")
    parser.add_argument("--inject-bind", default="127.0.0.1:18570", help="UDP socket for fake target injection")
    args = parser.parse_args()

    qgc_out = parse_endpoint(args.qgc_out)
    qgc_bind = parse_endpoint(args.qgc_bind)
    inject_bind = parse_endpoint(args.inject_bind)

    serial_port = None
    qgc_sock = make_bound_udp(qgc_bind)
    inject_sock = make_bound_udp(inject_bind)

    print("=== MAVLink USB Router ===")
    print(f"  Serial:       {args.device} @ {args.baud}")
    print(f"  QGC out:      udpout:{qgc_out[0]}:{qgc_out[1]}")
    print(f"  QGC bind:     udp:{qgc_bind[0]}:{qgc_bind[1]}")
    print(f"  Inject bind:  udp:{inject_bind[0]}:{inject_bind[1]}")
    print()
    print("Close QGC before starting this router, then reopen QGC with USB autoconnect disabled.")
    print("Fake target port: udpout:%s:%d" % inject_bind)
    print("Press Ctrl+C to stop.")

    counters = {
        "serial_to_qgc": 0,
        "qgc_to_serial": 0,
        "inject_to_serial": 0,
    }
    last_print = time.time()
    last_open_attempt = 0.0

    try:
        while True:
            if serial_port is None:
                now = time.time()

                if now - last_open_attempt >= 1.0:
                    last_open_attempt = now

                    try:
                        serial_port = open_serial(args.device, args.baud)
                        print(f"Serial connected: {args.device}")
                    except serial.SerialException as exc:
                        print(f"Waiting for serial {args.device}: {exc}")

                time.sleep(0.1)
                continue

            try:
                readable, _, _ = select.select([serial_port, qgc_sock, inject_sock], [], [], 0.1)
            except (OSError, serial.SerialException) as exc:
                print(f"Serial select failed, reconnecting: {exc}")
                serial_port.close()
                serial_port = None
                continue

            for source in readable:
                try:
                    if source is serial_port:
                        data = serial_port.read(4096)

                        if data:
                            qgc_sock.sendto(data, qgc_out)
                            counters["serial_to_qgc"] += len(data)

                    elif source is qgc_sock:
                        data, _ = qgc_sock.recvfrom(4096)

                        if data:
                            serial_port.write(data)
                            counters["qgc_to_serial"] += len(data)

                    elif source is inject_sock:
                        data, _ = inject_sock.recvfrom(4096)

                        if data:
                            serial_port.write(data)
                            counters["inject_to_serial"] += len(data)
                except (OSError, serial.SerialException) as exc:
                    print(f"Serial IO failed, reconnecting: {exc}")
                    serial_port.close()
                    serial_port = None

            now = time.time()

            if now - last_print >= 2.0:
                print(
                    "bytes: fc->qgc={serial_to_qgc} qgc->fc={qgc_to_serial} inject->fc={inject_to_serial}".format(
                        **counters
                    )
                )
                last_print = now

    except KeyboardInterrupt:
        print("\nStopping router.")
    finally:
        if serial_port is not None:
            serial_port.close()
        qgc_sock.close()
        inject_sock.close()


if __name__ == "__main__":
    main()
