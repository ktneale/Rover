#!/usr/bin/env python3
import argparse
import json
import socket

import serial

from rover.common.rover_ports import HOST, ODOM_PORT
from rover.common.udp import udp_send_json, udp_recv_json_nonblocking

DEFAULT_SERIAL_PORT = "/dev/ttyACM0"
DEFAULT_BAUD = 115200
DEFAULT_SERIAL_TIMEOUT = 0.1


def send_serial_json(ser, obj):
    data = (json.dumps(obj, separators=(",", ":")) + "\n").encode("utf-8")
    ser.write(data)
    ser.flush()


def parse_args():
    ap = argparse.ArgumentParser(description="Read odometry JSON from serial and publish over UDP")
    ap.add_argument("--serial-port", default=DEFAULT_SERIAL_PORT, help="Serial device path")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Serial baud rate")
    ap.add_argument("--serial-timeout", type=float, default=DEFAULT_SERIAL_TIMEOUT, help="Serial read timeout in seconds")

    ap.add_argument("--odom-pub-host", default="10.42.0.213", help="Primary odom UDP destination host")
    ap.add_argument("--odom-pub-port", type=int, default=5017, help="Primary odom UDP destination port")

    ap.add_argument("--telemetry-pub-host", default=None, help="Optional telemetry UDP destination host")
    ap.add_argument("--telemetry-pub-port", type=int, default=None, help="Optional telemetry UDP destination port")
    ap.add_argument("--odom-bind-host", default=HOST, help="UDP bind host for odometry control messages")
    ap.add_argument("--odom-bind-port", type=int, default=ODOM_PORT, help="UDP bind port for odometry control messages")

    return ap.parse_args()


def build_odom_message(msg):
    return {
        "type": "odom",
        "seq": msg.get("seq"),
        "ms": msg.get("ms"),
        "dL": msg.get("dL"),
        "dR": msg.get("dR"),
        "tL": msg.get("tL"),
        "tR": msg.get("tR"),
        "x": msg.get("x"),
        "y": msg.get("y"),
        "th": msg.get("th"),
        "v": msg.get("v"),
        "w": msg.get("w"),
    }


def handle_control_message(ser, msg):
    if msg.get("type") != "odom_reset":
        return

    uart_msg = "reset\n"
    print(f"[ODOM CTRL] rx={msg} -> uart={uart_msg.strip()!r}", flush=True)
    ser.write(uart_msg.encode("utf-8"))
    ser.flush()


def main():
    args = parse_args()

    ser = serial.Serial(args.serial_port, args.baud, timeout=args.serial_timeout)
    odom_ctrl_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    odom_ctrl_sock.bind((args.odom_bind_host, args.odom_bind_port))
    odom_ctrl_sock.setblocking(False)

    telemetry_enabled = (
        args.telemetry_pub_host is not None and
        args.telemetry_pub_port is not None
    )

    print(
        f"[ODOM] serial={args.serial_port} baud={args.baud} "
        f"-> odom={args.odom_pub_host}:{args.odom_pub_port}"
    )
    print(
        f"[ODOM] control listen -> {args.odom_bind_host}:{args.odom_bind_port}",
        flush=True,
    )
    if telemetry_enabled:
        print(
            f"[ODOM] telemetry forwarding enabled "
            f"-> {args.telemetry_pub_host}:{args.telemetry_pub_port}"
        )

    while True:
        while True:
            ctrl_msg = udp_recv_json_nonblocking(odom_ctrl_sock)
            if ctrl_msg is None:
                break
            handle_control_message(ser, ctrl_msg)

        line = ser.readline().decode(errors="ignore").strip()
        if not line or not line.startswith("{"):
            continue

        try:
            msg = json.loads(line)
        except json.JSONDecodeError:
            continue

        if msg.get("T") != "odom":
            continue

        out = build_odom_message(msg)

        udp_send_json(args.odom_pub_host, args.odom_pub_port, out)

        if telemetry_enabled:
            udp_send_json(args.telemetry_pub_host, args.telemetry_pub_port, out)


if __name__ == "__main__":
    main()
