#!/usr/bin/env python3
import argparse
import json
import serial

from rover.common.udp import udp_send_json

DEFAULT_SERIAL_PORT = "/dev/ttyACM0"
DEFAULT_BAUD = 115200


def parse_args():
    ap = argparse.ArgumentParser(description="Read odometry JSON from serial and publish over UDP")
    ap.add_argument("--serial-port", default=DEFAULT_SERIAL_PORT, help="Serial device path")
    ap.add_argument("--baud", type=int, default=DEFAULT_BAUD, help="Serial baud rate")

    ap.add_argument("--odom-pub-host", default="10.42.0.213", help="Primary odom UDP destination host")
    ap.add_argument("--odom-pub-port", type=int, default=5017, help="Primary odom UDP destination port")

    ap.add_argument("--telemetry-pub-host", default=None, help="Optional telemetry UDP destination host")
    ap.add_argument("--telemetry-pub-port", type=int, default=None, help="Optional telemetry UDP destination port")

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


def main():
    args = parse_args()

    ser = serial.Serial(args.serial_port, args.baud, timeout=1)

    telemetry_enabled = (
        args.telemetry_pub_host is not None and
        args.telemetry_pub_port is not None
    )

    print(
        f"[ODOM] serial={args.serial_port} baud={args.baud} "
        f"-> odom={args.odom_pub_host}:{args.odom_pub_port}"
    )
    if telemetry_enabled:
        print(
            f"[ODOM] telemetry forwarding enabled "
            f"-> {args.telemetry_pub_host}:{args.telemetry_pub_port}"
        )

    while True:
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
