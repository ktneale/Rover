#!/usr/bin/env python3
"""
udp_send.py

Send a single JSON message over UDP.

Examples:
    ./udp_send.py --port 5001 --json '{"type":"stop"}'
    ./udp_send.py --port 5001 --json '{"type":"drive_lr","left":20,"right":20}'
    ./udp_send.py --port 5001 --json '{"type":"cmd_vel","v":0.0,"w":0.3}'
    ./udp_send.py --port 5001 --json '{"type":"request_imu"}'
"""

from __future__ import annotations

import argparse
import json
import socket
import sys


def main() -> int:
    ap = argparse.ArgumentParser(description="Send one JSON UDP packet")
    ap.add_argument("--host", default="127.0.0.1", help="Destination host")
    ap.add_argument("--port", type=int, required=True, help="Destination UDP port")
    ap.add_argument("--json", required=True, help="JSON payload string")
    args = ap.parse_args()

    try:
        obj = json.loads(args.json)
    except json.JSONDecodeError as e:
        print(f"Invalid JSON: {e}", file=sys.stderr)
        return 1

    data = json.dumps(obj, separators=(",", ":")).encode("utf-8")

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.sendto(data, (args.host, args.port))
    finally:
        sock.close()

    print(f"Sent to {args.host}:{args.port}: {json.dumps(obj)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
