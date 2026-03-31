#!/usr/bin/env python3
"""
udp_monitor.py

Simple UDP listener for debugging rover telemetry.

Usage:
    python3 udp_monitor.py --port 5002
    python3 udp_monitor.py --port 5002 --pretty
"""

import argparse
import json
import socket
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bind-host", default="0.0.0.0",
                        help="Host/interface to bind to")
    parser.add_argument("--port", type=int, required=True,
                        help="UDP port to listen on")
    parser.add_argument("--pretty", action="store_true",
                        help="Pretty print JSON")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((args.bind_host, args.port))

    print(f"Listening on {args.bind_host}:{args.port}")

    while True:
        data, addr = sock.recvfrom(65535)

        ts = time.strftime("%H:%M:%S")
        text = data.decode("utf-8", errors="replace")

        print(f"\n[{ts}] packet from {addr[0]}:{addr[1]}")

        try:
            obj = json.loads(text)
            if args.pretty:
                print(json.dumps(obj, indent=2))
            else:
                print(json.dumps(obj))
        except Exception:
            print(text)


if __name__ == "__main__":
    main()
