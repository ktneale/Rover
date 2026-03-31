#!/usr/bin/env python3
import argparse
import time

from rover.common.udp import udp_send_json
from rover.rover_ports import HOST, DRIVE_PORT

def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def main() -> int:
    parser = argparse.ArgumentParser(description="Send a test drive command over UDP")
    parser.add_argument("--host", default="127.0.0.1", help="Destination host")
    parser.add_argument("--port", type=int, default=20010, help="Destination port")
    parser.add_argument("--left", type=float, required=True, help="Left command in [-1.0, 1.0]")
    parser.add_argument("--right", type=float, required=True, help="Right command in [-1.0, 1.0]")
    parser.add_argument("--repeat", type=int, default=1, help="Number of times to send")
    parser.add_argument("--interval", type=float, default=0.1, help="Seconds between sends")
    args = parser.parse_args()

    left = clamp(args.left, -1.0, 1.0)
    right = clamp(args.right, -1.0, 1.0)

    msg = {
        "type": "drive.cmd",
        "ts": time.time(),
        "left": left,
        "right": right,
        "source": "send_drive",
    }

    for _ in range(args.repeat):
        msg["ts"] = time.time()
        #udp_send_json(args.host, args.port, msg)
        udp_send_json(HOST, DRIVE_PORT, msg)
        time.sleep(args.interval)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
