#!/usr/bin/env python3
import argparse
import time

from rover.common.udp import udp_send_json
from rover.rover_ports import HOST, VISION_PORT


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def main() -> int:
    parser = argparse.ArgumentParser(description="Send fake vision message")
    parser.add_argument("--host", default=HOST)
    parser.add_argument("--port", type=int, default=VISION_PORT)
    parser.add_argument("--seen", action="store_true")
    parser.add_argument("--centered", action="store_true")
    parser.add_argument("--err", type=float, default=0.0)
    parser.add_argument("--repeat", type=int, default=1)
    parser.add_argument("--interval", type=float, default=0.1)
    args = parser.parse_args()

    msg = {
        "type": "vision.track",
        "ts": time.time(),
        "seen": args.seen,
        "centered": args.centered,
        "err_norm": clamp(args.err, -1.0, 1.0),
        "area": 1000,
    }

    for _ in range(args.repeat):
        msg["ts"] = time.time()
        udp_send_json(args.host, args.port, msg)
        time.sleep(args.interval)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
