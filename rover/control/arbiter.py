#!/usr/bin/env python3
import argparse
import select
import signal
import sys
import time
from typing import Any, Dict, Optional

from rover.common.udp import make_udp_rx_socket, udp_recv_json_nonblocking, udp_send_json
from rover.rover_ports import HOST, VISION_PORT, LIDAR_PORT, TELEOP_PORT, DRIVE_PORT


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


class Arbiter:
    def __init__(
        self,
        host: str,
        vision_port: int,
        lidar_port: int,
        teleop_port: int,
        drive_port: int,
        output_hz: float = 15.0,
        teleop_timeout: float = 0.5,
        vision_timeout: float = 0.5,
        lidar_timeout: float = 0.75,
        max_turn: float = 0.35,
        center_deadband: float = 0.08,
        verbose: bool = False,
    ) -> None:
        self.host = host
        self.vision_port = vision_port
        self.lidar_port = lidar_port
        self.teleop_port = teleop_port
        self.drive_port = drive_port

        self.output_hz = output_hz
        self.output_period = 1.0 / output_hz

        self.teleop_timeout = teleop_timeout
        self.vision_timeout = vision_timeout
        self.lidar_timeout = lidar_timeout

        self.max_turn = max_turn
        self.center_deadband = center_deadband
        self.verbose = verbose

        self.vision_sock = make_udp_rx_socket(host, vision_port, nonblocking=True)
        self.lidar_sock = make_udp_rx_socket(host, lidar_port, nonblocking=True)
        self.teleop_sock = make_udp_rx_socket(host, teleop_port, nonblocking=True)

        self.running = True

        self.latest_vision: Optional[Dict[str, Any]] = None
        self.latest_lidar: Optional[Dict[str, Any]] = None
        self.latest_teleop: Optional[Dict[str, Any]] = None

        self.last_vision_rx = 0.0
        self.last_lidar_rx = 0.0
        self.last_teleop_rx = 0.0

        self.last_sent = {"left": None, "right": None, "source": None}

    def log(self, *args) -> None:
        if self.verbose:
            print(*args, flush=True)

    def is_fresh(self, last_rx: float, timeout_s: float) -> bool:
        return (time.monotonic() - last_rx) <= timeout_s

    def teleop_fresh(self) -> bool:
        return self.latest_teleop is not None and self.is_fresh(self.last_teleop_rx, self.teleop_timeout)

    def vision_fresh(self) -> bool:
        return self.latest_vision is not None and self.is_fresh(self.last_vision_rx, self.vision_timeout)

    def lidar_fresh(self) -> bool:
        return self.latest_lidar is not None and self.is_fresh(self.last_lidar_rx, self.lidar_timeout)

    def recv_all(self) -> None:
        while True:
            got_any = False

            msg = udp_recv_json_nonblocking(self.vision_sock)
            if msg is not None:
                got_any = True
                if isinstance(msg, dict) and msg.get("type") == "vision.track":
                    self.latest_vision = msg
                    self.last_vision_rx = time.monotonic()
                    self.log(f"[arbiter] vision <- {msg}")

            msg = udp_recv_json_nonblocking(self.lidar_sock)
            if msg is not None:
                got_any = True
                if isinstance(msg, dict) and msg.get("type") == "lidar.obstacles":
                    self.latest_lidar = msg
                    self.last_lidar_rx = time.monotonic()
                    self.log(f"[arbiter] lidar <- {msg}")

            msg = udp_recv_json_nonblocking(self.teleop_sock)
            if msg is not None:
                got_any = True
                if isinstance(msg, dict) and msg.get("type") == "teleop.cmd":
                    self.latest_teleop = msg
                    self.last_teleop_rx = time.monotonic()
                    self.log(f"[arbiter] teleop <- {msg}")

            if not got_any:
                break

    def compute_drive(self) -> Dict[str, Any]:
        now = time.time()

        # 1) Manual override if teleop is fresh and enabled
        if self.teleop_fresh():
            enabled = bool(self.latest_teleop.get("enable", True))
            if enabled:
                left = clamp(float(self.latest_teleop.get("left", 0.0)), -1.0, 1.0)
                right = clamp(float(self.latest_teleop.get("right", 0.0)), -1.0, 1.0)
                return {
                    "type": "drive.cmd",
                    "ts": now,
                    "left": left,
                    "right": right,
                    "source": "teleop",
                }

        # 2) LiDAR safety override
        if self.lidar_fresh():
            front_blocked = bool(self.latest_lidar.get("front_blocked", False))
            if front_blocked:
                return {
                    "type": "drive.cmd",
                    "ts": now,
                    "left": 0.0,
                    "right": 0.0,
                    "source": "lidar_stop",
                }

        # 3) Vision-based rotate-to-center
        if self.vision_fresh():
            seen = bool(self.latest_vision.get("seen", False))
            if seen:
                centered = bool(self.latest_vision.get("centered", False))
                err_norm = float(self.latest_vision.get("err_norm", 0.0))

                if centered or abs(err_norm) <= self.center_deadband:
                    return {
                        "type": "drive.cmd",
                        "ts": now,
                        "left": 0.0,
                        "right": 0.0,
                        "source": "vision_centered",
                    }

                turn = clamp(err_norm, -1.0, 1.0) * self.max_turn
                return {
                    "type": "drive.cmd",
                    "ts": now,
                    "left": -turn,
                    "right": turn,
                    "source": "vision_track",
                }

        # 4) Default safe stop
        return {
            "type": "drive.cmd",
            "ts": now,
            "left": 0.0,
            "right": 0.0,
            "source": "idle_stop",
        }

    def send_drive(self, cmd: Dict[str, Any]) -> None:
        left = float(cmd["left"])
        right = float(cmd["right"])
        source = str(cmd["source"])

        changed = (
            self.last_sent["left"] != left or
            self.last_sent["right"] != right or
            self.last_sent["source"] != source
        )

        udp_send_json(self.host, self.drive_port, cmd)

        if changed:
            self.log(
                f"[arbiter] drive -> left={left:+.3f} right={right:+.3f} source={source}"
            )
            self.last_sent["left"] = left
            self.last_sent["right"] = right
            self.last_sent["source"] = source

    def run(self) -> None:
        self.log(
            f"[arbiter] listening vision={self.vision_port} lidar={self.lidar_port} "
            f"teleop={self.teleop_port}, output -> {self.drive_port}"
        )
        self.log(
            f"[arbiter] timeouts teleop={self.teleop_timeout}s vision={self.vision_timeout}s "
            f"lidar={self.lidar_timeout}s output_hz={self.output_hz}"
        )

        next_tick = time.monotonic()

        while self.running:
            self.recv_all()

            now = time.monotonic()
            if now >= next_tick:
                cmd = self.compute_drive()
                self.send_drive(cmd)
                next_tick = now + self.output_period

            time.sleep(0.005)


def main() -> int:
    parser = argparse.ArgumentParser(description="Rover command arbiter")
    parser.add_argument("--host", default=HOST, help="UDP host")
    parser.add_argument("--vision-port", type=int, default=VISION_PORT)
    parser.add_argument("--lidar-port", type=int, default=LIDAR_PORT)
    parser.add_argument("--teleop-port", type=int, default=TELEOP_PORT)
    parser.add_argument("--drive-port", type=int, default=DRIVE_PORT)
    parser.add_argument("--output-hz", type=float, default=15.0)
    parser.add_argument("--teleop-timeout", type=float, default=0.5)
    parser.add_argument("--vision-timeout", type=float, default=0.5)
    parser.add_argument("--lidar-timeout", type=float, default=0.75)
    parser.add_argument("--max-turn", type=float, default=0.35)
    parser.add_argument("--center-deadband", type=float, default=0.08)
    parser.add_argument("--verbose", action="store_true")
    args = parser.parse_args()

    arbiter = Arbiter(
        host=args.host,
        vision_port=args.vision_port,
        lidar_port=args.lidar_port,
        teleop_port=args.teleop_port,
        drive_port=args.drive_port,
        output_hz=args.output_hz,
        teleop_timeout=args.teleop_timeout,
        vision_timeout=args.vision_timeout,
        lidar_timeout=args.lidar_timeout,
        max_turn=args.max_turn,
        center_deadband=args.center_deadband,
        verbose=args.verbose,
    )

    def handle_signal(signum, _frame):
        print(f"[arbiter] signal {signum} received", flush=True)
        arbiter.running = False

    signal.signal(signal.SIGINT, handle_signal)
    signal.signal(signal.SIGTERM, handle_signal)

    try:
        arbiter.run()
    except KeyboardInterrupt:
        pass
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
