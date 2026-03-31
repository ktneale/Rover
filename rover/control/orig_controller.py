#!/usr/bin/env python3
"""
controller.py

Simple rover controller / arbiter.

Responsibilities:
- Receive higher-level inputs over UDP
- Maintain a small control state machine
- Decide what motion command should be active
- Publish cmd_vel to rover_uart
- Stop safely on stale inputs

Initial supported modes:
    - manual
    - track

Example accepted UDP inputs:
    {"type":"set_mode","mode":"manual"}
    {"type":"set_mode","mode":"track"}

    {"type":"teleop_cmd","v":0.2,"w":0.0}
    {"type":"teleop_stop"}

    {"type":"vision_track","target":true,"err_x":-0.18,"area":4210}
    {"type":"vision_track","target":false}

Example published UDP outputs:
    {"type":"cmd_vel","v":0.2,"w":0.0}
    {"type":"stop"}
    {"type":"heartbeat", ...}
"""

from __future__ import annotations

import argparse
import socket
import time
from typing import Any, Optional

from rover.common.udp import udp_send_json, udp_recv_json_nonblocking
from rover.common.heartbeat import HeartbeatPublisher


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def now_monotonic() -> float:
    return time.monotonic()


class Controller:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.start_mono = now_monotonic()

        # UDP RX
        self.rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx_sock.bind((args.bind_host, args.bind_port))
        self.rx_sock.setblocking(False)

        # UDP TX
        self.tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        # Heartbeat
        self.heartbeat = HeartbeatPublisher(
            module_name="controller",
            publish_fn=self.publish_event,
            interval_s=args.heartbeat_s,
        )

        # Control state
        self.mode = args.initial_mode  # manual | track

        # Manual / teleop state
        self.last_teleop_cmd: Optional[dict[str, float]] = None
        self.last_teleop_time: float = 0.0

        # Vision tracking state
        self.target_seen: bool = False
        self.target_err_x: float = 0.0
        self.target_area: float = 0.0
        self.last_vision_time: float = 0.0
        self.last_track_err_x: float = 0.0

        # Latest published command cache
        self.last_cmd_v: float = 0.0
        self.last_cmd_w: float = 0.0
        self.last_cmd_time: float = 0.0
        self.last_cmd_source: str = "none"

        self.stop_sent = False

        self.running = True

    # -------------------------
    # UDP publish helpers
    # -------------------------

    def publish_event(self, obj: dict[str, Any]) -> None:
        udp_send_json(self.args.pub_host, self.args.pub_port, obj)

    def publish_cmd_vel(self, v: float, w: float, source: str) -> None:
        self.stop_sent = False
        v = clamp(v, -1.0, 1.0)
        w = clamp(w, -1.0, 1.0)

        self.last_cmd_v = v
        self.last_cmd_w = w
        self.last_cmd_time = now_monotonic()
        self.last_cmd_source = source

        self.publish_event({
            "type": "cmd_vel",
            "v": v,
            "w": w,
            "source": source,
            "ts": time.time(),
        })

    def publish_stop(self, source: str) -> None:
        if self.stop_sent:
            return

        self.stop_sent = True

        self.publish_event({
            "type": "stop",
            "source": source,
            "ts": time.time(),
        })

    # -------------------------
    # Input handling
    # -------------------------

    def handle_set_mode(self, msg: dict[str, Any]) -> None:
        print("mode!!")
        mode = msg.get("mode")
        if mode not in ("manual", "track"):
            self.publish_event({
                "type": "warning",
                "ts": time.time(),
                "src": "controller",
                "warning": "invalid_mode",
                "raw": msg,
            })
            return

        self.mode = mode
        self.publish_event({
            "type": "mode_changed",
            "ts": time.time(),
            "src": "controller",
            "mode": self.mode,
        })

    def handle_teleop_cmd(self, msg: dict[str, Any]) -> None:
        v = float(msg.get("v", 0.0))
        w = float(msg.get("w", 0.0))
        self.last_teleop_cmd = {
            "v": clamp(v, -1.0, 1.0),
            "w": clamp(w, -1.0, 1.0),
        }
        print("Teleop")
        self.last_teleop_time = now_monotonic()

    def handle_teleop_stop(self, _msg: dict[str, Any]) -> None:
        self.last_teleop_cmd = {"v": 0.0, "w": 0.0}
        self.last_teleop_time = now_monotonic()
        if self.mode == "manual":
            self.publish_stop(source="teleop_stop")

    def handle_vision_track(self, msg: dict[str, Any]) -> None:
        self.target_seen = bool(msg.get("target", False))
        self.target_err_x = float(msg.get("err_x", 0.0))
        self.target_area = float(msg.get("area", 0.0))
        self.last_vision_time = now_monotonic()

    def handle_ping(self) -> None:
        self.publish_event({
            "type": "pong",
            "ts": time.time(),
            "src": "controller",
        })

    def handle_message(self, msg: dict[str, Any]) -> None:
        msg_type = msg.get("type")
        print(msg)
        if msg_type == "set_mode":
            self.handle_set_mode(msg)
            return

        if msg_type == "teleop_cmd":
            self.handle_teleop_cmd(msg)
            return

        if msg_type == "teleop_stop":
            self.handle_teleop_stop(msg)
            return

        if msg_type == "vision_track":
            self.handle_vision_track(msg)
            return

        if msg_type == "ping":
            self.handle_ping()
            return

        self.publish_event({
            "type": "warning",
            "ts": time.time(),
            "src": "controller",
            "warning": "unknown_message",
            "raw": msg,
        })

    # -------------------------
    # Mode logic
    # -------------------------

    def teleop_is_fresh(self) -> bool:
        return (
            self.last_teleop_cmd is not None
            and (now_monotonic() - self.last_teleop_time) <= self.args.teleop_timeout_s
        )

    def vision_is_fresh(self) -> bool:
        return (now_monotonic() - self.last_vision_time) <= self.args.vision_timeout_s

    def compute_track_cmd(self) -> tuple[float, float]:
        """
        Very simple P/PD steering controller.

        err_x:
            negative -> target left
            positive -> target right

        Convention here:
            w > 0 => turn left
            w < 0 => turn right

        So w = -kp * err_x maps naturally if err_x is image-center based.
        """
        err = self.target_err_x
        derr = (err - self.last_track_err_x) / max(self.args.loop_sleep_s, 1e-3)
        self.last_track_err_x = err

        w = -(self.args.track_kp * err + self.args.track_kd * derr)
        w = clamp(w, -self.args.track_max_w, self.args.track_max_w)

        # Keep first version conservative: rotate only by default.
        v = self.args.track_forward_v if self.args.track_enable_forward else 0.0
        v = clamp(v, -1.0, 1.0)

        # Optional deadband to reduce twitching near center
        if abs(err) < self.args.track_deadband:
            w = 0.0

        return v, w

    def step_manual_mode(self) -> None:
        if not self.teleop_is_fresh():
            self.publish_stop(source="manual_timeout")
            return

        assert self.last_teleop_cmd is not None
        self.publish_cmd_vel(
            v=self.last_teleop_cmd["v"],
            w=self.last_teleop_cmd["w"],
            source="manual",
        )

    def step_track_mode(self) -> None:
        if not self.vision_is_fresh():
            self.publish_stop(source="track_vision_timeout")
            return

        if not self.target_seen:
            self.publish_stop(source="track_target_lost")
            return

        v, w = self.compute_track_cmd()
        self.publish_cmd_vel(v=v, w=w, source="track")

    def control_step(self) -> None:
        if self.mode == "manual":
            self.step_manual_mode()
            return

        if self.mode == "track":
            self.step_track_mode()
            return

        self.publish_stop(source="invalid_mode")

    # -------------------------
    # Heartbeat
    # -------------------------

    def build_heartbeat_summary(self) -> dict[str, Any]:
        now = now_monotonic()

        teleop_age = None
        if self.last_teleop_time > 0:
            teleop_age = now - self.last_teleop_time

        vision_age = None
        if self.last_vision_time > 0:
            vision_age = now - self.last_vision_time

        return {
            "mode": self.mode,
            "target_seen": self.target_seen,
            "target_err_x": self.target_err_x,
            "target_area": self.target_area,
            "teleop_age_s": teleop_age,
            "vision_age_s": vision_age,
            "last_cmd_v": self.last_cmd_v,
            "last_cmd_w": self.last_cmd_w,
            "last_cmd_source": self.last_cmd_source,
        }

    # -------------------------
    # Main loop
    # -------------------------

    def run(self) -> None:
        self.publish_event({
            "type": "controller_started",
            "ts": time.time(),
            "bind": f"{self.args.bind_host}:{self.args.bind_port}",
            "pub_dest": f"{self.args.pub_host}:{self.args.pub_port}",
            "initial_mode": self.mode,
        })
        print("Controller running...")
        try:
            while self.running:
                while True:
                    #print("recv json...")
                    msg = udp_recv_json_nonblocking(self.rx_sock)
                    if msg is None:
                        break
                    self.handle_message(msg)

                self.control_step()
                self.heartbeat.maybe_publish(summary=self.build_heartbeat_summary())

                time.sleep(self.args.loop_sleep_s)

        finally:
            self.publish_stop(source="controller_shutdown")
            self.rx_sock.close()
            self.tx_sock.close()


def build_arg_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="Simple rover controller / arbiter")

    ap.add_argument("--bind-host", default="127.0.0.1", help="UDP bind host for inputs")
    ap.add_argument("--bind-port", type=int, default=5010, help="UDP bind port for inputs")

    ap.add_argument("--pub-host", default="127.0.0.1", help="UDP publish destination host")
    ap.add_argument("--pub-port", type=int, default=5002, help="UDP publish destination port (rover_uart cmd port)")

    ap.add_argument("--initial-mode", default="manual", choices=["manual", "track"])
    ap.add_argument("--loop-sleep-s", type=float, default=0.05, help="Main loop sleep interval")
    ap.add_argument("--heartbeat-s", type=float, default=1.0, help="Heartbeat publish interval")

    ap.add_argument("--teleop-timeout-s", type=float, default=0.35, help="Manual command timeout")
    ap.add_argument("--vision-timeout-s", type=float, default=0.35, help="Vision message timeout")

    ap.add_argument("--track-kp", type=float, default=0.9, help="Track mode proportional gain")
    ap.add_argument("--track-kd", type=float, default=0.08, help="Track mode derivative gain")
    ap.add_argument("--track-max-w", type=float, default=0.7, help="Clamp angular command in track mode")
    ap.add_argument("--track-deadband", type=float, default=0.03, help="No steering inside this |err_x| band")
    ap.add_argument("--track-enable-forward", action="store_true", help="Allow forward motion in track mode")
    ap.add_argument("--track-forward-v", type=float, default=0.15, help="Forward speed in track mode if enabled")

    return ap


def main() -> int:
    args = build_arg_parser().parse_args()
    controller = Controller(args)
    controller.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
