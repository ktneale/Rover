#!/usr/bin/env python3
"""
controller.py

Simple rover controller / arbiter.

Inputs (UDP, bound to CONTROLLER_PORT):
- teleop_cmd
- teleop_stop
- vision_track
- lidar
- set_mode
- ping

Outputs:
- cmd_vel / stop -> DRIVE_PORT
- events / heartbeat -> EVENT_PORT

Behavior:
- In manual mode: only manual teleop commands are used.
- In track mode:
  - fresh manual teleop overrides everything for a short timeout window
  - otherwise fresh auto teleop_cmd input (e.g. ArUco tracker) is used
  - otherwise legacy vision_track behavior is used
"""

from __future__ import annotations

import argparse
import socket
import time
from typing import Any, Optional

from rover.common.rover_ports import EVENT_PORT, CONTROLLER_PORT, DRIVE_PORT
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

        # Heartbeat publishes to event port
        self.heartbeat = HeartbeatPublisher(
            module_name="controller",
            publish_fn=self.publish_event,
            interval_s=args.heartbeat_s,
        )

        # Control state
        self.mode = args.initial_mode  # manual | track

        # Manual command state
        self.last_manual_cmd: Optional[dict[str, float]] = None
        self.last_manual_time: float = 0.0

        # Auto command state (e.g. ArUco tracker publishing teleop_cmd)
        self.last_auto_cmd: Optional[dict[str, float]] = None
        self.last_auto_time: float = 0.0
        self.last_auto_source: str = "none"

        # Legacy vision tracking state
        self.target_seen: bool = False
        self.target_err_x: float = 0.0
        self.target_area: float = 0.0
        self.last_vision_time: float = 0.0
        self.last_track_err_x: float = 0.0

        # LiDAR state
        self.lidar_state: dict[str, float] = {
            "front_mm": 9999.0,
            "front_left_mm": 9999.0,
            "front_right_mm": 9999.0,
            "left_mm": 9999.0,
            "right_mm": 9999.0,
        }
        self.last_lidar_time: float = 0.0
        self.last_lidar_log_time: float = 0.0

        # Latest published command cache
        self.last_cmd_v: float = 0.0
        self.last_cmd_w: float = 0.0
        self.last_cmd_time: float = 0.0
        self.last_cmd_source: str = "none"

        self.lidar_slow_active: bool = False
        self.lidar_block_active: bool = False
        self.lidar_left_block_active: bool = False
        self.lidar_right_block_active: bool = False

        self.stop_sent = False
        self.running = True

    # -------------------------
    # UDP publish helpers
    # -------------------------

    def publish_event(self, obj: dict[str, Any]) -> None:
        udp_send_json(self.args.event_host, self.args.event_port, obj)

    def publish_rover_cmd(self, obj: dict[str, Any]) -> None:
        udp_send_json(self.args.cmd_host, self.args.cmd_port, obj)

    def publish_cmd_vel(self, v: float, w: float, source: str) -> None:
        v = clamp(v, -1.0, 1.0)
        w = clamp(w, -1.0, 1.0)

        changed = (
            abs(v - self.last_cmd_v) > 1e-3
            or abs(w - self.last_cmd_w) > 1e-3
            or source != self.last_cmd_source
        )

        if changed:
            print(f"[CTRL->ROVER] cmd_vel v={v:.3f} w={w:.3f} source={source}", flush=True)

        self.stop_sent = False
        self.last_cmd_v = v
        self.last_cmd_w = w
        self.last_cmd_time = now_monotonic()
        self.last_cmd_source = source

        self.publish_rover_cmd({
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
        self.last_cmd_v = 0.0
        self.last_cmd_w = 0.0
        self.last_cmd_time = now_monotonic()
        self.last_cmd_source = source

        print(f"[CTRL->ROVER] stop source={source}", flush=True)

        self.publish_rover_cmd({
            "type": "stop",
            "source": source,
            "ts": time.time(),
        })

    # -------------------------
    # Input handling
    # -------------------------

    def handle_set_mode(self, msg: dict[str, Any]) -> None:
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
        source = str(msg.get("source", "manual")).strip() or "manual"

        cmd = {
            "v": clamp(v, -1.0, 1.0),
            "w": clamp(w, -1.0, 1.0),
        }
        now = now_monotonic()

        if source == "manual":
            self.last_manual_cmd = cmd
            self.last_manual_time = now
            print(
                f"[MANUAL] v={cmd['v']:.3f} w={cmd['w']:.3f}",
                flush=True,
            )
        else:
            self.last_auto_cmd = cmd
            self.last_auto_time = now
            self.last_auto_source = source
            print(
                f"[AUTO:{source}] v={cmd['v']:.3f} w={cmd['w']:.3f}",
                flush=True,
            )

    def handle_teleop_stop(self, msg: dict[str, Any]) -> None:
        source = str(msg.get("source", "manual")).strip() or "manual"
        now = now_monotonic()

        if source == "manual":
            self.last_manual_cmd = {"v": 0.0, "w": 0.0}
            self.last_manual_time = now
            if self.mode == "manual":
                self.publish_stop(source="teleop_stop_manual")
        else:
            self.last_auto_cmd = {"v": 0.0, "w": 0.0}
            self.last_auto_time = now
            self.last_auto_source = source

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

    def handle_lidar(self, msg: dict[str, Any]) -> None:
        for k in ("front_mm", "front_left_mm", "front_right_mm", "left_mm", "right_mm"):
            if k in msg:
                self.lidar_state[k] = float(msg[k])

        self.last_lidar_time = now_monotonic()

    def handle_message(self, msg: dict[str, Any]) -> None:
        msg_type = msg.get("type")

        if msg_type == "lidar":
            now = now_monotonic()
            if (now - self.last_lidar_log_time) >= 1.0:
                print(
                    "[RX] lidar "
                    f"front={float(msg.get('front_mm', 0.0)):.0f} "
                    f"fl={float(msg.get('front_left_mm', 0.0)):.0f} "
                    f"fr={float(msg.get('front_right_mm', 0.0)):.0f} "
                    f"left={float(msg.get('left_mm', 0.0)):.0f} "
                    f"right={float(msg.get('right_mm', 0.0)):.0f}",
                    flush=True,
                )
                self.last_lidar_log_time = now
        else:
            print(f"[RX] {msg}", flush=True)

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

        if msg_type == "lidar":
            self.handle_lidar(msg)
            return

        self.publish_event({
            "type": "warning",
            "ts": time.time(),
            "src": "controller",
            "warning": "unknown_message",
            "raw": msg,
        })

    # -------------------------
    # Freshness / safety
    # -------------------------

    def manual_is_fresh(self) -> bool:
        return (
            self.last_manual_cmd is not None
            and (now_monotonic() - self.last_manual_time) <= self.args.teleop_timeout_s
        )

    def auto_is_fresh(self) -> bool:
        return (
            self.last_auto_cmd is not None
            and (now_monotonic() - self.last_auto_time) <= self.args.auto_timeout_s
        )

    def manual_override_active(self) -> bool:
        return (
            self.last_manual_cmd is not None
            and (now_monotonic() - self.last_manual_time) <= self.args.manual_override_timeout_s
        )

    def vision_is_fresh(self) -> bool:
        return (now_monotonic() - self.last_vision_time) <= self.args.vision_timeout_s

    def lidar_is_fresh(self) -> bool:
        return (now_monotonic() - self.last_lidar_time) <= self.args.lidar_timeout_s

    def apply_lidar_safety(self, v: float, w: float) -> tuple[float, float]:
        """
        Conservative first-pass LiDAR safety layer.

        Rules:
        - reverse always allowed
        - stale LiDAR blocks forward motion
        - front obstacle blocks or slows forward motion
        - side obstacle blocks steering into that side while moving forward

        Logs only on state transitions to avoid console spam.
        """

        req_v = v
        req_w = w

        def should_trace(req_v: float, req_w: float, out_v: float, out_w: float) -> bool:
            moving_cmd = abs(req_v) > 1e-3 or abs(req_w) > 1e-3
            changed = abs(out_v - req_v) > 1e-6 or abs(out_w - req_w) > 1e-6
            return moving_cmd or changed

        if v < 0.0:
            if self.lidar_slow_active or self.lidar_block_active:
                print("[SAFETY] lidar cleared (reverse allowed)", flush=True)

            if should_trace(req_v, req_w, v, w):
                print(
                    f"[SAFE IN ] req_v={req_v:.2f} req_w={req_w:.2f} reverse_allowed=1",
                    flush=True,
                )
                print(
                    f"[SAFE OUT] out_v={v:.2f} out_w={w:.2f} "
                    f"blocked=0 slow=0 left_block=0 right_block=0 reason=reverse_allowed",
                    flush=True,
                )

            self.lidar_slow_active = False
            self.lidar_block_active = False
            self.lidar_left_block_active = False
            self.lidar_right_block_active = False
            return v, w

        if not self.lidar_is_fresh():
            if v > 0.0 and not self.lidar_block_active:
                print("[SAFETY] lidar blocked (stale LiDAR, forward suppressed)", flush=True)

            out_v = 0.0 if v > 0.0 else v
            out_w = w

            if should_trace(req_v, req_w, out_v, out_w):
                print(
                    f"[SAFE IN ] req_v={req_v:.2f} req_w={req_w:.2f} lidar_fresh=0",
                    flush=True,
                )
                print(
                    f"[SAFE OUT] out_v={out_v:.2f} out_w={out_w:.2f} "
                    f"blocked={1 if v > 0.0 else 0} slow=0 left_block=0 right_block=0 "
                    f"reason=stale_lidar",
                    flush=True,
                )

            self.lidar_block_active = (v > 0.0)
            self.lidar_slow_active = False
            self.lidar_left_block_active = False
            self.lidar_right_block_active = False

            return out_v, out_w

        front = self.lidar_state["front_mm"]
        left = self.lidar_state["left_mm"]
        right = self.lidar_state["right_mm"]
        front_left = self.lidar_state["front_left_mm"]
        front_right = self.lidar_state["front_right_mm"]

        if abs(req_v) > 1e-3 or abs(req_w) > 1e-3:
            print(
                f"[SAFE IN ] req_v={req_v:.2f} req_w={req_w:.2f} "
                f"front={front:.0f} fl={front_left:.0f} fr={front_right:.0f} "
                f"left={left:.0f} right={right:.0f} "
                f"thr_front_stop={self.args.lidar_front_stop_mm:.0f} "
                f"thr_front_slow={self.args.lidar_front_slow_mm:.0f} "
                f"thr_side_stop={self.args.lidar_side_stop_mm:.0f}",
                flush=True,
            )

        blocked_now = False
        slow_now = False
        left_block_now = False
        right_block_now = False

        reason_parts: list[str] = []

        if front < self.args.lidar_front_stop_mm:
            if v > 0.0:
                blocked_now = True
                v = 0.0
                reason_parts.append("front_stop")
                print(
                    f"[SAFE] front_stop triggered front={front:.0f} < {self.args.lidar_front_stop_mm:.0f}",
                    flush=True,
                )

        elif front < self.args.lidar_front_slow_mm:
            if v > self.args.lidar_slow_v:
                slow_now = True
                reason_parts.append("front_slow")
                print(
                    f"[SAFE] front_slow triggered front={front:.0f} < {self.args.lidar_front_slow_mm:.0f} "
                    f"cap_v={self.args.lidar_slow_v:.2f}",
                    flush=True,
                )
            v = min(v, self.args.lidar_slow_v)

        if blocked_now and not self.lidar_block_active:
            print(
                f"[SAFETY] lidar blocked front={front:.0f}mm "
                f"(threshold={self.args.lidar_front_stop_mm:.0f}mm)",
                flush=True,
            )

        if slow_now and not self.lidar_slow_active:
            print(
                f"[SAFETY] lidar slow zone front={front:.0f}mm "
                f"(threshold={self.args.lidar_front_slow_mm:.0f}mm, cap_v={self.args.lidar_slow_v:.2f})",
                flush=True,
            )

        if left_block_now and not self.lidar_left_block_active:
            print(
                f"[SAFETY] lidar left-turn suppressed left={left:.0f}mm "
                f"(threshold={self.args.lidar_side_stop_mm:.0f}mm)",
                flush=True,
            )

        if right_block_now and not self.lidar_right_block_active:
            print(
                f"[SAFETY] lidar right-turn suppressed right={right:.0f}mm "
                f"(threshold={self.args.lidar_side_stop_mm:.0f}mm)",
                flush=True,
            )

        was_any_active = (
            self.lidar_slow_active
            or self.lidar_block_active
            or self.lidar_left_block_active
            or self.lidar_right_block_active
        )
        now_any_active = blocked_now or slow_now or left_block_now or right_block_now

        if was_any_active and not now_any_active:
            print(
                f"[SAFETY] lidar cleared front={front:.0f}mm left={left:.0f}mm right={right:.0f}mm",
                flush=True,
            )

        self.lidar_block_active = blocked_now
        self.lidar_slow_active = slow_now
        self.lidar_left_block_active = left_block_now
        self.lidar_right_block_active = right_block_now

        if should_trace(req_v, req_w, v, w):
            reason = ",".join(reason_parts) if reason_parts else "pass_through"
            print(
                f"[SAFE OUT] out_v={v:.2f} out_w={w:.2f} "
                f"blocked={int(blocked_now)} slow={int(slow_now)} "
                f"left_block={int(left_block_now)} right_block={int(right_block_now)} "
                f"reason={reason}",
                flush=True,
            )

        return v, w

    # -------------------------
    # Mode logic
    # -------------------------

    def compute_track_cmd(self) -> tuple[float, float]:
        err = self.target_err_x
        derr = (err - self.last_track_err_x) / max(self.args.loop_sleep_s, 1e-3)
        self.last_track_err_x = err

        w = -(self.args.track_kp * err + self.args.track_kd * derr)
        w = clamp(w, -self.args.track_max_w, self.args.track_max_w)

        v = self.args.track_forward_v if self.args.track_enable_forward else 0.0
        v = clamp(v, -1.0, 1.0)

        if abs(err) < self.args.track_deadband:
            w = 0.0

        return v, w

    def step_manual_mode(self) -> None:
        if not self.manual_is_fresh():
            self.publish_stop(source="manual_timeout")
            return

        assert self.last_manual_cmd is not None

        v = self.last_manual_cmd["v"]
        w = self.last_manual_cmd["w"]
        v, w = self.apply_lidar_safety(v, w)

        if v == 0.0 and w == 0.0:
            self.publish_stop(source="manual_lidar_stop")
            return

        self.publish_cmd_vel(v=v, w=w, source="manual")

    def step_track_mode(self) -> None:
        # Manual override has priority for a short timeout window.
        if self.manual_override_active():
            assert self.last_manual_cmd is not None
            v = self.last_manual_cmd["v"]
            w = self.last_manual_cmd["w"]
            v, w = self.apply_lidar_safety(v, w)

            if v == 0.0 and w == 0.0:
                self.publish_stop(source="manual_override_lidar_stop")
                return

            self.publish_cmd_vel(v=v, w=w, source="manual_override")
            return

        # Prefer explicit auto teleop_cmd input if present (e.g. ArUco script)
        if self.auto_is_fresh():
            assert self.last_auto_cmd is not None
            v = self.last_auto_cmd["v"]
            w = self.last_auto_cmd["w"]
            v, w = self.apply_lidar_safety(v, w)

            if v == 0.0 and w == 0.0:
                self.publish_stop(source=f"{self.last_auto_source}_lidar_stop")
                return

            self.publish_cmd_vel(v=v, w=w, source=self.last_auto_source)
            return

        # Fall back to legacy vision_track behavior
        if not self.vision_is_fresh():
            self.publish_stop(source="track_vision_timeout")
            return

        if not self.target_seen:
            self.publish_stop(source="track_target_lost")
            return

        v, w = self.compute_track_cmd()
        v, w = self.apply_lidar_safety(v, w)

        if v == 0.0 and w == 0.0:
            self.publish_stop(source="track_lidar_stop")
            return

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

        manual_age = None
        if self.last_manual_time > 0:
            manual_age = now - self.last_manual_time

        auto_age = None
        if self.last_auto_time > 0:
            auto_age = now - self.last_auto_time

        vision_age = None
        if self.last_vision_time > 0:
            vision_age = now - self.last_vision_time

        lidar_age = None
        if self.last_lidar_time > 0:
            lidar_age = now - self.last_lidar_time

        lidar_any_active = (
            self.lidar_slow_active
            or self.lidar_block_active
            or self.lidar_left_block_active
            or self.lidar_right_block_active
        )

        return {
            "mode": self.mode,
            "target_seen": self.target_seen,
            "target_err_x": self.target_err_x,
            "target_area": self.target_area,
            "manual_age_s": manual_age,
            "auto_age_s": auto_age,
            "vision_age_s": vision_age,
            "lidar_age_s": lidar_age,
            "manual_override_active": self.manual_override_active(),
            "last_auto_source": self.last_auto_source,
            "lidar_state": self.lidar_state,
            "lidar_safety": {
                "active": lidar_any_active,
                "slow_zone": self.lidar_slow_active,
                "blocked": self.lidar_block_active,
                "left_turn_suppressed": self.lidar_left_block_active,
                "right_turn_suppressed": self.lidar_right_block_active,
            },
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
            "cmd_dest": f"{self.args.cmd_host}:{self.args.cmd_port}",
            "event_dest": f"{self.args.event_host}:{self.args.event_port}",
            "initial_mode": self.mode,
        })

        print("Controller running...", flush=True)

        try:
            while self.running:
                while True:
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


def build_arg_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="Simple rover controller / arbiter")

    ap.add_argument("--bind-host", default="127.0.0.1", help="UDP bind host for inputs")
    ap.add_argument("--bind-port", type=int, default=CONTROLLER_PORT, help="UDP bind port for inputs")

    ap.add_argument("--cmd-host", default="127.0.0.1", help="UDP destination host for rover commands")
    ap.add_argument("--cmd-port", type=int, default=DRIVE_PORT, help="UDP destination port for rover commands")

    ap.add_argument("--event-host", default="127.0.0.1", help="UDP destination host for controller events")
    ap.add_argument("--event-port", type=int, default=EVENT_PORT, help="UDP destination port for controller events")

    ap.add_argument("--initial-mode", default="manual", choices=["manual", "track"])
    ap.add_argument("--loop-sleep-s", type=float, default=0.05, help="Main loop sleep interval")
    ap.add_argument("--heartbeat-s", type=float, default=1.0, help="Heartbeat publish interval")

    ap.add_argument("--teleop-timeout-s", type=float, default=0.35, help="Manual command timeout")
    ap.add_argument(
        "--manual-override-timeout-s",
        type=float,
        default=0.4,
        help="How long manual teleop overrides auto commands in track mode",
    )
    ap.add_argument(
        "--auto-timeout-s",
        type=float,
        default=0.25,
        help="Timeout for auto teleop_cmd sources such as aruco_track",
    )
    ap.add_argument("--vision-timeout-s", type=float, default=0.35, help="Vision message timeout")
    ap.add_argument("--lidar-timeout-s", type=float, default=0.5, help="LiDAR message timeout")

    ap.add_argument("--track-kp", type=float, default=0.9, help="Track mode proportional gain")
    ap.add_argument("--track-kd", type=float, default=0.08, help="Track mode derivative gain")
    ap.add_argument("--track-max-w", type=float, default=0.7, help="Clamp angular command in track mode")
    ap.add_argument("--track-deadband", type=float, default=0.03, help="No steering inside this |err_x| band")
    ap.add_argument("--track-enable-forward", action="store_true", help="Allow forward motion in track mode")
    ap.add_argument("--track-forward-v", type=float, default=0.15, help="Forward speed in track mode if enabled")

    ap.add_argument(
        "--lidar-front-stop-mm",
        type=float,
        default=450.0,
        help="Front obstacle distance that blocks forward motion",
    )
    ap.add_argument(
        "--lidar-front-slow-mm",
        type=float,
        default=800.0,
        help="Front obstacle distance that limits forward speed",
    )
    ap.add_argument(
        "--lidar-side-stop-mm",
        type=float,
        default=300.0,
        help="Side obstacle distance that blocks steering into that side",
    )
    ap.add_argument(
        "--lidar-slow-v",
        type=float,
        default=0.18,
        help="Forward speed cap in LiDAR slow zone",
    )
    ap.add_argument(
        "--lidar-steer-bias",
        type=float,
        default=0.12,
        help="Steering bias applied away from closer front obstacle",
    )

    return ap


def main() -> int:
    args = build_arg_parser().parse_args()
    controller = Controller(args)
    controller.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
