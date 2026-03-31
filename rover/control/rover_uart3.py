#!/usr/bin/env python3
"""
rover_uart.py

UART bridge for the rover ESP32 board.

Responsibilities:
- Own the UART device exclusively
- Receive internal UDP commands from other rover processes
- Translate those commands into rover-board JSON messages over UART
- Read rover-board JSON messages from UART in a background thread
- Republish received telemetry/status over UDP
- Enforce a motor watchdog timeout for safety
- Periodically poll IMU and publish it on the event stream

Example internal UDP commands accepted:
    {"type":"drive_lr","left":20,"right":20}
    {"type":"cmd_vel","v":0.0,"w":0.25}
    {"type":"stop"}
    {"type":"request_imu"}
    {"type":"request_status"}

Example UDP publications:
    {"type":"uart_rx","raw":{...}}
    {"type":"imu","yaw_deg":...}
    {"type":"board_status","raw":{...}}
"""

from __future__ import annotations

import argparse
import json
import queue
import socket
import threading
import time
from dataclasses import dataclass
from typing import Any, Optional

import serial

from rover.common.udp_utils import udp_send_json
from rover.common.heartbeat import HeartbeatPublisher
from rover.common.rover_ports import EVENT_PORT, DRIVE_PORT


# -----------------------------
# Utility helpers
# -----------------------------

def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def now_monotonic() -> float:
    return time.monotonic()


def json_dumps_line(obj: dict[str, Any]) -> bytes:
    return (json.dumps(obj, separators=(",", ":")) + "\n").encode("utf-8")


def udp_recv_json_nonblocking(sock: socket.socket, bufsize: int = 65535) -> Optional[dict[str, Any]]:
    try:
        data, _addr = sock.recvfrom(bufsize)
    except BlockingIOError:
        return None
    except OSError:
        return None

    try:
        return json.loads(data.decode("utf-8"))
    except Exception:
        return None


# -----------------------------
# UART reader thread
# -----------------------------

@dataclass
class UartEvent:
    ts: float
    msg: Optional[dict[str, Any]] = None
    raw_line: Optional[str] = None
    error: Optional[str] = None


class UartReaderThread(threading.Thread):
    def __init__(
        self,
        ser: serial.Serial,
        out_queue: queue.Queue[UartEvent],
        stop_event: threading.Event,
    ) -> None:
        super().__init__(daemon=True)
        self.ser = ser
        self.out_queue = out_queue
        self.stop_event = stop_event

    def run(self) -> None:
        while not self.stop_event.is_set():
            try:
                line = self.ser.readline()  # respects serial timeout
                if not line:
                    continue

                text = line.decode("utf-8", errors="replace").strip()
                if not text:
                    continue

                print(text, flush=True)

                try:
                    msg = json.loads(text)
                    self.out_queue.put(UartEvent(ts=now_monotonic(), msg=msg, raw_line=text))
                except json.JSONDecodeError:
                    self.out_queue.put(
                        UartEvent(
                            ts=now_monotonic(),
                            msg=None,
                            raw_line=text,
                            error="invalid_json",
                        )
                    )

            except serial.SerialException as e:
                self.out_queue.put(
                    UartEvent(
                        ts=now_monotonic(),
                        error=f"serial_exception: {e}",
                    )
                )
                time.sleep(0.2)
            except Exception as e:
                self.out_queue.put(
                    UartEvent(
                        ts=now_monotonic(),
                        error=f"reader_exception: {e}",
                    )
                )
                time.sleep(0.1)


# -----------------------------
# Rover UART bridge
# -----------------------------

class RoverUartBridge:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args

        self.stop_event = threading.Event()
        self.uart_write_lock = threading.Lock()
        self.rx_queue: queue.Queue[UartEvent] = queue.Queue()

        self.last_drive_cmd_time = 0.0
        self.watchdog_stopped = False

        self.last_imu_request_time = 0.0
        self.last_status_request_time = 0.0

        # Optional cached state
        self.latest_imu: Optional[dict[str, Any]] = None
        self.latest_status: Optional[dict[str, Any]] = None
        self.last_uart_rx_time: float = 0.0

        # UDP RX socket (commands in)
        self.cmd_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.cmd_sock.bind((args.cmd_bind_host, args.cmd_bind_port))
        print(f"[UDP RX BIND] {args.cmd_bind_host}:{args.cmd_bind_port}", flush=True)
        self.cmd_sock.setblocking(False)

        # UDP TX socket (telemetry out)
        self.pub_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.heartbeat = HeartbeatPublisher(
            module_name="rover_uart",
            publish_fn=self.publish_event,
            interval_s=args.heartbeat_s,
        )

        # UART
        self.ser = serial.Serial(
            port=args.serial,
            baudrate=args.baud,
            timeout=args.serial_timeout,
            write_timeout=args.serial_write_timeout,
        )

        # Reader thread
        self.reader = UartReaderThread(self.ser, self.rx_queue, self.stop_event)

    # -------------------------
    # UART TX helpers
    # -------------------------

    def send_to_uart(self, msg: str) -> None:
        if not msg.endswith("\n"):
            msg += "\n"

        data = msg.encode("utf-8")
        print(f"[UART TX] {data!r}", flush=True)

        with self.uart_write_lock:
            self.ser.write(data)
            self.ser.flush()

    def send_uart_json(self, obj: dict[str, Any]) -> None:
        self.send_to_uart(json.dumps(obj, separators=(",", ":")))

    def send_stop(self) -> None:
        self.send_uart_json({"T": 1, "L": 0, "R": 0})

    def send_drive_lr(self, left: float, right: float) -> None:
        left = float(clamp(left, -0.5, 0.5))
        right = float(clamp(right, -0.5, 0.5))

        print(f"[DRIVE_LR] left={left:.3f} right={right:.3f}", flush=True)
        self.send_uart_json({"T": 1, "L": left, "R": right})

    def send_cmd_vel(self, v: float, w: float) -> None:
        """
        Convert normalized robot motion command into board wheel commands.

        Internal convention:
            v in [-1.0, +1.0]
            w in [-1.0, +1.0]

        Board wheel command range:
            L, R in [-0.5, +0.5]
        """
        v = clamp(v, -1.0, 1.0)
        w = clamp(w, -1.0, 1.0)

        wheel_scale = 1.0

        left = (v - w) * wheel_scale
        right = (v + w) * wheel_scale

        print(
            f"[CMD_VEL->LR] v={v:.3f} w={w:.3f} left={left:.3f} right={right:.3f}",
            flush=True,
        )

        self.send_drive_lr(left, right)

    def request_imu(self) -> None:
        self.send_uart_json({"T": 126})

    def request_status(self) -> None:
        self.send_uart_json({"T": 130})

    # -------------------------
    # UART RX handling
    # -------------------------

    def classify_uart_message(self, msg: dict[str, Any]) -> dict[str, Any]:
        """
        Map raw rover-board JSON to internal UDP message format.
        """
        t = msg.get("T")

        if t == 126:
            return {
                "type": "imu",
                "ts": time.time(),
                "raw": msg,
            }

        if t == 1001:
            return {
                "type": "status",
                "ts": time.time(),
                "T": msg.get("T"),
                "L": msg.get("L"),
                "R": msg.get("R"),
                "r": msg.get("r"),
                "p": msg.get("p"),
                "y": msg.get("y"),
                "temp_c": msg.get("temp"),
                "v": msg.get("v"),
            }

        if t == 1002:
            return {
                "type": "imu",
                "ts": time.time(),
                "yaw_deg": msg.get("y"),
                "yaw_rate_dps": msg.get("gz"),
                "roll_deg": msg.get("r"),
                "pitch_deg": msg.get("p"),
                "accel_x": msg.get("ax"),
                "accel_y": msg.get("ay"),
                "accel_z": msg.get("az"),
                "temp_c": msg.get("temp"),
            }

        if t == 1002132:
            return {
                "type": "imu",
                "ts": time.time(),
                "roll_deg": msg.get("r"),
                "pitch_deg": msg.get("p"),
                "yaw_deg": msg.get("y"),
                "accel_x": msg.get("ax"),
                "accel_y": msg.get("ay"),
                "accel_z": msg.get("az"),
                "gyro_x": msg.get("gx"),
                "gyro_y": msg.get("gy"),
                "gyro_z": msg.get("gz"),
                "mag_x": msg.get("mx"),
                "mag_y": msg.get("my"),
                "mag_z": msg.get("mz"),
                "temp_c": msg.get("temp"),
                "raw": msg,
            }

        if t == 405:
            return {
                "type": "board_status",
                "ts": time.time(),
                "raw": msg,
            }

        return {
            "type": "uart_rx",
            "ts": time.time(),
            "raw": msg,
        }

    def publish_event(self, obj: dict[str, Any]) -> None:
        udp_send_json(self.pub_sock, self.args.pub_host, self.args.pub_port, obj)

    def handle_uart_event(self, ev: UartEvent) -> None:
        self.last_uart_rx_time = ev.ts

        if ev.error and ev.msg is None:
            self.publish_event({
                "type": "uart_error",
                "ts": time.time(),
                "error": ev.error,
                "raw_line": ev.raw_line,
            })
            return

        if ev.msg is None:
            return

        pub = self.classify_uart_message(ev.msg)

        if pub["type"] == "imu":
            self.latest_imu = pub
        elif pub["type"] in ("board_status", "status"):
            self.latest_status = pub

        self.publish_event(pub)

    # -------------------------
    # UDP command handling
    # -------------------------

    def handle_udp_command(self, cmd: dict[str, Any]) -> None:
        cmd_type = cmd.get("type")

        if cmd_type == "drive_lr":
            left = float(cmd.get("left", 0))
            right = float(cmd.get("right", 0))
            self.send_drive_lr(left, right)
            self.last_drive_cmd_time = now_monotonic()
            self.watchdog_stopped = False
            return

        if cmd_type == "cmd_vel":
            v = float(cmd.get("v", 0.0))
            w = float(cmd.get("w", 0.0))
            self.send_cmd_vel(v, w)
            self.last_drive_cmd_time = now_monotonic()
            self.watchdog_stopped = False

            self.publish_event({
                "type": "cmd_vel",
                "ts": time.time(),
                "v": v,
                "w": w,
                "src": "rover_uart",
            })
            return

        if cmd_type == "stop":
            self.send_stop()
            self.watchdog_stopped = True

            self.publish_event({
                "type": "cmd_vel",
                "ts": time.time(),
                "v": 0.0,
                "w": 0.0,
                "src": "rover_uart",
            })
            return

        if cmd_type == "request_imu":
            self.request_imu()
            return

        if cmd_type == "request_status":
            self.request_status()
            return

        if cmd_type == "ping":
            self.publish_event({
                "type": "pong",
                "ts": time.time(),
                "src": "rover_uart",
            })
            return

        self.publish_event({
            "type": "warning",
            "ts": time.time(),
            "src": "rover_uart",
            "warning": "unknown_command",
            "raw": cmd,
        })

    # -------------------------
    # Periodic services
    # -------------------------

    def service_imu_poll(self) -> None:
        if self.args.imu_poll_s <= 0:
            return

        now = now_monotonic()
        if (now - self.last_imu_request_time) >= self.args.imu_poll_s:
            try:
                self.request_imu()
                self.last_imu_request_time = now
            except Exception as e:
                self.publish_event({
                    "type": "uart_error",
                    "ts": time.time(),
                    "error": f"imu_poll_failed: {e}",
                })

    def service_status_poll(self) -> None:
        if self.args.status_poll_s <= 0:
            return

        now = now_monotonic()
        if (now - self.last_status_request_time) >= self.args.status_poll_s:
            try:
                self.request_status()
                self.last_status_request_time = now
            except Exception as e:
                self.publish_event({
                    "type": "uart_error",
                    "ts": time.time(),
                    "error": f"status_poll_failed: {e}",
                })

    def service_watchdog(self) -> None:
        if self.args.watchdog_s <= 0:
            return

        if self.last_drive_cmd_time <= 0:
            return

        age = now_monotonic() - self.last_drive_cmd_time
        if age > self.args.watchdog_s and not self.watchdog_stopped:
            try:
                self.send_stop()
                self.watchdog_stopped = True
                self.publish_event({
                    "type": "watchdog_stop",
                    "ts": time.time(),
                    "src": "rover_uart",
                    "age_s": age,
                })
            except Exception as e:
                self.publish_event({
                    "type": "uart_error",
                    "ts": time.time(),
                    "error": f"watchdog_stop_failed: {e}",
                })

    def build_heartbeat_summary(self) -> dict[str, Any]:
        summary: dict[str, Any] = {
            "watchdog_stopped": self.watchdog_stopped,
            "last_uart_rx_age_s": None,
        }

        if self.last_uart_rx_time > 0:
            summary["last_uart_rx_age_s"] = now_monotonic() - self.last_uart_rx_time

        if self.latest_status is not None:
            if "v" in self.latest_status:
                summary["voltage"] = self.latest_status.get("v")
            if "temp_c" in self.latest_status:
                summary["temp_c"] = self.latest_status.get("temp_c")

        if self.latest_imu is not None:
            if "yaw_deg" in self.latest_imu:
                summary["yaw_deg"] = self.latest_imu.get("yaw_deg")
            if "yaw_rate_dps" in self.latest_imu:
                summary["yaw_rate_dps"] = self.latest_imu.get("yaw_rate_dps")
            if "roll_deg" in self.latest_imu:
                summary["roll_deg"] = self.latest_imu.get("roll_deg")
            if "pitch_deg" in self.latest_imu:
                summary["pitch_deg"] = self.latest_imu.get("pitch_deg")

        return summary

    # -------------------------
    # Main loop
    # -------------------------

    def run(self) -> None:
        self.reader.start()

        self.publish_event({
            "type": "rover_uart_started",
            "ts": time.time(),
            "serial": self.args.serial,
            "baud": self.args.baud,
            "cmd_bind": f"{self.args.cmd_bind_host}:{self.args.cmd_bind_port}",
            "pub_dest": f"{self.args.pub_host}:{self.args.pub_port}",
            "imu_poll_s": self.args.imu_poll_s,
            "status_poll_s": self.args.status_poll_s,
        })

        print("Starting", flush=True)

        try:
            while not self.stop_event.is_set():
                # 1) service inbound UDP commands
                while True:
                    cmd = udp_recv_json_nonblocking(self.cmd_sock)
                    if cmd is None:
                        break
                    self.handle_udp_command(cmd)

                # 2) drain UART events from reader thread
                while True:
                    try:
                        ev = self.rx_queue.get_nowait()
                    except queue.Empty:
                        break
                    self.handle_uart_event(ev)

                # 3) periodic polling
                self.service_imu_poll()
                self.service_status_poll()

                # 4) safety watchdog / heartbeat
                self.service_watchdog()
                self.heartbeat.maybe_publish(summary=self.build_heartbeat_summary())

                time.sleep(self.args.loop_sleep_s)

        finally:
            self.shutdown()

    def shutdown(self) -> None:
        if self.stop_event.is_set():
            return

        self.stop_event.set()

        try:
            self.send_stop()
        except Exception:
            pass

        try:
            self.reader.join(timeout=1.0)
        except Exception:
            pass

        try:
            self.ser.close()
        except Exception:
            pass

        try:
            self.cmd_sock.close()
        except Exception:
            pass

        try:
            self.pub_sock.close()
        except Exception:
            pass


# -----------------------------
# CLI
# -----------------------------

def build_arg_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="UART bridge for rover ESP32 board")
    ap.add_argument("--serial", default="/dev/serial0", help="UART device")
    ap.add_argument("--baud", type=int, default=115200, help="UART baud rate")
    ap.add_argument("--serial-timeout", type=float, default=0.05, help="UART read timeout (s)")
    ap.add_argument("--serial-write-timeout", type=float, default=0.2, help="UART write timeout (s)")

    ap.add_argument("--cmd-bind-host", default="127.0.0.1", help="UDP bind host for internal commands")
    ap.add_argument("--cmd-bind-port", type=int, default=DRIVE_PORT, help="UDP bind port for internal commands")

    ap.add_argument("--pub-host", default="127.0.0.1", help="UDP publish destination host")
    ap.add_argument("--pub-port", type=int, default=EVENT_PORT, help="UDP publish destination port")

    ap.add_argument("--max-cmd", type=int, default=60, help="Max abs left/right motor command")
    ap.add_argument("--watchdog-s", type=float, default=0.35, help="Stop motors if no drive command within this many seconds")
    ap.add_argument("--loop-sleep-s", type=float, default=0.01, help="Main loop sleep interval")
    ap.add_argument("--heartbeat-s", type=float, default=10.0, help="Heartbeat publish interval")

    ap.add_argument(
        "--imu-poll-s",
        type=float,
        default=0.05,
        help="Period for automatic IMU requests over UART; <=0 disables",
    )
    ap.add_argument(
        "--status-poll-s",
        type=float,
        default=0.0,
        help="Period for automatic status requests over UART; <=0 disables",
    )

    return ap


def main() -> int:
    args = build_arg_parser().parse_args()
    bridge = RoverUartBridge(args)
    bridge.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
