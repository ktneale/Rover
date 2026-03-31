#!/usr/bin/env python3
"""
lidar_headless.py

Lightweight LDROBOT / LD06-style LiDAR process for rover integration.

Features:
- reads LiDAR packets from serial
- computes simple sector minimum distances
- publishes sector distances to controller
- optionally publishes 360-bin scan snapshots for debug / visualisation
- optionally publishes completed revolution scans for later SLAM / mapping

Coordinate convention:
- x = d * sin(a)
- y = d * cos(a)
- rover forward is +Y
- angle 0 deg = forward
- +angle = left side
- -angle / wrapped high angles = right side
"""

from __future__ import annotations

import argparse
import math
import time
from typing import Any, Optional

import numpy as np
import serial

from rover.common.ld06_driver import iter_decoded_packets, wrap_deg
from rover.common.rover_ports import CONTROLLER_PORT, EVENT_PORT
from rover.common.udp import udp_send_json


def angle_diff_deg(a: float, b: float) -> float:
    """Shortest signed angular difference a-b in [-180, 180)."""
    return (a - b + 180.0) % 360.0 - 180.0


def angle_in_sector(angle_deg: float, center_deg: float, half_width_deg: float) -> bool:
    return abs(angle_diff_deg(angle_deg, center_deg)) <= half_width_deg


class ScanBins:
    """
    Very lightweight 360-bin scan store.

    Each valid sample updates the nearest 1-degree bin.
    Bins expire after max_age_s so the scan reflects recent surroundings.
    """

    def __init__(self) -> None:
        self.dist_mm = np.full(360, np.inf, dtype=np.float32)
        self.ts = np.zeros(360, dtype=np.float64)

    def update(self, angles_deg: np.ndarray, d_mm: np.ndarray, now_mono: float) -> None:
        for ang, dist in zip(angles_deg, d_mm):
            idx = int(round(float(ang))) % 360
            self.dist_mm[idx] = float(dist)
            self.ts[idx] = now_mono

    def expire(self, now_mono: float, max_age_s: float) -> None:
        stale = (self.ts > 0.0) & ((now_mono - self.ts) > max_age_s)
        self.dist_mm[stale] = np.inf
        self.ts[stale] = 0.0

    def sector_min_mm(self, center_deg: float, half_width_deg: float, now_mono: float, max_age_s: float) -> float:
        best = math.inf
        for ang in range(360):
            if not angle_in_sector(float(ang), center_deg, half_width_deg):
                continue
            ts = float(self.ts[ang])
            if ts <= 0.0:
                continue
            if (now_mono - ts) > max_age_s:
                continue
            d = float(self.dist_mm[ang])
            if math.isfinite(d) and d < best:
                best = d
        return float(best) if math.isfinite(best) else 9999.0

    def snapshot_mm(self, now_mono: float, max_age_s: float) -> list[int]:
        out: list[int] = []
        for i in range(360):
            ts = float(self.ts[i])
            d = float(self.dist_mm[i])
            if ts <= 0.0 or (now_mono - ts) > max_age_s or not math.isfinite(d):
                out.append(0)
            else:
                out.append(int(round(d)))
        return out


class RevolutionScanBuilder:
    """
    Build one completed LiDAR revolution at a time.

    Behaviour:
    - fixed angular bins (default 360 => 1 degree)
    - keep nearest valid sample per bin within a revolution
    - finalize and publish when angle wraps from high degrees to low degrees
    """

    def __init__(
        self,
        *,
        bins: int = 360,
        min_mm: float = 80.0,
        max_mm: float = 6000.0,
        wrap_threshold_deg: float = 20.0,
    ) -> None:
        self.bins = bins
        self.min_mm = min_mm
        self.max_mm = max_mm
        self.wrap_threshold_deg = wrap_threshold_deg
        self.angle_increment_deg = 360.0 / float(bins)

        self.seq = 0
        self.last_angle_deg: Optional[float] = None
        self.scan_start_mono: Optional[float] = None

        self.ranges_mm = np.zeros(self.bins, dtype=np.int32)
        self.valid = np.zeros(self.bins, dtype=bool)

    def reset(self, now_mono: float) -> None:
        self.ranges_mm.fill(0)
        self.valid.fill(False)
        self.scan_start_mono = now_mono

    def angle_to_bin(self, angle_deg: float) -> int:
        idx = int(angle_deg / self.angle_increment_deg)
        if idx < 0:
            idx = 0
        elif idx >= self.bins:
            idx = self.bins - 1
        return idx

    def is_wrap(self, prev_deg: float, cur_deg: float) -> bool:
        return prev_deg > (360.0 - self.wrap_threshold_deg) and cur_deg < self.wrap_threshold_deg

    def add_samples(
        self,
        angles_deg: np.ndarray,
        d_mm: np.ndarray,
        now_mono: float,
    ) -> list[dict[str, Any]]:
        """
        Add a batch of samples. Returns zero or more completed scan messages.
        Usually zero or one, but list keeps things safe if weird angle jumps happen.
        """
        out: list[dict[str, Any]] = []

        if self.scan_start_mono is None:
            self.reset(now_mono)

        for ang, dist in zip(angles_deg, d_mm):
            angle_deg = float(ang) % 360.0
            distance_mm = float(dist)

            if self.last_angle_deg is not None and self.is_wrap(self.last_angle_deg, angle_deg):
                out.append(self.finalize(now_mono))
                self.reset(now_mono)

            self.last_angle_deg = angle_deg

            if not math.isfinite(distance_mm):
                continue
            if distance_mm < self.min_mm or distance_mm > self.max_mm:
                continue

            idx = self.angle_to_bin(angle_deg)
            new_val = int(round(distance_mm))

            if (not self.valid[idx]) or (new_val < int(self.ranges_mm[idx])):
                self.ranges_mm[idx] = new_val
                self.valid[idx] = True

        return out

    def finalize(self, now_mono: float) -> dict[str, Any]:
        if self.scan_start_mono is None:
            scan_time_s = 0.0
        else:
            scan_time_s = max(0.0, now_mono - self.scan_start_mono)

        msg = {
            "type": "lidar_scan",
            "seq": self.seq,
            "frame": "base_lidar",
            "ts": time.time(),
            "t_monotonic": now_mono,
            "angle_min_deg": 0.0,
            "angle_max_deg": 360.0 - self.angle_increment_deg,
            "angle_increment_deg": self.angle_increment_deg,
            "scan_time_s": scan_time_s,
            "range_min_mm": int(round(self.min_mm)),
            "range_max_mm": int(round(self.max_mm)),
            "ranges_mm": self.ranges_mm.tolist(),
            "valid": self.valid.tolist(),
        }
        self.seq += 1
        return msg


class LidarHeadless:
    def __init__(self, args: argparse.Namespace) -> None:
        self.args = args
        self.ser = serial.Serial(args.port, args.baud, timeout=args.serial_timeout_s)
        self.scan = ScanBins()

        self.rev_scan = RevolutionScanBuilder(
            bins=args.slam_scan_bins,
            min_mm=args.min_mm,
            max_mm=args.max_mm,
            wrap_threshold_deg=args.slam_scan_wrap_threshold_deg,
        )

        self.last_sector_pub = 0.0
        self.last_scan_pub = 0.0
        self.last_hb_pub = 0.0

        self.packet_count = 0
        self.point_count = 0
        self.rev_scan_pub_count = 0
        self.start_wall = time.time()
        self.last_rev_log_mono = time.monotonic()

    def compute_sector_message(self, now_mono: float) -> dict[str, Any]:
        front = self.scan.sector_min_mm(
            center_deg=self.args.front_center_deg,
            half_width_deg=self.args.front_half_width_deg,
            now_mono=now_mono,
            max_age_s=self.args.bin_max_age_s,
        )
        front_left = self.scan.sector_min_mm(
            center_deg=wrap_deg(self.args.front_center_deg - self.args.front_left_center_deg),
            half_width_deg=self.args.front_left_half_width_deg,
            now_mono=now_mono,
            max_age_s=self.args.bin_max_age_s,
        )
        front_right = self.scan.sector_min_mm(
            center_deg=wrap_deg(self.args.front_center_deg + self.args.front_right_center_deg),
            half_width_deg=self.args.front_right_half_width_deg,
            now_mono=now_mono,
            max_age_s=self.args.bin_max_age_s,
        )
        left = self.scan.sector_min_mm(
            center_deg=wrap_deg(self.args.front_center_deg - self.args.left_center_deg),
            half_width_deg=self.args.left_half_width_deg,
            now_mono=now_mono,
            max_age_s=self.args.bin_max_age_s,
        )
        right = self.scan.sector_min_mm(
            center_deg=wrap_deg(self.args.front_center_deg + self.args.right_center_deg),
            half_width_deg=self.args.right_half_width_deg,
            now_mono=now_mono,
            max_age_s=self.args.bin_max_age_s,
        )

        if self.args.debug_sectors:
            print(
                f"[LIDAR] front={front:.0f} fl={front_left:.0f} fr={front_right:.0f} "
                f"left={left:.0f} right={right:.0f}",
                flush=True,
            )

        return {
            "type": "lidar",
            "front_mm": front,
            "front_left_mm": front_left,
            "front_right_mm": front_right,
            "left_mm": left,
            "right_mm": right,
            "ts": time.time(),
        }

    def publish_sector_message(self, msg: dict[str, Any]) -> None:
        udp_send_json(self.args.controller_host, self.args.controller_port, msg)

    def publish_scan_snapshot(self, now_mono: float) -> None:
        if self.args.scan_port <= 0:
            return

        msg = {
            "type": "lidar_scan_snapshot",
            "frame": "base_lidar",
            "ranges_mm": self.scan.snapshot_mm(now_mono, self.args.bin_max_age_s),
            "ts": time.time(),
        }
        udp_send_json(self.args.scan_host, self.args.scan_port, msg)

    def publish_revolution_scan(self, msg: dict[str, Any]) -> None:
        if self.args.slam_scan_port <= 0:
            return
        udp_send_json(self.args.slam_scan_host, self.args.slam_scan_port, msg)

    def publish_heartbeat(self, sector_msg: dict[str, Any]) -> None:
        if self.args.event_port <= 0:
            return

        msg = {
            "type": "heartbeat",
            "src": "lidar_headless",
            "ts": time.time(),
            "summary": {
                "port": self.args.port,
                "baud": self.args.baud,
                "packets": self.packet_count,
                "points": self.point_count,
                "uptime_s": time.time() - self.start_wall,
                "front_mm": sector_msg["front_mm"],
                "front_left_mm": sector_msg["front_left_mm"],
                "front_right_mm": sector_msg["front_right_mm"],
                "left_mm": sector_msg["left_mm"],
                "right_mm": sector_msg["right_mm"],
                "slam_scan_enabled": self.args.slam_scan_port > 0,
                "slam_scan_bins": self.args.slam_scan_bins,
                "slam_scans_published": self.rev_scan_pub_count,
            },
        }
        udp_send_json(self.args.event_host, self.args.event_port, msg)

    def maybe_log_revolution_scan(self, msg: dict[str, Any], now_mono: float) -> None:
        if self.args.debug_revolution_scans and self.args.slam_scan_log_every > 0:
            if (self.rev_scan_pub_count % self.args.slam_scan_log_every) == 0:
                valid_count = sum(1 for v in msg["valid"] if v)
                dt = now_mono - self.last_rev_log_mono
                hz = (self.args.slam_scan_log_every / dt) if dt > 0.0 else 0.0
                self.last_rev_log_mono = now_mono

                print(
                    f"[REV_SCAN] seq={msg['seq']} valid_bins={valid_count}/{self.args.slam_scan_bins} "
                    f"scan_time={msg['scan_time_s']:.3f}s out_hz={hz:.2f}",
                    flush=True,
                )

    def run(self) -> None:
        print(
            f"[LIDAR] headless running on {self.args.port} @ {self.args.baud}, "
            f"controller={self.args.controller_host}:{self.args.controller_port}",
            flush=True,
        )

        if self.args.slam_scan_port > 0:
            print(
                f"[LIDAR] revolution scans enabled -> "
                f"{self.args.slam_scan_host}:{self.args.slam_scan_port} "
                f"(bins={self.args.slam_scan_bins})",
                flush=True,
            )

        try:
            for decoded in iter_decoded_packets(self.ser):
                self.packet_count += 1

                angles_deg = decoded.angles_deg
                d_mm = decoded.distances_mm

                if self.args.angle_offset_deg != 0.0:
                    angles_deg = np.mod(angles_deg + self.args.angle_offset_deg, 360.0)

                mask = (d_mm >= self.args.min_mm) & (d_mm <= self.args.max_mm)
                now_mono = time.monotonic()

                if not np.any(mask):
                    self.scan.expire(now_mono, self.args.bin_max_age_s)
                    continue

                valid_angles = angles_deg[mask]
                valid_dists = d_mm[mask]

                self.scan.update(valid_angles, valid_dists, now_mono)
                self.scan.expire(now_mono, self.args.bin_max_age_s)

                self.point_count += int(len(valid_dists))

                if self.args.slam_scan_port > 0:
                    completed_msgs = self.rev_scan.add_samples(valid_angles, valid_dists, now_mono)
                    for rev_msg in completed_msgs:
                        self.publish_revolution_scan(rev_msg)
                        self.rev_scan_pub_count += 1
                        self.maybe_log_revolution_scan(rev_msg, now_mono)

                if self.args.debug_packets and (self.packet_count % 50 == 0):
                    print(
                        f"[PKT] count={self.packet_count} "
                        f"start={float(angles_deg[0]):.1f} end={float(angles_deg[-1]):.1f} "
                        f"valid={len(valid_dists)}",
                        flush=True,
                    )

                if (now_mono - self.last_sector_pub) >= (1.0 / max(self.args.sector_hz, 0.1)):
                    sector_msg = self.compute_sector_message(now_mono)
                    self.publish_sector_message(sector_msg)
                    self.last_sector_pub = now_mono

                    if (now_mono - self.last_hb_pub) >= self.args.heartbeat_s:
                        self.publish_heartbeat(sector_msg)
                        self.last_hb_pub = now_mono

                if self.args.scan_port > 0 and (now_mono - self.last_scan_pub) >= (1.0 / max(self.args.scan_hz, 0.1)):
                    self.publish_scan_snapshot(now_mono)
                    self.last_scan_pub = now_mono

        except KeyboardInterrupt:
            pass
        finally:
            self.ser.close()


def build_arg_parser() -> argparse.ArgumentParser:
    ap = argparse.ArgumentParser(description="Lightweight LD06 / LDROBOT headless LiDAR publisher")

    ap.add_argument("--port", required=True, help="e.g. /dev/ttyUSB0")
    ap.add_argument("--baud", type=int, default=230400)
    ap.add_argument("--serial-timeout-s", type=float, default=0.2)

    ap.add_argument("--controller-host", default="127.0.0.1")
    ap.add_argument("--controller-port", type=int, default=CONTROLLER_PORT)

    ap.add_argument("--event-host", default="127.0.0.1")
    ap.add_argument("--event-port", type=int, default=EVENT_PORT)
    ap.add_argument("--heartbeat-s", type=float, default=1.0)

    ap.add_argument("--scan-host", default="127.0.0.1")
    ap.add_argument("--scan-port", type=int, default=0, help="0 disables debug snapshot scan publishing")
    ap.add_argument("--scan-hz", type=float, default=5.0)

    ap.add_argument("--slam-scan-host", default="127.0.0.1")
    ap.add_argument("--slam-scan-port", type=int, default=0, help="0 disables completed revolution scan publishing")
    ap.add_argument("--slam-scan-bins", type=int, default=360)
    ap.add_argument("--slam-scan-wrap-threshold-deg", type=float, default=20.0)
    ap.add_argument("--slam-scan-log-every", type=int, default=10)

    ap.add_argument("--sector-hz", type=float, default=15.0)
    ap.add_argument("--bin-max-age-s", type=float, default=0.20)

    ap.add_argument("--min_mm", type=float, default=80.0)
    ap.add_argument("--max_mm", type=float, default=6000.0)
    ap.add_argument("--angle-offset-deg", type=float, default=0.0)

    ap.add_argument("--front-center-deg", type=float, default=0.0)
    ap.add_argument("--front-half-width-deg", type=float, default=25.0)

    ap.add_argument("--front-left-center-deg", type=float, default=40.0)
    ap.add_argument("--front-left-half-width-deg", type=float, default=20.0)

    ap.add_argument("--front-right-center-deg", type=float, default=40.0)
    ap.add_argument("--front-right-half-width-deg", type=float, default=20.0)

    ap.add_argument("--left-center-deg", type=float, default=90.0)
    ap.add_argument("--left-half-width-deg", type=float, default=20.0)

    ap.add_argument("--right-center-deg", type=float, default=90.0)
    ap.add_argument("--right-half-width-deg", type=float, default=20.0)

    ap.add_argument("--debug-sectors", action="store_true")
    ap.add_argument("--debug-packets", action="store_true")
    ap.add_argument("--debug-revolution-scans", action="store_true")

    return ap


def main() -> int:
    args = build_arg_parser().parse_args()
    node = LidarHeadless(args)
    node.run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
