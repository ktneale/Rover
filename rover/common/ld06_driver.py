#!/usr/bin/env python3
"""
rover/common/ld06_driver.py

Shared LD06 / LDROBOT LiDAR packet parsing helpers.

This module provides:
- raw packet framing from serial
- packet decode into interpolated sample angles
- a simple sample iterator for downstream users

Coordinate / angle convention:
- 0 deg = forward
- +deg = left
- wrapped high angles = right
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Generator, Iterable, Optional

import numpy as np
import serial


PACKET_LEN = 47
HEADER = b"\x54\x2C"
POINTS_PER_PKT = 12


@dataclass(frozen=True)
class Ld06Packet:
    angles_deg: np.ndarray       # shape (12,)
    distances_mm: np.ndarray     # shape (12,)
    end_angle_deg: float         # wrapped to [0, 360)


def wrap_deg(a: float) -> float:
    a %= 360.0
    return a + 360.0 if a < 0 else a


def iter_packets(stream: serial.Serial) -> Generator[bytes, None, None]:
    """Yield aligned 47-byte packets starting with 0x54 0x2C."""
    buf = bytearray()

    while True:
        chunk = stream.read(512)
        if chunk:
            buf.extend(chunk)

        if len(buf) > 8192:
            buf = buf[-8192:]

        while True:
            idx = buf.find(HEADER)
            if idx < 0:
                if len(buf) > 1:
                    buf = buf[-1:]
                break

            if len(buf) < idx + PACKET_LEN:
                if idx > 0:
                    del buf[:idx]
                break

            pkt = bytes(buf[idx: idx + PACKET_LEN])
            del buf[: idx + PACKET_LEN]
            yield pkt


def decode_packet(pkt: bytes) -> Optional[Ld06Packet]:
    """
    Decode one LD06 packet.

    Returns:
        Ld06Packet or None if framing is invalid.
    """
    if len(pkt) != PACKET_LEN or pkt[0:2] != HEADER:
        return None

    start_angle = int.from_bytes(pkt[4:6], "little") / 100.0

    dists = []
    off = 6
    for _ in range(POINTS_PER_PKT):
        dist_mm = int.from_bytes(pkt[off: off + 2], "little")
        dists.append(dist_mm)
        off += 3  # skip confidence byte

    end_angle = int.from_bytes(pkt[off: off + 2], "little") / 100.0

    sa = wrap_deg(start_angle)
    ea = wrap_deg(end_angle)
    if ea < sa:
        ea += 360.0

    angles = np.linspace(sa, ea, POINTS_PER_PKT, endpoint=True)
    angles = np.mod(angles, 360.0)

    return Ld06Packet(
        angles_deg=angles.astype(np.float32),
        distances_mm=np.array(dists, dtype=np.float32),
        end_angle_deg=float(np.mod(end_angle, 360.0)),
    )


def iter_decoded_packets(stream: serial.Serial) -> Generator[Ld06Packet, None, None]:
    """Yield decoded LD06 packets from a serial stream."""
    for pkt in iter_packets(stream):
        decoded = decode_packet(pkt)
        if decoded is not None:
            yield decoded


def iter_samples(
    stream: serial.Serial,
    *,
    min_mm: float = 0.0,
    max_mm: float = math.inf,
    angle_offset_deg: float = 0.0,
) -> Generator[tuple[float, float], None, None]:
    """
    Yield individual valid samples as:
        (angle_deg, distance_mm)

    Useful for future scan builders or visualisers.
    """
    for decoded in iter_decoded_packets(stream):
        angles_deg = decoded.angles_deg
        d_mm = decoded.distances_mm

        if angle_offset_deg != 0.0:
            angles_deg = np.mod(angles_deg + angle_offset_deg, 360.0)

        mask = (d_mm >= min_mm) & (d_mm <= max_mm)
        if not np.any(mask):
            continue

        for ang, dist in zip(angles_deg[mask], d_mm[mask]):
            yield float(ang), float(dist)
