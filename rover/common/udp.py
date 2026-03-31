#!/usr/bin/env python3
import json
import socket
from typing import Any, Optional


def make_udp_rx_socket(host: str, port: int, nonblocking: bool = False) -> socket.socket:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((host, port))
    if nonblocking:
        sock.setblocking(False)
    return sock


def udp_send_json(host: str, port: int, obj: Any) -> None:
    data = json.dumps(obj, separators=(",", ":")).encode("utf-8")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sock.sendto(data, (host, port))
    finally:
        sock.close()


def udp_recv_json(sock: socket.socket, bufsize: int = 65535) -> Optional[Any]:
    data, _addr = sock.recvfrom(bufsize)
    return json.loads(data.decode("utf-8"))


def udp_recv_json_nonblocking(sock: socket.socket, bufsize: int = 65535) -> Optional[Any]:
    try:
        data, _addr = sock.recvfrom(bufsize)
    except BlockingIOError:
        return None
    return json.loads(data.decode("utf-8"))
