import json
import socket
from typing import Any, Optional


def udp_send_json(sock: socket.socket, host: str, port: int, obj: dict[str, Any]) -> None:
    payload = json.dumps(obj, separators=(",", ":")).encode("utf-8")
    sock.sendto(payload, (host, port))


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
