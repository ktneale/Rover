import time
from typing import Any, Callable


class HeartbeatPublisher:
    def __init__(
        self,
        module_name: str,
        publish_fn: Callable[[dict[str, Any]], None],
        interval_s: float = 1.0,
    ) -> None:
        self.module_name = module_name
        self.publish_fn = publish_fn
        self.interval_s = interval_s
        self.start_time = time.monotonic()
        self.last_pub_time = 0.0

    def maybe_publish(self, status: str = "ok", summary: dict[str, Any] | None = None) -> None:
        now = time.monotonic()
        if now - self.last_pub_time < self.interval_s:
            return

        self.last_pub_time = now
        self.publish_fn({
            "type": "heartbeat",
            "module": self.module_name,
            "ts": time.time(),
            "uptime_s": now - self.start_time,
            "status": status,
            "summary": summary or {},
        })
