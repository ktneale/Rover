#!/usr/bin/env python3
import asyncio
import json
import socket
import time
import websockets

WS_HOST = "0.0.0.0"
WS_PORT = 8765

UDP_HOST = "127.0.0.1"
UDP_PORT = 5010

TELEM_HOST = "127.0.0.1"
TELEM_PORT = 8012

DEADMAN_SEC = 0.4

udp_cmd = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

udp_telem = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
udp_telem.bind((TELEM_HOST, TELEM_PORT))
udp_telem.setblocking(False)

active_ws = set()
last_rx = 0.0
stop_sent = False


def send_udp(obj):
    msg = json.dumps(obj, separators=(",", ":")).encode("utf-8")
    udp_cmd.sendto(msg, (UDP_HOST, UDP_PORT))
    print("UDP CMD ->", msg.decode(), flush=True)


def send_stop():
    send_udp({"type": "teleop_cmd", "v": 0, "w": 0})


def clamp(x, lo, hi):
    return lo if x < lo else hi if x > hi else x


async def broadcast_text(text: str):
    global active_ws

    if not active_ws:
        return

    dead = []
    for ws in active_ws:
        try:
            await ws.send(text)
        except Exception:
            dead.append(ws)

    for ws in dead:
        active_ws.discard(ws)


async def telemetry_forwarder():
    loop = asyncio.get_running_loop()

    while True:
        try:
            data, addr = await loop.sock_recvfrom(udp_telem, 65535)
        except Exception as e:
            print(f"[TELEM] recv error: {e}", flush=True)
            await asyncio.sleep(0.05)
            continue

        try:
            text = data.decode("utf-8")
            json.loads(text)  # sanity check only
        except Exception:
            print(f"[TELEM] bad JSON from {addr}: {data!r}", flush=True)
            continue

        print(f"[TELEM] {text}", flush=True)
        await broadcast_text(text)


async def watchdog():
    global stop_sent
    while True:
        await asyncio.sleep(0.05)

        if not active_ws:
            continue

        if time.monotonic() - last_rx > DEADMAN_SEC and not stop_sent:
            send_stop()
            stop_sent = True


async def handler(ws):
    global last_rx, stop_sent

    active_ws.add(ws)
    last_rx = time.monotonic()
    stop_sent = False

    peer = getattr(ws, "remote_address", None)
    print(f"client connected: {peer}  total={len(active_ws)}", flush=True)

    try:
        async for text in ws:
            try:
                msg = json.loads(text)
            except json.JSONDecodeError:
                continue

            if msg.get("type") == "odom_reset":
                send_udp(msg)

            elif msg.get("type") == "teleop_cmd":

                v = clamp(float(msg.get("v", 0)), -1.0, 1.0)
                w = clamp(float(msg.get("w", 0)), -1.0, 1.0)

                out = {"type": "teleop_cmd", "v": v, "w": w}
                send_udp(out)
                last_rx = time.monotonic()
                stop_sent = False

            else:
                continue

    finally:
        active_ws.discard(ws)
        print(f"client disconnected: {peer}  total={len(active_ws)}", flush=True)

        if not active_ws:
            send_stop()


async def main():
    async with websockets.serve(handler, WS_HOST, WS_PORT):
        print(
            f"WS bridge on ws://{WS_HOST}:{WS_PORT} -> UDP CMD {UDP_HOST}:{UDP_PORT}, "
            f"UDP TELEM {TELEM_HOST}:{TELEM_PORT}",
            flush=True,
        )

        asyncio.create_task(telemetry_forwarder())
        await watchdog()


asyncio.run(main())
