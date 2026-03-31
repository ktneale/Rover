while True:
    frame = read_camera()
    result = track_red(frame)

    send_udp({
        "type": "vision.track",
        "ts": time.time(),
        "seen": result.seen,
        "centered": result.centered,
        "err_norm": result.err_norm,
        "turn": result.turn,
        "area": result.area,
    })

    render_overlay(frame, result)
