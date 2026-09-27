"""Camera process stand-ins (module level, so a spawned process can import them)."""

from time import monotonic


def good_camera(conn, stop):
    conn.send(("ready",))
    sequence = 0
    while not stop.is_set():
        sequence += 1
        now = monotonic()
        conn.send(("scene", sequence, now, now - 0.3, [("glass-1", 0.4, -0.3)]))
        conn.send(("frame", sequence, now, b"\xff\xd8jpeg"))
        stop.wait(0.05)
    conn.close()


def broken_camera(conn, stop):
    conn.send(("error", "RuntimeError: Could not open camera 2"))
    conn.close()


def short_lived_camera(conn, stop):
    now = monotonic()
    conn.send(("ready",))
    conn.send(("scene", 1, now, now - 0.3, []))
    conn.close()   # the process ends: the supervisor side must notice
