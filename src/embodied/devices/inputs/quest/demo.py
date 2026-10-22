"""Synthetic VR input for simulation only; commands still traverse native control."""

import math
import signal
import time
import numpy as np
from scipy.spatial.transform import Rotation
from embodied.contracts.control import XRFrame
from embodied.control.ipc import XRPublisher
from embodied.control.runtime import PeriodicClock


def frame_at(elapsed: float, sequence: int, now: float | None = None) -> XRFrame:
    t = elapsed % 18.0
    primary = 3.4 <= t < 3.65 or 4.0 <= t < 4.25 or 10 <= t < 10.06
    secondary = 10 <= t < 10.06
    grip = float(4.5 <= t < 8.0 or 8.5 <= t < 10.0)
    phase = max(0.0, min(t - 4.5, 3.5))
    position = np.array([.02 * math.sin(phase * 1.3), .03 * math.sin(phase), .025 * (1 - math.cos(phase))])
    rotation = Rotation.from_euler("z", .18 * math.sin(phase)).as_quat()
    if t >= 8:
        position += np.array([.2, .15, .1])  # Hand reposition while clutch is released.
    if t >= 8.5:
        position[1] += .01 * math.sin((t - 8.5) * 2)
    trigger = .5 + .45 * math.sin((t - 4.5) * 2) if 4.5 <= t < 10 else 0.0
    return XRFrame(sequence=sequence, sdk_timestamp_ns=int(elapsed * 1e9),
                   received_at=time.monotonic() if now is None else now,
                   pose=np.concatenate((position, rotation)), trigger=trigger, grip=grip,
                   primary=primary, secondary=secondary)


def run(prefix: str):
    stopped = False
    def stop(*_):
        nonlocal stopped
        stopped = True
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)
    publisher = XRPublisher(prefix)
    clock = PeriodicClock(90)
    started = time.monotonic()
    sequence = 0
    try:
        while not stopped:
            sequence += 1
            now = time.monotonic()
            publisher.publish(frame_at(now - started, sequence, now))
            clock.wait()
    finally:
        publisher.close()
