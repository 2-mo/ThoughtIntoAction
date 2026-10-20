"""Deterministic RGB source for camera/collection plugin acceptance without a device."""
import time
import uuid


class SyntheticCamera:
    def __init__(self, serial='synthetic', width=64, height=48, fps=30):
        if not 1 <= fps <= 120 or width < 1 or height < 1:
            raise ValueError('Invalid synthetic camera dimensions/rate')
        self.options = dict(serial=serial, width=width, height=height, fps=fps)

    def __enter__(self):
        self.metadata = {**self.options, 'stream_id': uuid.uuid4().hex, 'model': 'synthetic',
                         'format': 'bgr8', 'frame': 'color_optical'}
        self.number, self.deadline = 0, time.monotonic()
        return self

    def read(self):
        import numpy as np
        self.deadline += 1 / self.options['fps']
        time.sleep(max(0, self.deadline - time.monotonic()))
        self.number += 1
        return {'stream_id': self.metadata['stream_id'], 'frame_number': self.number,
                'host_received_monotonic_ns': time.monotonic_ns(),
                'image': np.full((self.options['height'], self.options['width'], 3), self.number % 255, np.uint8)}

    def __exit__(self, *exc):
        pass
