"""One-second downward hold activation; independent of devices and wall clocks."""
import math


class DownHoldGate:
    def __init__(self):
        self.reset()

    def reset(self):
        self.centered = False
        self.down_since = None

    def update(self, axes, now, *, released):
        x, y = axes
        if not released or not all(math.isfinite(v) for v in (x, y)):
            self.reset()
            return False
        if abs(x) < .25 and abs(y) < .25:
            self.centered = True
            self.down_since = None
        elif y < 0 and abs(x) < .35 and self.centered:
            # Allow travel from center to full down across several samples.
            # Only time spent continuously beyond the threshold counts.
            if y > -.8:
                self.down_since = None
                return False
            if self.down_since is None:
                self.down_since = now
            if now - self.down_since >= 1.0:
                self.reset()
                return True
        else:
            self.reset()
        return False
