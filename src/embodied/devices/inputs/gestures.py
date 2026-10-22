"""Debounced controller presses and stick flicks; no device, clock or robot dependency."""
import math


class Press:
    def __init__(self, long_s=None, debounce_s=.03):
        self.long_s, self.debounce_s = long_s, debounce_s
        self.raw = self.stable = False
        self.changed = 0.
        self.started = None
        self.fired = False
        self.armed = False
        self.allowed = True

    def update(self, pressed, now, enabled=True):
        if not pressed and self.started is None:
            self.armed = True
        if pressed != self.raw:
            self.raw, self.changed = pressed, now
            if pressed:
                self.allowed = enabled
        if self.raw != self.stable and now - self.changed >= self.debounce_s:
            self.stable = self.raw
            if self.stable:
                if self.armed and self.allowed:
                    self.started, self.fired = self.changed, False
            elif self.started is not None:
                duration = self.changed - self.started
                self.started = None
                if not self.fired:
                    return "long" if self.long_s is not None and duration >= self.long_s else "short"
        if self.stable and self.raw and self.started is not None and not self.fired:
            if self.long_s is not None and now - self.started >= self.long_s:
                self.fired = True
                return "long"
        return None


class StickAction:
    """Trigger on directional entry once; neutral rearms, without a hold timer."""
    def __init__(self):
        self.armed = False

    def update(self, axes, now, enabled=True):
        x, y = axes
        if not all(math.isfinite(v) for v in (x, y)):
            self.armed = False
            return None
        if abs(x) < .25 and abs(y) < .25:
            self.armed = True
            return None
        if not enabled:
            self.armed = False
            return None
        direction = ("start_task" if x < 0 else "pause_task") if abs(x) >= .8 and abs(y) < .35 else None
        if y >= .8 and abs(x) < .35:
            direction = "discard_previous"
        if self.armed and (abs(x) >= .8 or abs(y) >= .8):
            self.armed = False
            return direction
        return None
