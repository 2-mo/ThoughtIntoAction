"""Pose-input factories. Importing this module never initializes the Quest SDK."""

import time


class InputFreshness:
    """Track device timestamp progress, not whether the user's pose is stationary."""

    def __init__(self, timestamp, *, clock=time.monotonic, timeout=.5, startup_timeout=10.):
        self.timestamp, self.clock = timestamp, clock
        self.timeout, self.startup_timeout = timeout, startup_timeout
        self.started = self.progress = clock()
        self.last = None
        self.advanced = False

    def ready(self):
        now, stamp = self.clock(), self.timestamp()
        if not isinstance(stamp, int) or stamp < 0:
            raise RuntimeError("Invalid Quest input timestamp; control stopped")
        if self.last is None:
            if stamp > 0:
                # Require a second, advancing timestamp: SDK may retain an old frame.
                self.last, self.progress = stamp, now
            elif now - self.started > self.startup_timeout:
                raise RuntimeError("No Quest frames received; check headset wake state and XR connection")
            return False
        if stamp < self.last:
            raise RuntimeError("Quest timestamp reset; restart control after reconnecting")
        if stamp > self.last:
            self.last, self.progress = stamp, now
            self.advanced = True
        elif now - self.progress > self.timeout:
            raise RuntimeError("Quest input stopped updating for 0.5 s; control stopped. Check headset sleep and connection")
        return self.advanced


def quest():
    from xrobotoolkit_teleop.common.xr_client import XrClient
    return XrClient()


class IdleInput:
    """Unpressed controls and a fixed pose, for offline simulation only."""

    def get_pose_by_name(self, name):
        return [0., 0., 0., 0., 0., 0., 1.]

    def get_key_value_by_name(self, name):
        return 0.

    def get_button_state_by_name(self, name):
        return False

    def get_joystick_state(self, name):
        return [0., 0.]

    def get_motion_tracker_data(self):
        return {}

    def close(self):
        pass


def controller_snapshot(client, now):
    """SDK-global frame health, not a per-controller tracking/battery claim."""
    import math
    try:
        hands = {}
        for side, buttons in (("left", ("X", "Y")), ("right", ("A", "B"))):
            pose = [float(v) for v in client.get_pose_by_name(side + "_controller")]
            trigger = float(client.get_key_value_by_name(side + "_trigger"))
            grip = float(client.get_key_value_by_name(side + "_grip"))
            if len(pose) != 7 or not all(math.isfinite(v) for v in [*pose, trigger, grip]):
                raise ValueError("Invalid controller values")
            hands[side] = {"pose": pose, "trigger": trigger, "grip": grip,
                           "buttons": {**{b: bool(client.get_button_state_by_name(b)) for b in buttons},
                                       **({"R3": bool(client.get_button_state_by_name("right_axis_click"))} if side == "right" else {})}}
        return {"sample_monotonic_ns": now, "device_timestamp_ns": client.get_timestamp_ns(),
                "hands": hands, "error": None}
    except Exception as exc:
        return {"sample_monotonic_ns": now, "hands": {}, "error": str(exc)}
