"""Read-only quality derived from source timestamps, never browser polling rates."""
from collections import deque
import math
import statistics
import time


def finite(value):
    return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)


def vector(value):
    return isinstance(value, (list, tuple)) and bool(value) and all(finite(v) for v in value)


class MotionQuality:
    def __init__(self):
        self.reset()

    def reset(self):
        self.previous = None
        self.periods = deque(maxlen=256)
        self.skipped = 0
        self.latest = None

    def accept(self, tick):
        if self.previous and self.previous.get('runtime_id') != tick.get('runtime_id'):
            self.reset()
        timing = tick.get('timing') or {}
        if self.previous:
            delta = tick['sequence'] - self.previous['sequence']
            previous_ns = (self.previous.get('timing') or {}).get('tick_ns')
            stamp = timing.get('tick_ns')
            if delta > 1:
                self.skipped += delta - 1
            # A latest-only observer can skip samples; never call that a slow control cycle.
            if delta == 1 and finite(stamp) and finite(previous_ns) and stamp > previous_ns:
                self.periods.append((stamp - previous_ns) / 1e6)
        self.previous = self.latest = tick

    def snapshot(self, now_ns=None):
        now_ns = time.monotonic_ns() if now_ns is None else now_ns
        tick = self.latest or {}
        feedback, command = tick.get('feedback') or {}, tick.get('command') or {}
        timing = tick.get('timing') or {}
        received, submitted = timing.get('feedback_received_ns'), timing.get('command_submitted_ns')
        age = (now_ns - received) / 1e9 if finite(received) and received > 0 else None
        valid = age is not None and 0 <= age < .5 and not tick.get('error') and not tick.get('terminal')
        q, dq, target = feedback.get('q'), feedback.get('dq'), command.get('q')
        paired = valid and vector(q) and vector(target) and len(q) == len(target)
        periods = list(self.periods)
        return {
            'valid': bool(valid and vector(q)), 'feedback_age_s': age,
            'joint_velocity_rad_s': dq if valid and vector(dq) else None,
            'peak_joint_speed_rad_s': max(map(abs, dq)) if valid and vector(dq) else None,
            'target_feedback_error_rad': max(abs(a - b) for a, b in zip(target, q)) if paired else None,
            'feedback_monotonic_ns': received, 'command_monotonic_ns': submitted,
            'command_after_feedback_ms': (submitted - received) / 1e6
                if finite(received) and finite(submitted) and submitted > 0 else None,
            'cycle_samples': len(periods),
            'cycle_mean_ms': statistics.mean(periods) if valid and periods else None,
            'cycle_std_ms': statistics.pstdev(periods) if valid and len(periods) >= 2 else None,
            'cycle_max_ms': max(periods) if valid and periods else None,
            'monitor_skipped_ticks': self.skipped,
            'producer_dropped': tick.get('producer_dropped'), 'observer_gaps': tick.get('observer_gaps'),
            'operation_id': tick.get('operation_id'),
        }


def camera_quality(payload):
    """Compare host receipt times only. Preview skips are not camera source losses."""
    feedback_ns = (payload.get('motion_quality') or {}).get('feedback_monotonic_ns')
    result = {}
    for camera_id, feed in (payload.get('cameras') or {}).items():
        camera = feed.get('camera') or {}
        stamp, age = camera.get('host_received_monotonic_ns'), camera.get('age_s')
        valid = (not payload.get('camera_error') and not camera.get('error') and camera.get('enabled') is True
                 and finite(age) and 0 <= age < .5)
        result[camera_id] = {
            'valid': bool(valid), 'frame_age_s': age if finite(age) else None,
            'preview_age_s': (feed.get('preview') or {}).get('age_s'),
            'host_received_monotonic_ns': stamp, 'frame_number': camera.get('frame_number'),
            'source_missing_frames': camera.get('source_missing_frames'),
            'robot_receipt_delta_ms': (stamp - feedback_ns) / 1e6
                if valid and finite(stamp) and finite(feedback_ns) and (payload.get('motion_quality') or {}).get('valid') else None,
        }
    return result
