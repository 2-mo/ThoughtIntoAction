"""Native command ingress: reuse arx bridge, route health, permit and mode ACKs.

This worker-local adapter never starts a process or implements pose mapping/IK.
"""
import math
import time
from arx5_vr.ipc import RobotBridge
from arx5_vr.ipc.services import create_node, open_service, make_subscriber, drain_latest
from arx5_vr.ipc.types import XrControllerInput
from arx5_vr.types import ControlCommand, Mode
from .ingress import Candidate
from .native_health import DependencyHealth, TeleopGate, TeleopRouteHealth


class FreshSample:
    """Only increasing sequences and fresh producer stamps renew health."""
    def __init__(self, timeout, monotonic=False):
        self.timeout, self.monotonic = timeout, monotonic
        self.sequence = -1
        self.sample = None
        self.expires_at = float('-inf')

    def update(self, sample, now, wall):
        if sample is None or sample.sequence <= self.sequence:
            return False
        stamp = sample.monotonic_ns / 1e9 if self.monotonic else sample.timestamp_us / 1e6
        age = (now if self.monotonic else wall) - stamp
        if not 0 <= age < self.timeout:
            return False
        self.sequence, self.sample = sample.sequence, sample
        self.expires_at = now + self.timeout - age
        return True

    def fresh(self, now):
        return self.sample is not None and now < self.expires_at


class NativeIngress:
    def __init__(self, prefix, dependencies, gripper=True):
        self.bridge = RobotBridge(prefix, feedback=False)
        self.node = create_node()
        topics = {
            'input': (f'{prefix}/xr/right_controller/states/input', XrControllerInput, .1),
            'permit': (f'{prefix}/group/permit', TeleopGate, .1),
            'arm': (f'{prefix}/group/arm/health', TeleopRouteHealth, .5),
        }
        if gripper:
            topics['gripper'] = (f'{prefix}/group/gripper/health', TeleopRouteHealth, .5)
        topics['dependencies'] = (f'{prefix}/managed/dependencies', DependencyHealth, .2)
        self.subscribers = {key: make_subscriber(open_service(self.node, topic, kind))
                            for key, (topic, kind, _) in topics.items()}
        self.samples = {key: FreshSample(timeout, key == 'dependencies')
                        for key, (_, _, timeout) in topics.items()}
        self.dependencies = tuple(dependencies)
        self.epoch = None
        self.released = self.armed = self.was_ready = self.home_requested = False
        self.selected_since = 0.
        self.home_deadline = 0.
        self.home_active = False
        self.status = {'ready': False, 'reason': 'waiting for native components'}

    def poll(self, state, now, epoch, selected):
        native = self.bridge.poll_commands(now)  # Always drain, even while another owner is active.
        raw = {key: drain_latest(subscriber) for key, subscriber in self.subscribers.items()}
        now, wall = time.monotonic(), time.time()
        changed = {key: tracker.update(raw[key], now, wall)
                   for key, tracker in self.samples.items()}
        if epoch != self.epoch or not selected:
            self.epoch = epoch
            self.released = self.armed = self.was_ready = self.home_requested = False
            self.selected_since = now
            self.home_deadline = 0.
            self.home_active = False
        inp, permit, process = (self.samples[key] for key in ('input', 'permit', 'dependencies'))
        fatal = None
        if process.fresh(now) and process.sample.failed_mask:
            failed = [name for i, name in enumerate(self.dependencies) if process.sample.failed_mask & (1 << i)]
            fatal = 'native component exited: ' + ', '.join(failed)
        elif not process.fresh(now) and selected and now - self.selected_since > .2:
            fatal = 'native supervisor heartbeat expired'
        reason = fatal
        for name, tracker in self.samples.items():
            if not tracker.fresh(now):
                reason = reason or f'native {name} stale'
            elif name in ('arm', 'gripper') and tracker.sample.state == 2:
                reason = reason or f'native {name} route fault'
        valid = (inp.fresh(now) and inp.sample.connection_state == 1 and inp.sample.tracking_valid
                 and math.isfinite(inp.sample.grip))
        if not valid:
            reason = reason or 'native tracking invalid'
        ready = reason is None
        # Process death always withdraws ownership. Native health loss after readiness
        # also withdraws it; a disconnected headset at startup can still be connected later.
        if selected and self.was_ready and not ready:
            fatal = fatal or reason
        if ready and selected:
            self.was_ready = True
        self.status = {'ready': ready, 'reason': reason, 'armed': self.armed,
                       'permit': bool(permit.fresh(now) and permit.sample.filter_valid)}
        if not ready or not selected:
            self.released = self.armed = self.home_requested = False
            self.home_deadline = 0.
            self.home_active = False
        elif changed['input']:
            sample = inp.sample
            idle = not (sample.buttons_bitset & 12) and sample.grip < .2
            if idle:
                self.released = True
            elif self.released:
                self.armed = True
            if self.armed and sample.buttons_bitset & 12 == 12:
                self.home_deadline = now + .5
        if native.mode == Mode.DISABLED and now < self.home_deadline:
            self.home_active = True
        elif native.mode != Mode.DISABLED:
            self.home_active = False
        self.home_requested = self.home_active or now < self.home_deadline
        command = ControlCommand(Mode.COMMAND_FOLLOWING, hold=True, gripper_target=state.gripper_width)
        if selected and self.armed and ready:
            # An inactive coordinator may have requested Disabled after a fault.
            # Only an operator A+B in this ownership epoch can start native Home.
            if native.mode != Mode.DISABLED or self.home_requested:
                command = native
                if native.mode == Mode.COMMAND_FOLLOWING and not permit.sample.filter_valid:
                    command = ControlCommand(Mode.COMMAND_FOLLOWING, hold=True,
                                             gripper_target=state.gripper_width)
        phase = 'teleop' if command.mode == Mode.COMMAND_FOLLOWING and not command.hold else 'hold'
        return Candidate(epoch, command, min(t.expires_at for t in self.samples.values()),
                         inp.sequence, phase, fatal)

    def close(self):
        self.bridge.close()
        self.subscribers.clear()
        self.node = None
