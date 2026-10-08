"""Workbench buttons around the upstream native pose/IK/PD pipeline.

Pose messages pass untouched. Only the coordinator's button/analog input is
filtered while disarmed; physical A/B remain available to episode controls.
"""
import argparse
import ctypes
from dataclasses import replace
import json
from pathlib import Path
import signal
import sys
import time

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / 'src'))
from quest_teleop.arming import DownHoldGate
from arx5_vr.ipc.services import create_node, open_service, make_subscriber, make_publisher, drain_latest, send
from arx5_vr.ipc.types import XrControllerInput, Pose7
from arx5_vr.types import ControlCommand, Mode
from .composition import NativeRouting, QuestDevice as NativeQuestDevice
from .native_health import TeleopGate
from .sources import NativeIngress


def input_topic(prefix):
    return f'{prefix}/workbench/input'


class QuestDevice(NativeQuestDevice):
    @staticmethod
    def commands(configs, prefix):
        path = configs['xrobot']
        text = Path(path).read_text()
        if '"left_controller"' not in text:
            channel = text.split('[[channels]]', 1)[1]
            text += '\n[[channels]]' + channel.replace('right_controller', 'left_controller').replace('PICO Right Controller', 'Quest Left Controller')
            Path(path).write_text(text)
        return NativeQuestDevice.commands(configs, prefix)


class Routing(NativeRouting):
    @staticmethod
    def commands(configs, prefix):
        path = configs['coordinator']
        raw = f'{prefix}/xr/right_controller/states/input'
        Path(path).write_text(Path(path).read_text().replace(raw, input_topic(prefix)))
        commands = NativeRouting.commands(configs, prefix)
        commands['workbench-input'] = [sys.executable, '-m', 'adapters.workbench',
                                      '--prefix', prefix, '--directory', str(Path(path).parent)]
        return commands


class Ingress(NativeIngress):
    def __init__(self, prefix, dependencies, gripper=True):
        super().__init__(prefix, dependencies, gripper)
        self.subscribers['input'] = make_subscriber(open_service(self.node, input_topic(prefix), XrControllerInput))

    def poll(self, state, now, epoch, selected):
        candidate = super().poll(state, now, epoch, selected)
        sample = self.samples['input'].sample
        enabled = bool(sample and sample.reserved[0])
        self.status['control_enabled'] = enabled
        if not enabled:
            return replace(candidate, command=ControlCommand(Mode.COMMAND_FOLLOWING, hold=True,
                           gripper_target=state.gripper_width), phase='hold')
        return candidate


class Buttons:
    def __init__(self):
        self.gate = DownHoldGate()
        self.enabled = False
        self.active = False
        self.pulse_until = 0.

    def reset(self, active=False):
        self.active = active
        self.enabled = False
        self.pulse_until = 0.
        self.gate.reset()

    def update(self, sample, now, *, valid, following):
        if not valid or not self.active:
            self.enabled = False
            self.gate.reset()
            self.pulse_until = 0.
        else:
            released = sample.buttons_bitset == 0 and sample.grip < .1 and sample.trigger < .1
            if self.gate.update((sample.joystick_x, sample.joystick_y), now, released=released):
                self.enabled = not self.enabled
                if self.enabled and not following:
                    self.pulse_until = now + .22
        result = XrControllerInput.from_buffer_copy(bytes(sample))
        result.buttons_bitset = 4 if self.enabled and now < self.pulse_until else 0
        result.reserved[0] = int(self.enabled)
        if not self.enabled:
            result.grip = result.trigger = 0.
        return result


def hand(sample, pose, wall):
    if sample is None or pose is None or not sample.tracking_valid or sample.connection_state != 1:
        return None
    if not 0 <= wall - sample.timestamp_us / 1e6 < .15 or not 0 <= wall - pose.timestamp_us / 1e6 < .15:
        return None
    return {'pose': list(pose.values), 'trigger': sample.trigger, 'grip': sample.grip,
            'axes': [sample.joystick_x, sample.joystick_y],
            'buttons': {k: bool(sample.buttons_bitset & v) for k, v in
                        [('A', 4), ('B', 8), ('X', 1), ('Y', 2), ('R3', 16)]}}


def run(prefix, directory):
    stopped = False
    def stop(*_):
        nonlocal stopped
        stopped = True
    for sig in (signal.SIGTERM, signal.SIGINT):
        signal.signal(sig, stop)
    node = create_node()
    inputs = {side: make_subscriber(open_service(node, f'{prefix}/xr/{side}_controller/states/input', XrControllerInput))
              for side in ('left', 'right')}
    poses = {side: make_subscriber(open_service(node, f'{prefix}/xr/{side}_controller/states/pose', Pose7))
             for side in inputs}
    permits = make_subscriber(open_service(node, f'{prefix}/group/permit', TeleopGate))
    output = make_publisher(open_service(node, input_topic(prefix), XrControllerInput))
    buttons = Buttons()
    sequence = -1
    samples, positions = {}, {}
    permit = None
    revision = None
    displayed = 0.
    directory = Path(directory)
    while not stopped:
        now, wall = time.monotonic(), time.time()
        for side in inputs:
            samples[side] = drain_latest(inputs[side]) or samples.get(side)
            positions[side] = drain_latest(poses[side]) or positions.get(side)
        permit = drain_latest(permits) or permit
        try:
            request = json.loads((directory / 'workbench-request.json').read_text())
            if request['revision'] != revision:
                revision = request['revision']
                buttons.reset(bool(request['active']))
                sample = samples.get('right')
                if request.get('restore_enabled') and sample and sample.tracking_valid and sample.connection_state == 1 and 0 <= wall - sample.timestamp_us/1e6 < .1 and sample.buttons_bitset == 0 and sample.grip < .1 and sample.trigger < .1:
                    buttons.enabled = buttons.active
        except FileNotFoundError:
            pass
        sample = samples.get('right')
        fresh = sample is not None and 0 <= wall - sample.timestamp_us / 1e6 < .1
        valid = fresh and sample.connection_state == 1 and bool(sample.tracking_valid)
        if not valid:
            buttons.reset(buttons.active)
        if fresh and sample.sequence > sequence:
            sequence = sample.sequence
            following = permit and 0 <= wall - permit.timestamp_us / 1e6 < .1 and permit.filter_valid
            send(output, buttons.update(sample, now, valid=valid, following=following))
        if now - displayed >= .05:
            status = {'sample_monotonic_ns': time.monotonic_ns(), 'revision': revision,
                      'active': buttons.active, 'control_enabled': buttons.enabled, 'tracking': bool(valid),
                      'device_timestamp_ns': int(sample.timestamp_us * 1000) if sample else 0,
                      'hands': {s: hand(samples.get(s), positions.get(s), wall) for s in inputs},
                      'down_progress': min(1., now - buttons.gate.down_since) if buttons.gate.down_since else 0.}
            temporary = directory / 'workbench-input.tmp'
            temporary.write_text(json.dumps(status))
            temporary.replace(directory / 'workbench-input.json')
            displayed = now
        time.sleep(.002)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', required=True)
    parser.add_argument('--directory', required=True)
    args = parser.parse_args()
    run(args.prefix, args.directory)
