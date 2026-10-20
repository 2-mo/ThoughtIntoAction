"""Workbench allow/pause around the upstream native pose/IK/PD pipeline.

Pose messages pass untouched. While the console allows teleoperation, the right
controller reaches the coordinator unchanged, so A / A+B / Grip / Trigger keep the
upstream arx-televr meaning. Collection uses only the left controller.
"""
import argparse
import ctypes
from dataclasses import replace
import json
from pathlib import Path
import signal
import sys
import time

from arx5_vr.ipc.services import create_node, open_service, make_subscriber, make_publisher, drain_latest, send
from arx5_vr.ipc.types import XrControllerInput, Pose7
from arx5_vr.types import ControlCommand, Mode
from .composition import NativeRouting, QuestDevice as NativeQuestDevice
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
    """Console allow/pause gate; never synthesizes or remaps upstream buttons.

    Paused or newly allowed input reads as released until the operator lets go of
    every button, Grip and Trigger, so a held Grip never carries over into following.
    """
    def __init__(self):
        self.enabled = False
        self.active = False

    def reset(self, active=False):
        self.active = active
        self.enabled = False

    def update(self, sample, *, valid):
        released = sample.buttons_bitset == 0 and sample.grip < .1 and sample.trigger < .1
        if not valid or not self.active:
            self.enabled = False
        elif released:
            self.enabled = True
        result = XrControllerInput.from_buffer_copy(bytes(sample))
        result.reserved[0] = int(self.enabled)
        if not self.enabled:
            result.buttons_bitset = 0
            result.grip = result.trigger = 0.
        return result


# Per-controller bits from rollio-device-xrobot (rollio-types XR_BUTTON_*).
BUTTONS = {'right': (('A', 4), ('B', 8), ('R3', 2), ('menu', 1)),
           'left': (('X', 4), ('Y', 8), ('L3', 2), ('menu', 1))}


def hand(side, sample, pose, wall):
    if sample is None or pose is None or not sample.tracking_valid or sample.connection_state != 1:
        return None
    if not 0 <= wall - sample.timestamp_us / 1e6 < .15 or not 0 <= wall - pose.timestamp_us / 1e6 < .15:
        return None
    return {'pose': list(pose.values), 'trigger': sample.trigger, 'grip': sample.grip,
            'axes': [sample.joystick_x, sample.joystick_y],
            'buttons': {k: bool(sample.buttons_bitset & v) for k, v in BUTTONS[side]}}


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
    output = make_publisher(open_service(node, input_topic(prefix), XrControllerInput))
    buttons = Buttons()
    sequence = -1
    samples, positions = {}, {}
    revision = None
    displayed = 0.
    directory = Path(directory)
    while not stopped:
        now, wall = time.monotonic(), time.time()
        for side in inputs:
            samples[side] = drain_latest(inputs[side]) or samples.get(side)
            positions[side] = drain_latest(poses[side]) or positions.get(side)
        try:
            request = json.loads((directory / 'workbench-request.json').read_text())
            if request['revision'] != revision:
                revision = request['revision']
                buttons.reset(bool(request['active']))
        except FileNotFoundError:
            pass
        sample = samples.get('right')
        fresh = sample is not None and 0 <= wall - sample.timestamp_us / 1e6 < .1
        valid = fresh and sample.connection_state == 1 and bool(sample.tracking_valid)
        if not valid:
            buttons.reset(buttons.active)
        if fresh and sample.sequence > sequence:
            sequence = sample.sequence
            send(output, buttons.update(sample, valid=valid))
        if now - displayed >= .05:
            status = {'sample_monotonic_ns': time.monotonic_ns(), 'revision': revision,
                      'active': buttons.active, 'control_enabled': buttons.enabled, 'tracking': bool(valid),
                      'device_timestamp_ns': int(sample.timestamp_us * 1000) if sample else 0,
                      'hands': {s: hand(s, samples.get(s), positions.get(s), wall) for s in inputs}}
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
