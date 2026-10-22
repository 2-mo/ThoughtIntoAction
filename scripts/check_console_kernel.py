#!/usr/bin/env python3
"""Console backend in kernel mode on the MuJoCo kernel, through the public WebSocket protocol the page uses.

Hand-eye results are written to a temporary directory only.
"""
import argparse
import itertools
import json
import math
from pathlib import Path
import socket
import tempfile
import threading
import time

import numpy as np
import uvicorn
from websockets.sync.client import connect

from embodied.applications import calibration
from embodied.applications.console.app import create_app
from embodied.applications.console.core.settings import BackendSettings
from embodied.contracts.runtime.control import ControlClient
from embodied.applications.workstation.control import sim_kernel


class Page:
    """Speaks apps/console/src/api/control-protocol.ts."""

    def __init__(self, url, catalog):
        self.socket, self.catalog = connect(url, max_size=None), catalog
        self.ids, self.sequence = itertools.count(1), itertools.count(1)
        self.hello = self.receive()
        assert self.hello['type'] == 'session.ready', self.hello

    def receive(self):
        return json.loads(self.socket.recv(timeout=30))

    def send(self, action, payload=None):
        feature, command = self.catalog.public_name(action)
        request_id = next(self.ids)
        self.socket.send(json.dumps({'type': 'command', 'request_id': request_id, 'feature': feature,
                                     'command': command, 'payload': payload or {}}))
        return request_id

    def command(self, action, payload=None, *, ok=True):
        request_id = self.send(action, payload)
        progress = 0
        while True:
            event = self.receive()
            if event.get('request_id') != request_id:
                continue
            progress += event['type'] == 'motion.progress'
            if event['type'] in ('command.succeeded', 'command.failed', 'state.snapshot'):
                break
        assert event.get('ok', True) is ok, (action, event)
        return event['data'], progress

    def stream(self, feature, stream, command, payload, session_id=None):
        sequence = next(self.sequence)
        if session_id:
            payload = {**payload, 'session_id': session_id}
        self.socket.send(json.dumps({'type': 'stream', 'feature': feature, 'stream': stream, 'command': command,
                                     'sequence': sequence, 'payload': payload}))
        while True:
            event = self.receive()
            if event['type'] in ('teleoperation.state', 'servo.state') and event['data'].get('seq') == sequence:
                assert event.get('ok') is True, event
                return event['data']


def tcp(state):
    return [1000 * v for v in state['eef_pose_6d'][:3]]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    output = parser.parse_args().output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    with sim_kernel(output / 'runtime.log') as directory, socket.socket() as probe:
        probe.bind(('127.0.0.1', 0))
        port = probe.getsockname()[1]
        probe.close()
        app = create_app(BackendSettings(port=port, kernel=True, run=directory))
        server = uvicorn.Server(uvicorn.Config(app, host='127.0.0.1', port=port, log_level='warning'))
        thread = threading.Thread(target=server.run, daemon=True)
        thread.start()
        while not server.started:
            time.sleep(.05)
        try:
            page = Page(f'ws://127.0.0.1:{port}/api/v1/control', app.state.catalog)
            assert page.hello['data']['limits']['motion_profile_revision'] == 6
            assert page.hello['data']['state']['connected'] is False
            state = page.command('connect')[0]['state']
            assert state['connected'] and state['mode'] == 'active' and len(state['joint_position_deg']) == 6, state

            # Leave the singular zero pose joint by joint, as the page's joint sliders do.
            moves = 0
            for joint, target in ((3, 51.6), (2, 51.6), (4, -22.9)):
                data, progress = page.command('move_joint_target', {'joint': joint, 'target_deg': target,
                                                                    'speed_percent': 100})
                assert data['result']['kind'] == 'joint_motion' and max(data['result']['final_error_deg']) < 1, data
                moves += progress
            assert moves >= 10, f'only {moves} motion updates'
            page.command('set_reference')

            # Pose: preview +30 mm along base Y, then execute the same target.
            start = tcp(page.command('state')[0]['state'])
            preview = page.command('preview_xyz', {'x_mm': 0, 'y_mm': 30, 'z_mm': 0, 'speed_percent': 50,
                                                   'path_mode': 'linear'})[0]['result']
            assert preview['kind'] == 'cartesian_preview' and len(preview['path_xyz_mm']) >= 2, preview
            assert preview['ik_position_error_mm'] < 1, preview
            moved = page.command('move_xyz', {'x_mm': 0, 'y_mm': 30, 'z_mm': 0, 'speed_percent': 50,
                                              'path_mode': 'linear', 'mode_after_motion': 'active'})[0]
            pose_move = math.dist(start, tcp(moved['state']))
            assert moved['result']['final_position_error_mm'] < 3, moved['result']
            page.command('move_xyz', {'x_mm': 0, 'y_mm': 30, 'z_mm': 0, 'speed_percent': 50}, ok=False)  # preview used

            gripper = page.command('gripper_position', {'width_mm': 40, 'speed_percent': 50})[0]['result']
            assert abs(gripper['final_width_mm'] - 40) < 2, gripper

            # Gesture teleoperation: cumulative base-frame deltas from the session start.
            before = tcp(page.command('state')[0]['state'])
            reply = page.stream('teleoperation', 'target', 'start', {'profile': 'fine', 'control_side': 'left',
                                'open_width_mm': 60, 'closing_torque_nm': .1, 'preview_only': False})
            session_id = reply['session_id']
            for step in range(1, 46):
                page.stream('teleoperation', 'target', 'update', {
                    'delta_xyz_m': [0, 0, -.02 * min(1, step / 30)], 'delta_orientation_xyzw': [0, 0, 0, 1],
                    'gripper_state': 'open', 'client_time_ms': 0}, session_id)
                time.sleep(1 / 30)
            page.stream('teleoperation', 'target', 'stop', {}, session_id)
            after = page.command('state')[0]['state']
            assert abs(tcp(after)[2] - before[2] + 20) < 3 and abs(after['gripper_position_mm'] - 60) < 3, (before, tcp(after))

            # Drag servo: +15 mm along X, ending in damping, then back to position mode.
            before = tcp(after)
            page.stream('arm', 'cartesian', 'start', {'speed_percent': 50, 'mode_after_motion': 'damping'})
            for step in range(1, 31):
                page.stream('arm', 'cartesian', 'update', {'x_mm': 15 * min(1, step / 20), 'y_mm': 0, 'z_mm': 0})
                time.sleep(.05)
            page.stream('arm', 'cartesian', 'stop', {})
            state = page.command('state')[0]['state']
            assert state['mode'] == 'damping' and abs(tcp(state)[0] - before[0] - 15) < 3, (before, tcp(state))
            page.command('activate')

            # Back to the recorded reference.
            park = page.command('preview_park', {'speed_percent': 100})[0]['result']
            assert park['kind'] == 'park_preview' and park['trajectory_duration_s'] > 0, park
            state = page.command('park', {'speed_percent': 100})[0]['state']
            assert max(abs(a - b) for a, b in zip(state['joint_position_deg'], state['reference_joint_position_deg'])) < 1

            error = page.command('payload_calibration_start', ok=False)[0]['error']
            assert '尚未迁到' in error, error

            # Hand-eye runs in the CLI; a saved fixed-camera result places that camera in the scene.
            assert 'calib handeye' in page.command('handeye_calibration_start', ok=False)[0]['error']
            known = np.eye(4); known[:3, 3] = [.6, .1, .5]
            with tempfile.TemporaryDirectory() as results:
                calibration.RESULTS = Path(results)
                calibration.save('exterior_rgb', {'status': 'validated', 'mount': 'fixed', 'T_base_camera': known.tolist()})
                shown = page.command('state')[0]['state']['handeye_validation']
            assert shown['enabled'] and shown['T_base_camera'] == known.tolist(), shown
            page.command('disconnect')
            assert ControlClient(directory).status()['owner'] is None, 'lease not released'
            page.socket.close()
            print(f'PASS: console on kernel {directory.name}: joint, pose, park, gripper, teleoperation, drag and '
                  f'hand-eye results through /api/v1/control; pose move {pose_move:.1f} mm, lease released.')
        finally:
            server.should_exit = True
            thread.join(timeout=10)


if __name__ == '__main__':
    main()
