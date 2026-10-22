#!/usr/bin/env python3
"""Plugin acceptance against MuJoCo, real UDP and MCAP; no physical devices."""
import argparse
from dataclasses import replace
import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import threading
import time
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
from embodied.contracts.runtime.control import ControlClient, ControlSession
from embodied.devices.inputs.phone import PhoneController
from embodied.devices.phone.protocol.golden import GOLDEN_POSE
from embodied.devices.phone.protocol.packet import encode_packet
from embodied.devices.phone.core.models import Pose, Vector3, Quaternion


def until(predicate, timeout=20):
    deadline = time.monotonic()+timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(.03)
    raise TimeoutError('Acceptance condition timed out')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--calibration', action='store_true', help='Run all 26 arx calibration waypoints with synthetic marker poses')
    args = parser.parse_args()
    output = args.output.resolve(); output.mkdir(parents=True, exist_ok=False)
    result = {}
    with (output/'runtime.log').open('w') as log:
        runtime = subprocess.Popen([str(ROOT/'run.sh'), 'control', 'start', '--backend', 'sim',
            '--input', 'idle', '--managed', '--headless'], cwd=ROOT, stdout=log, stderr=subprocess.STDOUT)
        try:
            def ready():
                if runtime.poll() is not None:
                    raise RuntimeError((output/'runtime.log').read_text())
                for path in (ROOT/'outputs/runtime/control').glob('*/runtime.json'):
                    if json.loads(path.read_text())['supervisor_pid'] == runtime.pid and (path.parent/'backend.ready').exists():
                        return path.parent
            directory = until(ready)
            result['runtime'] = str(directory)
            client = ControlClient(directory)
            with ControlSession(directory) as cli:
                q = [0., .6, 1., 0., .2, 0.]
                plan = cli.preview('movej', q=q, speed=.4)
                op = cli.execute(plan)
                cli.wait(op['operation_id'], timeout=30)
                assert np.max(np.abs(np.array(cli.status()['q'])-q)) < .02
                pose = cli.status()['pose']; pose[1] += .002
                plan = cli.preview('movel', pose=pose)
                op = cli.execute(plan); cli.wait(op['operation_id'])
                result['motion'] = ['movej completed', 'movel completed']
            until(lambda: client.status()['owner'] is None and max(map(abs, client.status()['dq'])) < .02)
            phone = PhoneController(directory, port=0)
            phone.start()
            sender_stop = threading.Event()
            def send_poses():
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
                    sequence = 0
                    while not sender_stop.is_set():
                        sequence += 1
                        packet = replace(GOLDEN_POSE, sequence=sequence, session_id=42,
                            client_time_ns=time.monotonic_ns(), clutch_active=sequence > 8,
                            recenter_requested=False, gripper=.5,
                            pose=Pose(Vector3(0,0,0),Quaternion(0,0,0,1)))
                        sock.sendto(encode_packet(packet), phone.transport.local_address)
                        sender_stop.wait(1/60)
            sender = threading.Thread(target=send_poses, daemon=True); sender.start()
            collector = None
            try:
                until(lambda: phone.latest())
                collector = subprocess.Popen([str(ROOT/'run.sh'), 'collect', '--run', str(directory),
                    '--task','phone plugin acceptance', '--camera','synthetic','--duration','2',
                    '--output',str(output/'episodes')], cwd=ROOT, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                while collector.poll() is None:
                    phone.latest()
                text = collector.communicate()[0]; (output/'collector.log').write_text(text)
                assert collector.returncode == 0, text
                result['phone'] = '60 Hz UDP -> mapping -> arx command gateway; 30 FPS RGB + 250 Hz ticks'
            finally:
                if collector and collector.poll() is None:
                    collector.send_signal(signal.SIGINT); collector.wait(timeout=8)
                sender_stop.set(); sender.join(timeout=2); phone.stop()
            from embodied.data.export.exporters import Schema2
            episode = next((output/'episodes').glob('*/episode.json')).parent
            meta, rows, report, raw = Schema2().prepare(episode)
            assert len(rows) >= 30, report
            assert all(x['source'] == 'phone' for x in raw['state'])
            assert meta['source_metadata_events'][-1]['metadata']['mapping'] == 'phone_tool'
            result['episode'], result['alignment'] = str(episode), report
            until(lambda: client.status()['owner'] is None and max(map(abs, client.status()['dq'])) < .02)
            from embodied.applications.calibration.arm import CalibrationArm
            with CalibrationArm(directory) as arm:
                if args.calibration:
                    from embodied.applications.calibration.plan import kernel_model, make_plan, run_plan
                    from embodied.applications.calibration.session import Session, pose_matrix
                    from embodied.applications.calibration.solver import solve
                    from embodied.core.transforms import inverse
                    from embodied.devices.sensors.synthetic import SyntheticCamera
                    arm.hold()
                    model = kernel_model(Path(client.parameters['model']['xml_path']).with_suffix('.urdf'))
                    plan = make_plan(model, arm.snapshot()['q'], step=.2)
                    known = np.eye(4); known[:3,3] = [.03,.01,.05]
                    board = np.eye(4); board[:3,3] = [.4,0,.2]
                    class Detector:
                        def detect(self, image):
                            tcp = pose_matrix(arm.snapshot()['pose'])
                            return {'T_camera_target': inverse(known) @ inverse(tcp) @ board,
                                    'reprojection_error_undistorted_px':0., 'synthetic':True}
                    with SyntheticCamera() as camera:
                        session = Session(output/'calibration', 'wrist_rgb', 'wrist', camera.metadata, {'synthetic':True}, arm.robot)
                        session.data['plan'] = plan
                        taken = run_plan(arm, plan, lambda split, i: session.capture(camera, arm, Detector(), split, i), speed=.5)
                        session.finish('complete')
                    solved = solve(session.data['samples'], 'wrist')
                    assert solved['status'] == 'validated', solved
                    np.testing.assert_allclose(solved['T_eef_camera'],known,atol=.001)
                    (output/'calibration/result.json').write_text(json.dumps(solved,indent=2))
                    result['calibration'] = (f'{taken}/{len(plan["waypoints"])} kernel-previewed waypoints, stationary bursts, '
                                             'synthetic detections, held-out solve validated')
                else:
                    arm.hold()
                    q = np.array(arm.snapshot()['q']); q[0] += .015
                    arm.move(q, .15)
                    arm.teach(); time.sleep(.3); arm.hold()
                    result['calibration'] = 'kernel calibration arm: hold, previewed move, teaching, release'
            result['stop'] = client.request('stop')
            assert runtime.wait(timeout=15) == 0, (output/'runtime.log').read_text()
            result['runtime_report'] = json.loads((directory/'report.json').read_text())
            assert not result['runtime_report']['error']
        finally:
            if runtime.poll() is None:
                runtime.send_signal(signal.SIGINT)
                runtime.wait(timeout=15)
    (output/'result.json').write_text(json.dumps(result,indent=2))
    print(json.dumps(result,indent=2))


if __name__ == '__main__':
    main()
