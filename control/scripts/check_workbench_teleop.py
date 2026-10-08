"""Exercise the actual native routers + MuJoCo with isolated hand-controller frames."""
from pathlib import Path
import json
import os
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
sys.path[:0] = [str(ROOT), str(ROOT.parent / 'src')]

if '--input' in sys.argv:
    from arx5_vr.ipc.services import create_node, open_service, make_publisher, send
    from arx5_vr.ipc.types import XrControllerInput, Pose7
    directory, prefix = Path(sys.argv[-2]), sys.argv[-1]
    node = create_node()
    inp = make_publisher(open_service(node, f'{prefix}/xr/right_controller/states/input', XrControllerInput))
    pose = make_publisher(open_service(node, f'{prefix}/xr/right_controller/states/pose', Pose7))
    sequence = 0
    while True:
        try:
            action = json.loads((directory / 'fixture.json').read_text())
        except FileNotFoundError:
            action = {}
        sequence += 1
        stamp = time.time_ns() // 1000
        p = Pose7.from_values(stamp, [*action.get('position', [0,0,0]), 0,0,0,1])
        sample = XrControllerInput()
        sample.timestamp_us, sample.sequence = stamp, sequence
        sample.connection_state, sample.tracking_valid = 1, int(action.get('valid', True))
        sample.joystick_y = action.get('y', 0)
        sample.grip, sample.trigger = action.get('grip', 0), action.get('trigger', 0)
        sample.buttons_bitset = action.get('buttons', 0)
        send(inp, sample); send(pose, p)
        time.sleep(1/90)
elif '--runtime' in sys.argv:
    from adapters.workbench import QuestDevice
    from arx5_vr.app import main
    QuestDevice.commands = staticmethod(lambda configs, prefix: {'input': [sys.executable, __file__, '--input', str(configs['xrobot'].parent), prefix]})
    raise SystemExit(main(['run', '--backend', 'sim', '--input', 'vr', '--managed', '--headless', '--telemetry',
                          '--input-plugin', 'arx.workbench', '--routing-plugin', 'arx.workbench', '--ingress-plugin', 'arx.workbench']))
else:
    import numpy as np
    from runtime_client import RuntimeReader
    from runtime_client.control import ControlSession
    output = ROOT.parent / 'outputs/logs/workbench-native-check.log'
    child = subprocess.Popen([sys.executable, __file__, '--runtime'], cwd=ROOT, stdout=output.open('w'), stderr=subprocess.STDOUT)
    session = reader = None
    try:
        deadline = time.monotonic() + 15
        directory = None
        while time.monotonic() < deadline and child.poll() is None:
            for path in (ROOT/'run').glob('*/runtime.json'):
                identity = json.loads(path.read_text())
                if identity['supervisor_pid'] == child.pid and (path.parent/'runtime.ready').exists(): directory=path.parent
            if directory: break
            time.sleep(.05)
        assert directory, output.read_text()
        (directory/'workbench-request.json').write_text(json.dumps({'revision':1, 'active':True}))
        session = ControlSession(directory, source='quest', ingress='native', purpose='teleop').__enter__()
        reader = RuntimeReader(directory, 'latest')
        def action(**fields):
            temporary = directory/'fixture.tmp'; temporary.write_text(json.dumps(fields)); temporary.replace(directory/'fixture.json')
        def collect(seconds):
            samples=[]; end=time.monotonic()+seconds
            while time.monotonic()<end:
                tick=reader.read()
                assert not tick['error'], tick['error']
                if tick.get('feedback'): samples.append(tick)
            return samples
        def input_status(): return json.loads((directory/'workbench-input.json').read_text())
        q = lambda samples: np.array([s['feedback']['q'] for s in samples])
        collect(.3)
        idle=collect(.4)
        action(buttons=4, grip=1, trigger=1, position=[0,.03,-.03])
        before=collect(.35)
        assert not input_status()['control_enabled']
        assert np.max(np.abs(q(before)-q(idle)[-1])) < .005, 'Standby moved'
        action();collect(.2)
        action(y=-1);collect(1.35)
        assert input_status()['control_enabled'], input_status()
        action();collect(.3)
        action(grip=1);collect(.25)
        action(grip=1, trigger=.7, position=[0,.02,-.025])
        moved=collect(.8)
        assert any(s.get('target',{}).get('pose') for s in moved), ('No native target', session.status())
        assert np.max(np.abs(q(moved)-q(idle)[-1]))>.01, 'Quest did not drive MuJoCo'
        assert moved[-1]['feedback']['gripper_width'] < .06, 'Trigger did not close gripper'
        action(position=[.2,.2,.2]);collect(.4)
        released=collect(.35)
        assert np.max(np.ptp(q(released),axis=0))<.01, 'Unclutched hand moved arm'
        action(valid=False, grip=1);collect(.2)
        assert not input_status()['control_enabled'], 'Tracking loss did not disarm'
        action(grip=1);collect(.3)
        assert not input_status()['control_enabled'], 'Held Grip rearmed after reconnect'
        print('PASS: standby, original native follow + IK + MuJoCo, trigger, clutch release, tracking-loss disarm. '+str(directory))
    finally:
        if session: session.close()
        if reader: reader.close()
        child.send_signal(signal.SIGINT)
        child.wait(timeout=20)
