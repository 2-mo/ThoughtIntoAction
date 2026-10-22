"""Exercise the actual native routers + MuJoCo with isolated hand-controller frames."""
from pathlib import Path
import json
import os
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path[:0] = [str(ROOT / 'src'), str(ROOT)]

if '--input' in sys.argv:
    from embodied.control.ipc.services import create_node, open_service, make_publisher, send
    from embodied.control.ipc.types import XrControllerInput, Pose7
    directory, prefix = Path(sys.argv[-2]), sys.argv[-1]
    node = create_node()
    publishers = {side: (make_publisher(open_service(node, f'{prefix}/xr/{side}_controller/states/input', XrControllerInput)),
                         make_publisher(open_service(node, f'{prefix}/xr/{side}_controller/states/pose', Pose7)))
                  for side in ('right', 'left')}
    sequence = 0
    while True:
        try:
            action = json.loads((directory / 'fixture.json').read_text())
        except FileNotFoundError:
            action = {}
        sequence += 1
        stamp = time.time_ns() // 1000
        for side, (inp, pose) in publishers.items():
            fields = action if side == 'right' else action.get('left', {})
            p = Pose7.from_values(stamp, [*fields.get('position', [0,0,0]), 0,0,0,1])
            sample = XrControllerInput()
            sample.timestamp_us, sample.sequence = stamp, sequence
            sample.connection_state, sample.tracking_valid = 1, int(fields.get('valid', True))
            sample.joystick_y = fields.get('y', 0)
            sample.grip, sample.trigger = fields.get('grip', 0), fields.get('trigger', 0)
            sample.buttons_bitset = fields.get('buttons', 0)
            send(inp, sample); send(pose, p)
        time.sleep(1/90)
elif '--runtime' in sys.argv:
    from embodied.devices.inputs.quest.workbench import QuestDevice
    from embodied.control.app import main
    QuestDevice.commands = staticmethod(lambda configs, prefix: {'input': [sys.executable, __file__, '--input', str(configs['xrobot'].parent), prefix]})
    raise SystemExit(main(['run', '--backend', 'sim', '--input', 'vr', '--managed', '--headless', '--telemetry',
                          '--input-plugin', 'arx.workbench', '--routing-plugin', 'arx.workbench', '--ingress-plugin', 'arx.workbench']))
else:
    import numpy as np
    from embodied.contracts.runtime import RuntimeReader
    from embodied.contracts.runtime.control import ControlSession
    output = ROOT / 'outputs/logs/workbench-native-check.log'
    output.parent.mkdir(parents=True, exist_ok=True)
    child = subprocess.Popen([sys.executable, __file__, '--runtime'], cwd=ROOT, stdout=output.open('w'), stderr=subprocess.STDOUT)
    session = reader = None
    try:
        deadline = time.monotonic() + 15
        directory = None
        while time.monotonic() < deadline and child.poll() is None:
            for path in (ROOT/'outputs/runtime/control').glob('*/runtime.json'):
                identity = json.loads(path.read_text())
                if identity['supervisor_pid'] == child.pid and (path.parent/'runtime.ready').exists(): directory=path.parent
            if directory: break
            time.sleep(.05)
        assert directory, output.read_text()
        def action(**fields):
            temporary = directory/'fixture.tmp'; temporary.write_text(json.dumps(fields)); temporary.replace(directory/'fixture.json')
        # Controls already held when teleoperation is allowed stay inert until released.
        action(buttons=4, grip=1, trigger=1, position=[0,.03,-.03])
        time.sleep(.2)
        (directory/'workbench-request.json').write_text(json.dumps({'revision':1, 'active':True}))
        session = ControlSession(directory, source='quest', ingress='native', purpose='teleop').__enter__()
        reader = RuntimeReader(directory, 'latest')
        described = session.describe()
        assert described['robot']['control_hz'] == 250 and described['workspace'] == {'minimum_tcp_z_m': 0.0}, described
        def collect(seconds):
            samples=[]; end=time.monotonic()+seconds
            while time.monotonic()<end:
                tick=reader.read()
                assert not tick['error'], tick['error']
                if tick.get('feedback'): samples.append(tick)
            return samples
        def input_status(): return json.loads((directory/'workbench-input.json').read_text())
        q = lambda samples: np.array([s['feedback']['q'] for s in samples])
        idle=collect(.4)
        before=collect(.35)
        assert not input_status()['control_enabled']
        assert np.max(np.abs(q(before)-q(idle)[0])) < .005, 'Held controls moved the arm'
        action();collect(.3)
        assert input_status()['control_enabled'], input_status()
        # Left controller bits (rollio XR_BUTTON_*): X primary, Y secondary, L3 stick click.
        action(left={'buttons': 4|2});collect(.2)
        left = input_status()['hands']['left']['buttons']
        assert left['X'] and left['L3'] and not left['Y'], left
        action();collect(3.5)  # upstream waits 3 s in Disabled after start
        def mode_switch():
            # Upstream chord: A held for at least 150 ms, then released.
            action(buttons=4);collect(.25)
            action();s=collect(1.3)
            return s[-1]['feedback']['mode'], s[-1]['phase']  # ticks.MODES / PHASES (1 = teleop control)
        assert mode_switch() == (3, 1), 'first A did not start command-following'
        assert mode_switch()[0] == 2, 'second A did not switch to free-drive'
        assert mode_switch() == (3, 1), 'third A did not return to command-following'
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
        print('PASS: held-control arming, left bits, upstream A mode switch, native follow + IK + MuJoCo, trigger, clutch release, tracking-loss disarm. '+str(directory))
    finally:
        if session: session.close()
        if reader: reader.close()
        child.send_signal(signal.SIGINT)
        child.wait(timeout=20)
