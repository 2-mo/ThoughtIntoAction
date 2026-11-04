"""Compose two existing kernels and one Quest input owner; no robot control loop."""
import argparse
from contextlib import ExitStack
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import uuid

from embodied.contracts.paths import PROJECT_ROOT as ROOT
from embodied.applications.workstation.workbench import profile
from embodied.applications.workstation.pair_state import PAIR_STATE, active_pair
from embodied.contracts.device_lease import DeviceLease


def prepare(directory, *, backend='real', source='quest'):
    """Resolve static deployments and input topics without opening devices."""
    directory = Path(directory).resolve()
    prefix = 'robot-pair-'+uuid.uuid4().hex[:12]
    xr_config = None
    if source == 'quest':
        from embodied.devices.inputs.quest.native_config import write_native_configs
        from embodied.devices.inputs.quest.settings import load_profile
        vr_path = ROOT/'configs/inputs/quest.json'
        vr = load_profile(vr_path)
        paths = write_native_configs(directory, prefix, .07, vr.sdk_library,
            vr.base_yaw_deg, vr.grip_threshold, vr.position_scale)
        # Exactly one native SDK reader publishes both controllers. Both kernels
        # subscribe to these topics; routers/feedback stay in their own namespaces.
        raw = json.loads(vr_path.read_text())
        raw['sdk_library'] = vr.sdk_library
        xr_config = paths['xrobot']
        text = xr_config.read_text()
        channel = text.split('[[channels]]', 1)[1]
        text += '\n[[channels]]'+channel.replace('right_controller', 'left_controller').replace('Right Controller','Left Controller')
        xr_config.write_text(text)
    deployments = {}
    for side in ('left','right'):
        config, _ = profile('realman-'+side)
        if backend == 'sim':
            config['robot'] = 'realman.sim'
            config['config'] = 'configs/robots/realman/sim.json'
        config['pair'] = {'id': prefix, 'side': side, 'manifest': str(PAIR_STATE)}
        if source == 'quest':
            vp = directory/(side+'-quest.json')
            vp.write_text(json.dumps({**raw, 'controller': side, 'input_prefix': prefix}))
            config['vr_config'] = str(vp)
        else:
            config['source'] = source
            config.pop('native')
            config.pop('source_options', None)
        cp = directory/(side+'-workbench.json')
        cp.write_text(json.dumps(config))
        deployments[side] = cp
    return prefix, xr_config, deployments


def publish_manifest(prefix, runs):
    PAIR_STATE.parent.mkdir(parents=True, exist_ok=True)
    state = {'schema': 1, 'runtime_id': prefix, 'pid': os.getpid(),
        'boot_id': Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
        'runs': {side: str(path) for side,path in runs.items()}}
    temporary = PAIR_STATE.with_suffix('.tmp')
    temporary.write_text(json.dumps(state))
    temporary.replace(PAIR_STATE)


def find_runs(prefix):
    runs = {}
    for path in (ROOT/'outputs/runtime/control').glob('*/workbench.json'):
        try:
            config = json.loads(path.read_text())
            pair = config.get('pair', {})
            if pair.get('id') == prefix and (path.parent/'runtime.ready').exists():
                identity = json.loads((path.parent/'runtime.json').read_text())
                os.kill(identity['supervisor_pid'], 0)
                runs[pair['side']] = path.parent
        except (OSError, ValueError, KeyError):
            pass
    return runs


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--backend', choices=('real','sim'), default='real')
    parser.add_argument('--source', choices=('quest','cli'), default='quest')
    args = parser.parse_args(argv)
    if active_pair() is not None:
        raise RuntimeError('双臂会话已运行；保留现有会话，不重复打开Quest或机械臂')
    directory = ROOT/'outputs/runtime/realman'/('pair-'+time.strftime('%Y%m%d-%H%M%S'))
    directory.mkdir(parents=True, exist_ok=False)
    prefix, xr_config, deployments = prepare(directory, backend=args.backend, source=args.source)
    stopped = False
    def stop(*_):
        nonlocal stopped
        stopped = True
    previous = {sig:signal.signal(sig,stop) for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)}
    children = {}
    lease = None
    try:
        lease = DeviceLease('realman-pair')
        with ExitStack() as resources:
            # Reuse normal workbench lifecycle, including the source lease and
            # runtime's gateway, observer, planner and target controller.
            for side, path in deployments.items():
                stream = resources.enter_context((directory/(side+'.log')).open('w'))
                children[side] = subprocess.Popen([sys.executable, '-m',
                    'embodied.applications.workstation.workbench', 'start', '--profile', str(path), '--headless'],
                    cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
            deadline = time.monotonic()+30
            while not stopped:
                failed = [(side,p.returncode) for side,p in children.items() if p.poll() is not None]
                if failed:
                    raise RuntimeError(f'双臂内核退出：{failed}；日志：{directory}')
                runs = find_runs(prefix)
                if len(runs) == 2:
                    break
                if time.monotonic() >= deadline:
                    raise RuntimeError(f'等待双臂统一内核超时；日志：{directory}')
                time.sleep(.1)
            if not stopped:
                publish_manifest(prefix, runs)
                if xr_config is not None:
                    stream = resources.enter_context((directory/'quest.log').open('w'))
                    binary = ROOT/'src/embodied/devices/inputs/quest/native/bin/rollio-device-xrobot'
                    children['quest'] = subprocess.Popen([str(binary), 'run', '--config', str(xr_config)],
                        cwd=ROOT, stdout=stream, stderr=subprocess.STDOUT, start_new_session=True)
                    print('双臂已接入统一内核；原生Quest相对映射：左X / 右A按住150ms后松开切换跟随/保持，'
                          'Grip跟随，Trigger控制夹爪。启动不自动使能、回零或张开。', flush=True)
                else:
                    print('双臂已接入统一内核；无输入，控制权留给网页 / 动作 / 策略。', flush=True)
                if args.backend == 'sim':
                    print('双臂 MuJoCo 仿真通过工作台网页同屏显示；[ / ] 选择操作侧。', flush=True)
                for side,path in runs.items():
                    print(f'{side} run: {path}', flush=True)
                print(f'日志：{directory}', flush=True)
                while not stopped:
                    failed = [(side,p.returncode) for side,p in children.items() if p.poll() is not None]
                    if failed:
                        raise RuntimeError(f'双臂组件退出：{failed}；停止两个内核。日志：{directory}')
                    time.sleep(.1)
    finally:
        # Withdraw XR first, then allow each existing supervisor to stop its
        # own backend normally. Never broadcast native events to other runs.
        for name in ('quest', 'left', 'right'):
            p = children.get(name)
            if p is not None and p.poll() is None:
                os.killpg(p.pid, signal.SIGTERM)
        for p in children.values():
            try:
                p.wait(timeout=15)
            except subprocess.TimeoutExpired:
                print(f'组件 {p.pid} 未按时退出；请现场确认停止，未强制杀死。', file=sys.stderr)
        try:
            if json.loads(PAIR_STATE.read_text()).get('runtime_id') == prefix:
                PAIR_STATE.unlink()
        except (OSError, ValueError):
            pass
        for sig,handler in previous.items():
            signal.signal(sig,handler)
        if lease is not None:
            lease.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
