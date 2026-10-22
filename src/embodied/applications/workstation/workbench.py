"""Compose trusted component plugins; no robot or camera SDK imports during validation."""
import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import time

from embodied.contracts.registry import Registry, PROJECT_ROOT


def profile(path):
    path = Path(path)
    if not path.is_file():
        path = PROJECT_ROOT / 'configs/workbench' / (str(path)+'.json')
    config = json.loads(path.read_text())
    if config.get('version') != 1:
        raise ValueError('Unsupported workbench profile version')
    registry = Registry()
    robot = registry.spec('robot', config['robot'])
    if robot.runtime != 'control':
        raise ValueError('Robot backend must run in the control environment')
    source = config.get('source', 'cli')
    config.setdefault('source', source)
    source_spec = registry.spec('source', source)
    if source_spec.runtime != 'workstation':
        raise ValueError('Control source must run in the workstation environment')
    if registry.spec('planner', config.get('planner', 'motion.default')).runtime != 'control':
        raise ValueError('Planner must run in the control environment')
    if config.get('controller') and registry.spec('controller', config['controller']).runtime != 'control':
        raise ValueError('Target controller must run in the control environment')
    # Convert an explicitly supplied old VR selection; ordinary profiles have no VR dependency.
    if 'vr' in config and 'native' not in config:
        config['native'] = {'device': config.pop('vr'), 'routing': 'arx.native',
                            'ingress': 'arx.native', 'channel': 'native'}
    native = config.get('native')
    if native:
        for field, group in (('device', 'vr'), ('routing', 'routing'), ('ingress', 'ingress')):
            spec = registry.spec(group, native[field])
            if spec.runtime != 'control':
                raise ValueError(f'Native {field} must run in the control environment')
            if robot.hardware and spec.simulation_only:
                raise ValueError('Synthetic native components are simulation only')
        channel = native.get('channel', 'native')
        if not isinstance(channel, str) or not channel.isidentifier() or channel == 'rpc':
            raise ValueError('Native ingress channel must be an identifier other than rpc')
        native['channel'] = channel
    if 'native-ingress' in source_spec.capabilities:
        if not native:
            raise ValueError('Selected source requires an explicit native component connection')
        options = config.setdefault('source_options', {})
        options.setdefault('ingress', native['channel'])
        if options['ingress'] != native['channel']:
            raise ValueError('Source ingress does not match the installed native channel')
    if config.get('camera', 'none') != 'none':
        camera = registry.spec('camera', config['camera'])
        if robot.hardware and camera.simulation_only:
            raise ValueError('A real robot profile cannot label a synthetic camera as hardware')
    return config, robot


def start(config, robot, args):
    runtime = source = recorder = None
    children = []
    command = [str(PROJECT_ROOT/'run.sh'), 'control', 'start', '--managed',
               '--backend', 'real' if robot.hardware else 'sim', '--robot-plugin', robot.name,
               '--input', 'idle']
    if config.get('native'):
        native = config['native']
        for field, flag in (('device', 'input-plugin'), ('routing', 'routing-plugin'),
                            ('ingress', 'ingress-plugin'), ('channel', 'ingress-channel')):
            command += ['--'+flag, native[field]]
    if config.get('controller'):
        command += ['--controller-plugin', config['controller']]
    command += ['--planner-plugin', config.get('planner', 'motion.default')]
    if args.headless:
        command.append('--headless')
    for key in ('config', 'vr_config'):
        if config.get(key):
            command += ['--'+key.replace('_', '-'), str(PROJECT_ROOT/config[key])]
    runtime = subprocess.Popen(command, cwd=PROJECT_ROOT)
    children.append(runtime)
    try:
        deadline = time.monotonic()+20
        directory = None
        while time.monotonic() < deadline and runtime.poll() is None:
            for path in (PROJECT_ROOT/'outputs/runtime/control').glob('*/commands.sock'):
                identity = json.loads((path.parent/'runtime.json').read_text())
                if identity['supervisor_pid'] == runtime.pid and (path.parent/'runtime.ready').exists():
                    directory = path.parent
                    break
            if directory:
                break
            time.sleep(.05)
        if directory is None:
            raise RuntimeError('Control runtime did not become ready')
        (directory/'workbench.json').write_text(json.dumps(config, indent=2)+'\n')
        options = config.get('source_options', config.get('phone', {}))
        source = subprocess.Popen([str(PROJECT_ROOT/'run.sh'), 'component-source', '--run', str(directory),
            '--plugin', config['source'], '--options', json.dumps(options)], cwd=PROJECT_ROOT)
        children.insert(0, source)
        if args.record_task:
            from embodied.contracts.runtime.control import ControlClient
            if 'lease' in Registry().spec('source', config['source']).capabilities:
                deadline = time.monotonic()+5
                while time.monotonic() < deadline:
                    if source.poll() is not None:
                        raise RuntimeError('Source failed before acquiring control')
                    if ControlClient(directory).status()['owner']:
                        break
                    time.sleep(.05)
                else:
                    raise RuntimeError('Source did not acquire control')
            recorder = subprocess.Popen([str(PROJECT_ROOT/'run.sh'), 'collect', '--run', str(directory),
                '--camera', config.get('camera', 'none'), '--camera-options', json.dumps(config.get('camera_options', {})),
                '--task', args.record_task], cwd=PROJECT_ROOT)
            children.insert(0, recorder)
        print(f'Workbench ready: {directory}', flush=True)
        while runtime.poll() is None:
            if source and source.poll() is not None:
                print(f'Source exited ({source.returncode}); worker deadlines enter hold.', flush=True)
                source = None
            time.sleep(.1)
        return runtime.returncode
    finally:
        # Stop recording before source ownership changes, then stop the runtime.
        for child in children:
            if child.poll() is None:
                child.send_signal(signal.SIGINT)
                try:
                    child.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    child.terminate()
                    child.wait(timeout=5)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('command', choices=('plugins', 'validate', 'start', 'status', 'stop'))
    parser.add_argument('--profile', default='sim-manual')
    parser.add_argument('--run', type=Path)
    parser.add_argument('--headless', action='store_true')
    parser.add_argument('--record-task')
    args = parser.parse_args(argv)
    if args.command == 'plugins':
        print(json.dumps(Registry().entries, ensure_ascii=False, indent=2)); return 0
    if args.command in ('status', 'stop'):
        if args.run is None:
            parser.error('--run is required')
        from embodied.contracts.runtime.control import ControlClient
        client = ControlClient(args.run)
        print(json.dumps(client.status() if args.command == 'status' else client.request('stop'), indent=2))
        return 0
    config, robot = profile(args.profile)
    if args.command == 'validate':
        print(json.dumps({'valid': True, 'profile': config, 'hardware': robot.hardware}, indent=2)); return 0
    def stop(*_):
        raise KeyboardInterrupt
    previous = {sig: signal.signal(sig, stop) for sig in (signal.SIGTERM, signal.SIGHUP)}
    try:
        return start(config, robot, args)
    except KeyboardInterrupt:
        return 130
    finally:
        for sig, handler in previous.items():
            signal.signal(sig, handler)


if __name__ == '__main__':
    raise SystemExit(main())
