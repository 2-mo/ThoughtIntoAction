"""Supervisor-only process factories. No subscriptions, leases, IK or robot SDK."""
from pathlib import Path
import sys
import time
from arx5_vr.settings import ROOT


class QuestDevice:
    @staticmethod
    def validate(settings):
        if not Path(settings.sdk_library).is_file():
            raise ValueError(f'XRoboToolkit PC Service SDK not found: {settings.sdk_library}')

    @staticmethod
    def commands(configs, prefix):
        return {'input': [str(ROOT / 'native/bin/rollio-device-xrobot'), 'run', '--config', str(configs['xrobot'])]}


class DemoDevice:
    @staticmethod
    def validate(settings):
        pass

    @staticmethod
    def commands(configs, prefix):
        return {'input': [sys.executable, '-m', 'adapters.demo', '--prefix', prefix]}


class NativeRouting:
    @staticmethod
    def commands(configs, prefix):
        router = str(ROOT / 'native/bin/rollio-teleop-router')
        commands = {name: [router, 'run', '--config', str(configs[name])]
                    for name in ('arm', 'gripper') if name in configs}
        commands['coordinator'] = [router, 'coordinate', '--config', str(configs['coordinator'])]
        return commands

    @classmethod
    def start(cls, children, configs, prefix, stopped):
        # Bundled routers open global control/events with library defaults if
        # they win creation. Let the coordinator establish the native 4/32/32
        # capacity contract first. A published permit proves its ports opened.
        from arx5_vr.ipc.services import create_node, open_service, make_subscriber, drain_latest
        from .native_health import TeleopGate
        node = create_node()
        subscriber = make_subscriber(open_service(node, f'{prefix}/group/permit', TeleopGate))
        commands = cls.commands(configs, prefix)
        process = children.start('coordinator', commands.pop('coordinator'), cwd=ROOT)
        deadline = time.monotonic() + 5
        while drain_latest(subscriber) is None:
            if stopped():
                raise InterruptedError('Native startup interrupted')
            if process.poll() is not None or time.monotonic() > deadline:
                raise RuntimeError('Native coordinator startup failed; see coordinator.log')
            time.sleep(.01)
        for name, command in commands.items():
            children.start(name, command, cwd=ROOT)


def native_components(registry, device, routing, settings, backend):
    """Resolve only explicitly installed components before opening the robot."""
    spec = registry.spec('vr', device)
    if backend == 'real' and spec.simulation_only:
        raise ValueError('Synthetic input is simulation only')
    driver = registry.load('vr', device, runtime='control')
    driver.validate(settings)
    routes = registry.load('routing', routing, runtime='control')
    return driver, routes
