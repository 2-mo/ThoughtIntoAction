"""Supervisor-only process factories. No subscriptions, leases, IK or robot SDK."""
from pathlib import Path
import sys
import time
from embodied.control.settings import ROOT


class QuestDevice:
    @staticmethod
    def open_observer(prefix, config):
        from .workbench import InputObserver
        return InputObserver(config.get('input_prefix') or prefix)

    @staticmethod
    def validate(settings):
        if not Path(settings.sdk_library).is_file():
            raise ValueError(f'XRoboToolkit PC Service SDK not found: {settings.sdk_library}')

    @staticmethod
    def commands(configs, prefix):
        return {'input': [str(ROOT / 'src/embodied/devices/inputs/quest/native/bin/rollio-device-xrobot'), 'run', '--config', str(configs['xrobot'])]}


class DemoDevice:
    @staticmethod
    def validate(settings):
        pass

    @staticmethod
    def commands(configs, prefix):
        return {'input': [sys.executable, '-m', 'embodied.devices.inputs.quest.source_demo', '--prefix', prefix]}


class NativeRouting:
    @staticmethod
    def health(prefix, dependencies):
        from .native_health import DependencyPublisher
        return DependencyPublisher(prefix, dependencies)

    @staticmethod
    def configure(directory, prefix, settings):
        from .native_config import write_native_configs
        return write_native_configs(directory, prefix, settings.robot.gripper_max_width,
                                    settings.sdk_library, settings.vr.base_yaw_deg,
                                    settings.vr.grip_threshold, settings.vr.position_scale,
                                    feedback_hz=settings.robot.control_hz,
                                    cartesian_speed_limits=(settings.robot.teleop_linear_speed_m_s,
                                        settings.robot.teleop_angular_speed_rad_s)
                                    if hasattr(settings.robot, 'teleop_linear_speed_m_s') else None)

    @staticmethod
    def commands(configs, prefix):
        router = str(ROOT / 'src/embodied/devices/inputs/quest/native/bin/rollio-teleop-router')
        commands = {name: [router, 'run', '--config', str(configs[name])]
                    for name in ('arm', 'gripper') if name in configs}
        commands['coordinator'] = [router, 'coordinate', '--config', str(configs['coordinator'])]
        return commands

    @classmethod
    def start(cls, children, configs, prefix, stopped):
        # Bundled routers open global control/events with library defaults if
        # they win creation. Let the coordinator establish the native 4/32/32
        # capacity contract first. A published permit proves its ports opened.
        from embodied.control.ipc.services import create_node, open_service, make_subscriber, drain_latest
        from embodied.devices.inputs.quest.native_health import TeleopGate
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
