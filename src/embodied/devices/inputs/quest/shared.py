"""Reuse native Quest routing with a single input owner and per-arm namespaces."""
from dataclasses import replace
import time
try:
    import tomllib
except ModuleNotFoundError:
    import tomli as tomllib

from .composition import NativeRouting, QuestDevice
from .native_config import write_native_configs, _toml
from .sources import NativeIngress
from embodied.contracts.control import Mode


class SharedDevice(QuestDevice):
    @staticmethod
    def commands(configs, prefix):
        return {}  # The workbench pair supervisor owns the single input process.


class Routing(NativeRouting):
    @staticmethod
    def configure(directory, prefix, settings):
        vr = settings.vr
        paths = write_native_configs(directory, prefix, settings.robot.gripper_max_width,
            settings.sdk_library, vr.base_yaw_deg, vr.grip_threshold, vr.position_scale,
            controller=vr.controller, input_prefix=vr.input_prefix or prefix,
            feedback_hz=settings.robot.control_hz,
            cartesian_speed_limits=(settings.robot.teleop_linear_speed_m_s,
                                    settings.robot.teleop_angular_speed_rad_s))
        # Keep both native mappings but give the gripper its own readiness
        # group. Missing gripper feedback must not withdraw the arm permit.
        if 'gripper' in paths:
            coordinator = tomllib.loads(paths['coordinator'].read_text())
            grip_group = f'{prefix}/gripper-group'
            grip_coordinator = {**coordinator, 'process_id': f'{prefix}.gripper-coordinator',
                'group_id': grip_group, 'permit_topic': grip_group+'/permit',
                'members': coordinator['members'][1:], 'routes': coordinator['routes'][1:]}
            coordinator['members'] = coordinator['members'][:1]
            coordinator['routes'] = coordinator['routes'][:1]
            paths['coordinator'].write_text(_toml(coordinator))
            path = paths['coordinator'].with_name('gripper-coordinator.toml')
            path.write_text(_toml(grip_coordinator))
            paths['gripper-coordinator'] = path
            gripper = tomllib.loads(paths['gripper'].read_text())
            gripper['control_group_id'] = grip_group
            gripper['group_permit_topic'] = grip_group+'/permit'
            paths['gripper'].write_text(_toml(gripper))
        return paths

    @staticmethod
    def commands(configs, prefix):
        commands = NativeRouting.commands(configs, prefix)
        if 'gripper-coordinator' in configs:
            commands['gripper-coordinator'] = [commands['coordinator'][0], 'coordinate',
                '--config', str(configs['gripper-coordinator'])]
        return commands


class Ingress(NativeIngress):
    def __init__(self, prefix, dependencies, gripper=True, settings=None):
        super().__init__(prefix, dependencies, gripper=False)
        from embodied.control.ipc.services import open_service, make_subscriber
        from embodied.control.ipc.types import XrControllerInput
        vr = settings.vr
        self.subscribers['input'] = make_subscriber(open_service(self.node,
            f'{vr.input_prefix or prefix}/xr/{vr.controller}_controller/states/input', XrControllerInput))
        self.gripper_released = self.gripper_armed = False
        self.gripper_epoch = None
        self.initialize_released = False
        self.initialize_since = None
        self.initialize_sent = False

    def poll(self, state, now, epoch, selected):
        candidate = super().poll(state, now, epoch, selected)
        now = time.monotonic()
        sample = self.samples['input']
        valid = selected and self.status['ready'] and sample.fresh(now) and not candidate.fault
        if epoch != self.gripper_epoch or not valid:
            self.gripper_epoch = epoch
            self.gripper_released = self.gripper_armed = False
            self.initialize_released = self.initialize_sent = False
            self.initialize_since = None
        command = candidate.command
        initialize = False
        if valid:
            inp = sample.sample
            neutral = inp.trigger < .2 and inp.grip < .2 and not (inp.buttons_bitset & 12)
            if neutral:
                self.initialize_released = True
            secondary = bool(inp.buttons_bitset & 8)
            if not secondary:
                self.initialize_since = None
                self.initialize_sent = False
            elif (self.initialize_released and not self.initialize_sent and
                  not (inp.buttons_bitset & 4) and inp.trigger < .2 and inp.grip < .2):
                if self.initialize_since is None:
                    self.initialize_since = now
                if now-self.initialize_since >= 1.:
                    initialize = True
                    self.initialize_sent = True
                    self.initialize_released = False
            else:
                self.initialize_since = None
            if state.gripper_width is None or initialize:
                self.gripper_released = self.gripper_armed = False
            elif inp.trigger < .2 and not secondary:
                self.gripper_released = True
            elif self.gripper_released and inp.trigger > .2 and not secondary:
                self.gripper_armed = True
        if not valid or not self.gripper_armed:
            command = replace(command, gripper_target=None)
        if initialize:
            command = replace(command, mode=Mode.COMMAND_FOLLOWING, hold=True, reset_reference=True,
                pose_target=None, joint_target=None, engaged=False, gripper_target=None,
                gripper_initialize_open=True)
        self.status.update(gripper_ready=state.gripper_width is not None,
            gripper_armed=self.gripper_armed, gripper_initialize_open=initialize)
        return replace(candidate, command=command)
