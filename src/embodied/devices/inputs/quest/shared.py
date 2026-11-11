"""Reuse native Quest routing with a single input owner and per-arm namespaces."""
from dataclasses import replace
import time
import numpy as np
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
        # One mode owner per hand. The EG2 driver already handles absolute
        # opening targets; a second router/coordinator can retain an old
        # opening target or toggle out of phase with the arm.
        if 'gripper' in paths:
            coordinator = tomllib.loads(paths['coordinator'].read_text())
            coordinator['members'] = coordinator['members'][:1]
            coordinator['routes'] = coordinator['routes'][:1]
            paths['coordinator'].write_text(_toml(coordinator))
            paths.pop('gripper').unlink()
        return paths


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
        self.gripper_max_width = settings.robot.gripper_max_width
        self.home_q = np.asarray(settings.robot.park_q, dtype=float)
        self.home_released = self.returning_home = self.home_blocked = False
        self.home_since = None

    def poll(self, state, now, epoch, selected):
        candidate = super().poll(state, now, epoch, selected)
        now = time.monotonic()
        sample = self.samples['input']
        valid = selected and self.status['ready'] and sample.fresh(now) and not candidate.fault
        was_returning = self.returning_home
        if epoch != self.gripper_epoch or not valid:
            self.gripper_epoch = epoch
            self.gripper_released = self.gripper_armed = False
            self.home_released = self.returning_home = self.home_blocked = False
            self.home_since = None
        command = candidate.command
        home = secondary = False
        if valid:
            inp = sample.sample
            neutral = inp.trigger < .2 and inp.grip < .2 and not (inp.buttons_bitset & 12)
            if neutral:
                self.home_released = True
                self.home_blocked = False
            secondary = bool(inp.buttons_bitset & 8)
            if (secondary and self.home_released and not self.home_blocked and
                    not (inp.buttons_bitset & 4) and inp.trigger < .2 and inp.grip < .2):
                if self.home_since is None:
                    self.home_since = now
                home = now-self.home_since >= 1.
            else:
                if self.home_since is not None and not neutral:
                    self.home_released = False
                    self.home_blocked = True
                self.home_since = None
            if (state.gripper_width is None or secondary or self.home_blocked or was_returning
                    or command.mode != Mode.COMMAND_FOLLOWING or not self.status['permit']):
                self.gripper_released = self.gripper_armed = False
            elif inp.trigger < .2:
                self.gripper_released = True
            elif self.gripper_released and inp.trigger > .2:
                self.gripper_armed = True
        # Same absolute mapping as the native coordinator's parallel_target.
        # Startup/recovery does not open a partly closed gripper: require a
        # neutral Trigger followed by a new press, then release opens normally.
        width = None
        if valid and self.gripper_armed:
            if not np.isfinite(inp.trigger) or not 0 <= inp.trigger <= 1:
                raise ValueError('Invalid Quest trigger')
            width = self.gripper_max_width * (1. - inp.trigger)
        command = replace(command, gripper_target=width, gripper_mode=command.mode)
        overriding = secondary or was_returning or self.home_blocked
        if overriding:
            # Use the existing bounded joint stream, not Disabled (ARX Home)
            # or a scene reset. B/Y must stay held; release/conflict stops Home.
            command = replace(command, mode=Mode.COMMAND_FOLLOWING, hold=not home,
                reset_reference=was_returning != home, pose_target=None,
                joint_target=self.home_q.copy() if home else None,
                joint_velocity_target=None, joint_acceleration_target=None, planned=False,
                engaged=False, gripper_target=None, gripper_initialize_open=False)
        self.returning_home = home
        self.status.update(gripper_ready=state.gripper_width is not None,
            gripper_armed=self.gripper_armed, returning_home=home)
        return replace(candidate, command=command,
                       phase='home' if home else 'hold' if overriding else candidate.phase)
