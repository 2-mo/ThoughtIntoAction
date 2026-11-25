"""Reuse native Quest routing with a single input owner and per-arm namespaces."""
import json
from pathlib import Path
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


def _vendor_basis(settings):
    """A vendor profile with its own XR-to-base rotation overrides the X5 basis."""
    path = getattr(settings.robot, 'connection_path', None)
    if not path:
        return None
    try:
        return json.loads(Path(path).read_text())['xr_to_robot_basis']
    except (OSError, ValueError, KeyError, TypeError):
        return None


class SharedDevice(QuestDevice):
    @staticmethod
    def commands(configs, prefix):
        return {}  # The workbench pair supervisor owns the single input process.


class OwnedDevice(QuestDevice):
    """One kernel owns its own XR input process (no pair supervisor around it)."""

    @staticmethod
    def commands(configs, prefix):
        return QuestDevice.commands(configs, prefix)


class Routing(NativeRouting):
    @staticmethod
    def configure(directory, prefix, settings):
        vr = settings.vr
        paths = write_native_configs(directory, prefix, settings.robot.gripper_max_width,
            settings.sdk_library, vr.base_yaw_deg, vr.grip_threshold, vr.position_scale,
            controller=vr.controller, input_prefix=vr.input_prefix or prefix,
            feedback_hz=settings.robot.control_hz,
            cartesian_speed_limits=(settings.robot.teleop_linear_speed_m_s,
                                    settings.robot.teleop_angular_speed_rad_s),
            # Two kernels and one XR owner share the bus here; a 100 ms window
            # trips on a single dropped burst and drops the group to Disabled.
            input_timeout_ms=300, xr_basis=_vendor_basis(settings))
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
    def __init__(self, prefix, dependencies, gripper=True, *, dual=False, settings=None):
        super().__init__(prefix, dependencies, gripper=False, dual=dual)
        from embodied.control.ipc.services import open_service, make_subscriber
        from embodied.control.ipc.types import XrControllerInput
        vr = settings.vr
        # Every kernel reads the hand it follows: A arms this side and its
        # Trigger owns this side's gripper (the coordinator also keeps a
        # per-side mode, so reading one hand for both would drive both arms
        # and both grippers from a single controller).
        self.subscribers['input'] = make_subscriber(open_service(self.node,
            f'{vr.input_prefix or prefix}/xr/{vr.controller}_controller/states/input', XrControllerInput))
        self.gripper_released = self.gripper_armed = False
        self.gripper_epoch = None
        self.gripper_max_width = settings.robot.gripper_max_width
        self.wrist_limit = settings.robot.joint_min[5]
        self.wrist_step = .35 / settings.robot.control_hz
        self.wrist_ready = False
        self.wrist_target = None

    def poll(self, state, now, epoch, selected):
        candidate = super().poll(state, now, epoch, selected)
        now = time.monotonic()
        sample = self.samples['input']
        valid = selected and self.status['ready'] and sample.fresh(now) and not candidate.fault
        was_turning = self.wrist_target is not None
        if epoch != self.gripper_epoch or not valid:
            self.gripper_epoch = epoch
            self.gripper_released = self.gripper_armed = False
            self.wrist_ready = False
            self.wrist_target = None
        command = candidate.command
        permitted = valid and self.status['permit'] and command.mode == Mode.COMMAND_FOLLOWING
        inp = sample.sample
        secondary = valid and bool(inp.buttons_bitset & 8)
        if not permitted:
            self.wrist_ready = False
        elif not secondary:
            self.wrist_ready = True
        elif not (inp.grip < .2) or inp.buttons_bitset & 4:
            self.wrist_ready = False
        if not secondary or not self.wrist_ready:
            self.wrist_target = None
        elif self.wrist_target is None:
            self.wrist_target = state.q.copy()
            self.wrist_target[5] = self.wrist_limit
        if not permitted or state.gripper_width is None:
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
        turning = self.wrist_target is not None
        overriding = secondary or was_turning
        if overriding:
            command = replace(command, hold=not turning, reset_reference=was_turning != turning,
                pose_target=None, joint_target=self.wrist_target,
                joint_step_limit=self.wrist_step if turning else None,
                joint_velocity_target=None, joint_acceleration_target=None, planned=False,
                engaged=turning, gripper_initialize_open=False)
        self.status.update(gripper_ready=state.gripper_width is not None,
            gripper_armed=self.gripper_armed, turning_wrist=turning)
        return replace(candidate, command=command,
                       phase='teleop' if turning else 'hold' if overriding else candidate.phase)
