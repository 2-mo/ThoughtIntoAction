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
from embodied.control.ipc.services import make_subscriber, open_service
from embodied.control.ipc.types import XrControllerInput
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
            paths.pop('gripper').unlink()
            paths['coordinator'].write_text(_toml(coordinator))
        return paths


class Ingress(NativeIngress):
    def __init__(self, prefix, dependencies, gripper=True, *, dual=False, settings=None):
        super().__init__(prefix, dependencies, gripper=False, dual=dual)
        vr = settings.vr
        # The primary button (left X or right A) and Trigger belong to this hand.
        self.subscribers['input'] = make_subscriber(open_service(self.node,
            f'{vr.input_prefix or prefix}/xr/{vr.controller}_controller/states/input', XrControllerInput))
        self.gripper_released = self.gripper_armed = False
        self.gripper_epoch = None
        self.gripper_max_width = settings.robot.gripper_max_width
        self.home_q = np.asarray(settings.robot.park_q, dtype=float)
        self.home_released = self.returning_home = self.home_blocked = False
        self.home_since = None
        self.wrist_ready = False
        self.wrist_target = None
        self.wrist_limits = settings.robot.joint_min[5], settings.robot.joint_max[5]
        self.wrist_step = .35 / settings.robot.control_hz

    def poll(self, state, now, epoch, selected):
        candidate = super().poll(state, now, epoch, selected)
        now = time.monotonic()
        sample = self.samples['input']
        was_turning = self.wrist_target is not None
        valid = selected and self.status['ready'] and sample.fresh(now) and not candidate.fault
        was_returning = self.returning_home
        new_epoch = epoch != self.gripper_epoch
        if new_epoch or not valid:
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
            # B (right) / Y (left) is the vendor's Home button; A+B also works,
            # which is the habit from the ARX profiles.
            if (secondary and self.home_released and not self.home_blocked and
                    inp.trigger < .2 and inp.grip < .2):
                if self.home_since is None:
                    self.home_since = now
                home = now-self.home_since >= 1.
            else:
                if self.home_since is not None and not neutral:
                    self.home_released = False
                    self.home_blocked = True
                self.home_since = None
            if (state.gripper_width is None or home
                    or command.mode != Mode.COMMAND_FOLLOWING or not self.status['permit']):
                self.gripper_released = self.gripper_armed = False
            elif inp.trigger < .2:
                self.gripper_released = True
            elif self.gripper_released and inp.trigger > .2:
                self.gripper_armed = True
        # Startup/recovery does not open a partly closed gripper: require a
        # neutral Trigger followed by a new press, then release opens normally.
        axis = inp.joystick_x if valid else 0.
        wrist_permitted = (valid and not (inp.buttons_bitset & 12)
                           and self.status['permit'] and command.mode == Mode.COMMAND_FOLLOWING
                           and not self.home_blocked and not was_returning)
        if not wrist_permitted or new_epoch or not np.isfinite(axis) or abs(axis) > 1.:
            self.wrist_ready = False
        elif abs(axis) <= .2:
            self.wrist_ready = True
        if not self.wrist_ready or abs(axis) <= .2:
            self.wrist_target = None
        else:
            if not was_turning:
                self.wrist_target = state.q.copy()
            self.wrist_target[5] = self.wrist_limits[int(axis > 0)]
        turning = self.wrist_target is not None
        width = None
        if valid and self.gripper_armed:
            if not np.isfinite(inp.trigger) or not 0 <= inp.trigger <= 1:
                raise ValueError('Invalid Quest trigger')
            # Treat the final 2% of Trigger travel as fully pressed.
            width = 0. if inp.trigger >= .98 else self.gripper_max_width * (1. - inp.trigger)
        command = replace(command, gripper_target=width, gripper_mode=command.mode)
        overriding = secondary or was_returning or self.home_blocked
        if overriding:
            # Use the existing bounded joint stream, not Disabled (ARX Home)
            # or a scene reset. B/Y must stay held; release/conflict stops Home.
            command = replace(command, mode=Mode.COMMAND_FOLLOWING, hold=not home,
                reset_reference=was_returning != home, pose_target=None,
                joint_target=self.home_q.copy() if home else None,
                joint_velocity_target=None, joint_acceleration_target=None, planned=False,
                engaged=False, gripper_initialize_open=False)
        if turning or (was_turning and not overriding):
            command = replace(command, hold=not turning, reset_reference=was_turning != turning,
                pose_target=None, joint_target=self.wrist_target.copy() if turning else None,
                joint_step_limit=self.wrist_step * ((abs(axis) - .2) / .8) if turning else None, engaged=turning)
        self.returning_home = home
        self.status.update(gripper_ready=state.gripper_width is not None,
            gripper_armed=self.gripper_armed, returning_home=home, turning_wrist=turning)
        return replace(candidate, command=command,
                       phase='home' if home else 'teleop' if turning else 'hold' if overriding or was_turning else candidate.phase)
