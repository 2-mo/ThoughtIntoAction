"""Two single-arm backends behind one 12-joint state.

Each arm keeps its own SDK controller, CAN interface and MuJoCo data; this
composite only concatenates feedback and splits commands per arm.
"""
import numpy as np

from embodied.contracts.control import ControlCommand, Mode, RobotState


def combined_mode(modes):
    """A dual arm is as permissive as its most restrictive arm."""
    modes = tuple(modes)
    if Mode.DISABLED in modes:
        return Mode.DISABLED
    if Mode.FREE_DRIVE in modes:
        return Mode.FREE_DRIVE
    return Mode.COMMAND_FOLLOWING


class DualBackend:
    def __init__(self, profile, arms):
        self.profile = profile
        self.arms = tuple(arms)
        if len(self.arms) != 2:
            raise ValueError('A dual backend requires exactly two arm backends')

    def _split(self, vector):
        if vector is None:
            return (None, None)
        values = np.asarray(vector)
        if values.shape[-1] == self.profile.joint_count:
            half = self.profile.left.joint_count
            return values[:half], values[half:]
        return values, values

    def arm_commands(self, command):
        poses = command.pose_targets
        grippers = command.gripper_targets
        joints = self._split(command.joint_target)
        velocities = self._split(command.joint_velocity_target)
        accelerations = self._split(command.joint_acceleration_target)
        limits = self._split(command.joint_step_limit)
        for index in range(2):
            pose = poses[index] if poses is not None else None
            yield ControlCommand(
                mode=command.mode,
                pose_target=pose,
                gripper_target=grippers[index] if grippers is not None else None,
                hold=command.hold and pose is None,
                engaged=command.engaged,
                reason=command.reason,
                gripper_mode=command.gripper_mode,
                joint_target=joints[index],
                joint_step_limit=limits[index],
                reset_reference=command.reset_reference,
                joint_velocity_target=velocities[index],
                planned=command.planned,
                gripper_effort=command.gripper_effort,
                joint_acceleration_target=accelerations[index],
            )

    def read_state(self, now):
        # Serial by measurement: concurrent SDK calls were slower (93 Hz vs 150 Hz).
        states = [arm.read_state(now) for arm in self.arms]
        return RobotState(
            q=np.concatenate([state.q for state in states]),
            dq=np.concatenate([state.dq for state in states]),
            pose=states[0].pose,
            gripper_width=states[0].gripper_width,
            received_at=now,
            mode=combined_mode(state.mode for state in states),
            fault='; '.join(filter(None, (state.fault for state in states))),
            gripper_mode=states[0].gripper_mode,
            gripper_velocity=states[0].gripper_velocity,
            gripper_effort=states[0].gripper_effort,
            gripper_contact=any(state.gripper_contact for state in states),
            poses=tuple(state.pose for state in states),
            gripper_widths=tuple(state.gripper_width for state in states),
        )

    def step(self, command, now, **options):
        for arm, arm_command in zip(self.arms, self.arm_commands(command)):
            arm.step(arm_command, now, **options)

    @property
    def controller(self):
        """Tuning and diagnostics read the primary arm's controller."""
        return self.arms[0].controller

    @property
    def joint_goal(self):
        """IK goal in wire order: one per arm, or None when either arm has none.

        Telemetry copies this as the 12-wide `joint_goal`, so falling back to the
        primary controller's 6-wide goal would be rejected.
        """
        goals = [getattr(arm.controller, '_joint_goal', None) for arm in self.arms]
        if any(goal is None for goal in goals):
            return None
        return np.concatenate([np.asarray(goal, dtype=float) for goal in goals])

    def telemetry(self):
        return {'arms': [arm.telemetry() for arm in self.arms]}

    def runtime_parameters(self):
        return {'backend': 'dual', 'arms': [arm.runtime_parameters() for arm in self.arms]}

    def request_stop(self):
        for arm in self.arms:
            arm.request_stop()

    def close(self):
        for arm in self.arms:
            arm.close()


class DualKinematics:
    """Per-arm kinematics exposed as one 12-joint model.

    Pose commands currently plan on the left arm and pass the right joints
    through unchanged; per-arm pose routing arrives with the command layer.
    """

    def __init__(self, config):
        from embodied.devices.robots.arx5.kinematics import X5Model

        self.config = config
        self.models = tuple(X5Model(arm) for arm in config.arms)
        self.lower = np.asarray(config.joint_min, dtype=float)
        self.upper = np.asarray(config.joint_max, dtype=float)
        self.half = config.left.joint_count

    def fk(self, q):
        return self.models[0].fk(np.asarray(q, dtype=float)[: self.half])

    def solve_ik(self, pose, seed):
        seed = np.asarray(seed, dtype=float)
        left, converged, residual = self.models[0].solve_ik(pose, seed[: self.half])
        return np.concatenate((left, seed[self.half :])), converged, residual
