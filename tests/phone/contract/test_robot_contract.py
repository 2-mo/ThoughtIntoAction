import pytest

from embodied.phone.core.models import (
    ControlMode,
    Pose,
    Quaternion,
    RobotCommand,
    Vector3,
)
from embodied.phone.robots.mock import (
    MockRobotBackend,
    MockRobotConfig,
    RobotConnectionError,
    RobotSafetyError,
)


def command(
    mode: ControlMode,
    *,
    pose: Pose | None = None,
    timestamp_ns: int = 1,
) -> RobotCommand:
    return RobotCommand(
        mode=mode,
        target_pose=pose,
        target_twist=None,
        target_joint_position=None,
        gripper=None,
        velocity_scale=1.0 if mode is ControlMode.CARTESIAN_POSITION else 0.0,
        timestamp_ns=timestamp_ns,
    )


def test_connect_disconnect_are_idempotent_and_state_is_canonical() -> None:
    robot = MockRobotBackend()
    robot.connect()
    robot.connect()

    state = robot.state()

    assert len(state.joint_position) == robot.capabilities.joint_count
    assert len(state.joint_velocity) == robot.capabilities.joint_count
    robot.disconnect()
    robot.disconnect()
    with pytest.raises(RobotConnectionError):
        robot.state()


def test_unconnected_robot_cannot_receive_command() -> None:
    robot = MockRobotBackend()

    with pytest.raises(RobotConnectionError):
        robot.send(command(ControlMode.HOLD))


def test_hold_stop_and_cartesian_position_are_executable() -> None:
    robot = MockRobotBackend()
    robot.connect()
    initial = robot.state()
    target_pose = Pose(
        initial.tcp_pose.position + Vector3(0.01, 0.0, 0.0),
        Quaternion(0.0, 0.0, 0.0, 1.0),
    )

    moved = robot.send(command(ControlMode.CARTESIAN_POSITION, pose=target_pose))
    held = robot.send(command(ControlMode.HOLD, timestamp_ns=2))
    stopped = robot.send(command(ControlMode.STOP, timestamp_ns=3))

    assert moved.tcp_pose == target_pose
    assert held.tcp_pose == target_pose
    assert all(value == 0.0 for value in stopped.joint_velocity)
    assert robot.hold_count == 1
    assert robot.stop_count == 1


def test_backend_rejects_excessive_target_and_enters_safe_stop() -> None:
    robot = MockRobotBackend(MockRobotConfig(maximum_cartesian_step_m=0.02))
    robot.connect()
    initial = robot.state()
    unsafe = Pose(
        initial.tcp_pose.position + Vector3(0.1, 0.0, 0.0),
        initial.tcp_pose.orientation,
    )

    with pytest.raises(RobotSafetyError, match="step"):
        robot.send(command(ControlMode.CARTESIAN_POSITION, pose=unsafe))

    assert robot.safe_stopped
    assert robot.safe_stop_count == 1
    with pytest.raises(RobotSafetyError, match="safe-stopped"):
        robot.send(command(ControlMode.HOLD))


def test_safe_stop_and_context_manager_cleanup() -> None:
    robot = MockRobotBackend()

    with robot as connected:
        connected.safe_stop("contract test")
        connected.safe_stop("duplicate")
        assert connected.safe_stopped
        assert connected.safe_stop_count == 1

    assert not robot.connected
