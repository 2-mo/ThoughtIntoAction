from robot_teleop.core.models import (
    Pose,
    Quaternion,
    TeleopInput,
    TrackingQuality,
    Vector3,
)
from robot_teleop.protocol.packet import EStopResetRequestPacket, SourceType
from robot_teleop.runtime.estop_reset import EStopResetGate


def safe_release(
    *,
    session_id: int = 7,
    timestamp_ns: int = 200,
    tracking: TrackingQuality = TrackingQuality.NORMAL,
    clutch_active: bool = False,
    estop_requested: bool = False,
) -> TeleopInput:
    return TeleopInput(
        sequence=2,
        session_id=session_id,
        timestamp_ns=timestamp_ns,
        pose=Pose(Vector3(0.0, 0.0, 0.0), Quaternion(0.0, 0.0, 0.0, 1.0)),
        linear_velocity=None,
        angular_velocity=None,
        clutch_active=clutch_active,
        recenter_requested=False,
        estop_requested=estop_requested,
        gripper=None,
        tracking_quality=tracking,
        source_name="arkit",
    )


def reset_request() -> EStopResetRequestPacket:
    return EStopResetRequestPacket(
        source_type=SourceType.ARKIT,
        sequence=3,
        session_id=7,
        client_time_ns=100,
    )


def test_reset_requires_matching_request_and_later_safe_release() -> None:
    gate = EStopResetGate()
    request = reset_request()

    assert not gate.record_request(request)

    assert gate.pending_request == request
    assert gate.ready_request is None

    gate.observe_input(safe_release(timestamp_ns=101))

    assert gate.ready_request == request
    assert gate.completed_request is None

    gate.complete(request)

    assert gate.pending_request is None
    assert gate.ready_request is None
    assert gate.completed_request == request
    assert gate.record_request(request)


def test_reset_does_not_become_ready_for_an_unsafe_or_other_session_pose() -> None:
    gate = EStopResetGate()
    request = reset_request()

    assert not gate.record_request(request)
    gate.observe_input(safe_release(session_id=8, timestamp_ns=200))
    assert gate.pending_request is None

    assert not gate.record_request(request)
    gate.observe_input(safe_release(timestamp_ns=101, clutch_active=True))
    assert gate.ready_request is None

    gate.observe_input(safe_release(timestamp_ns=102, estop_requested=True))
    assert gate.ready_request is None

    gate.observe_input(safe_release(timestamp_ns=103, tracking=TrackingQuality.LIMITED))
    assert gate.ready_request is None
