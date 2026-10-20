from dataclasses import replace

import pytest

from embodied.phone.core.capabilities import InputCapabilities
from embodied.phone.core.models import (
    Pose,
    Quaternion,
    TeleopInput,
    TrackingQuality,
    Vector3,
)
from embodied.phone.inputs.mock import InputSourceStateError, MockInputSource
from embodied.phone.inputs.replay import ReplayInputSource

CAPABILITIES = InputCapabilities(
    pose_6dof=True,
    linear_velocity=False,
    angular_velocity=False,
    analog_gripper=True,
    haptics=False,
    dual_hand=False,
)


def sample(
    sequence: int,
    *,
    session_id: int = 1,
    timestamp_ns: int | None = None,
    tracking: TrackingQuality = TrackingQuality.NORMAL,
) -> TeleopInput:
    return TeleopInput(
        sequence=sequence,
        session_id=session_id,
        timestamp_ns=sequence * 10 if timestamp_ns is None else timestamp_ns,
        pose=Pose(
            Vector3(sequence * 0.001, 0.0, 0.0),
            Quaternion(0.0, 0.0, 0.0, 2.0),
        ),
        linear_velocity=None,
        angular_velocity=None,
        clutch_active=False,
        recenter_requested=False,
        estop_requested=False,
        gripper=0.5,
        tracking_quality=tracking,
        source_name="contract",
    )


def test_mock_input_contract_and_latest_only_behavior() -> None:
    source = MockInputSource(CAPABILITIES)
    source.start()
    source.push(sample(1))
    source.push(sample(2, tracking=TrackingQuality.UNAVAILABLE))

    latest = source.latest()

    assert latest is not None
    assert latest.sequence == 2
    assert latest.timestamp_ns > 10
    assert latest.pose.orientation.as_tuple() == (0.0, 0.0, 0.0, 1.0)
    assert latest.tracking_quality is TrackingQuality.UNAVAILABLE
    assert source.capabilities == CAPABILITIES
    source.stop()
    assert source.latest() is None
    with pytest.raises(InputSourceStateError):
        source.push(sample(3))


def test_mock_input_accepts_sequence_reset_for_new_session() -> None:
    source = MockInputSource(CAPABILITIES)
    source.start()
    source.push(sample(100, session_id=1))

    source.push(sample(0, session_id=2, timestamp_ns=0))

    assert source.latest() == sample(0, session_id=2, timestamp_ns=0)


def test_mock_input_rejects_non_monotonic_sample() -> None:
    source = MockInputSource(CAPABILITIES)
    source.start()
    source.push(sample(2))

    with pytest.raises(ValueError, match="sequence"):
        source.push(sample(1, timestamp_ns=30))


class FakeClock:
    def __init__(self) -> None:
        self.now_ns = 1_000

    def __call__(self) -> int:
        return self.now_ns


def test_replay_input_contract_uses_source_timing_without_worker_thread() -> None:
    clock = FakeClock()
    source = ReplayInputSource(
        [sample(1, timestamp_ns=100), sample(2, timestamp_ns=200)],
        CAPABILITIES,
        clock=clock,
    )
    source.start()

    assert source.latest() == sample(1, timestamp_ns=100)
    clock.now_ns += 99
    assert source.latest() == sample(1, timestamp_ns=100)
    clock.now_ns += 1
    assert source.latest() == sample(2, timestamp_ns=200)
    source.stop()
    assert source.latest() is None


def test_replay_rejects_invalid_contract_before_start() -> None:
    with pytest.raises(ValueError, match="timestamp"):
        ReplayInputSource(
            [sample(1, timestamp_ns=200), sample(2, timestamp_ns=100)],
            CAPABILITIES,
        )

    no_gripper = replace(CAPABILITIES, analog_gripper=False)
    with pytest.raises(ValueError, match="gripper"):
        ReplayInputSource([sample(1)], no_gripper)
