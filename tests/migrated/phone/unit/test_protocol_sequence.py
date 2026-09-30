import pytest

from robot_teleop.protocol.sequence import (
    SequenceError,
    SequenceErrorCode,
    SequenceTracker,
)


def test_first_packet_and_forward_progress_are_accepted() -> None:
    tracker = SequenceTracker()

    first = tracker.observe(session_id=10, sequence=40, timestamp_ns=100)
    next_packet = tracker.observe(session_id=10, sequence=41, timestamp_ns=101)

    assert not first.session_changed
    assert first.lost_packets == 0
    assert not next_packet.session_changed
    assert next_packet.lost_packets == 0


def test_forward_gap_is_accepted_and_reports_estimated_loss() -> None:
    tracker = SequenceTracker()
    tracker.observe(session_id=1, sequence=4, timestamp_ns=10)

    result = tracker.observe(session_id=1, sequence=9, timestamp_ns=20)

    assert result.lost_packets == 4


def test_uint32_sequence_wrap_is_forward_progress() -> None:
    tracker = SequenceTracker()
    tracker.observe(session_id=1, sequence=0xFFFFFFFF, timestamp_ns=10)

    result = tracker.observe(session_id=1, sequence=0, timestamp_ns=20)

    assert result.lost_packets == 0


@pytest.mark.parametrize(
    ("sequence", "code"),
    [
        (100, SequenceErrorCode.DUPLICATE),
        (99, SequenceErrorCode.OUT_OF_ORDER),
        ((100 + (1 << 31)) & 0xFFFFFFFF, SequenceErrorCode.OUT_OF_ORDER),
    ],
)
def test_duplicate_and_old_sequences_are_rejected(
    sequence: int,
    code: SequenceErrorCode,
) -> None:
    tracker = SequenceTracker()
    tracker.observe(session_id=1, sequence=100, timestamp_ns=10)

    with pytest.raises(SequenceError) as error:
        tracker.observe(session_id=1, sequence=sequence, timestamp_ns=20)

    assert error.value.code is code


def test_timestamp_must_increase_inside_session() -> None:
    tracker = SequenceTracker()
    tracker.observe(session_id=1, sequence=1, timestamp_ns=10)

    with pytest.raises(SequenceError) as error:
        tracker.observe(session_id=1, sequence=2, timestamp_ns=10)

    assert error.value.code is SequenceErrorCode.TIMESTAMP


def test_new_session_resets_ordering_and_retires_previous_session() -> None:
    tracker = SequenceTracker()
    tracker.observe(session_id=10, sequence=100, timestamp_ns=1_000)

    result = tracker.observe(session_id=20, sequence=0, timestamp_ns=5)

    assert result.session_changed
    assert result.lost_packets == 0
    with pytest.raises(SequenceError) as error:
        tracker.observe(session_id=10, sequence=101, timestamp_ns=1_001)
    assert error.value.code is SequenceErrorCode.RETIRED_SESSION


def test_retired_session_history_is_bounded() -> None:
    tracker = SequenceTracker(retired_session_capacity=2)
    tracker.observe(session_id=1, sequence=0, timestamp_ns=1)
    tracker.observe(session_id=2, sequence=0, timestamp_ns=1)
    tracker.observe(session_id=3, sequence=0, timestamp_ns=1)
    tracker.observe(session_id=4, sequence=0, timestamp_ns=1)

    result = tracker.observe(session_id=1, sequence=0, timestamp_ns=1)

    assert result.session_changed


@pytest.mark.parametrize(
    ("session_id", "sequence", "timestamp_ns"),
    [
        (0, 1, 1),
        (1 << 32, 1, 1),
        (1, -1, 1),
        (1, 1 << 32, 1),
        (1, 1, -1),
        (True, 1, 1),
    ],
)
def test_wire_integer_ranges_are_validated(
    session_id: int,
    sequence: int,
    timestamp_ns: int,
) -> None:
    tracker = SequenceTracker()

    with pytest.raises(ValueError, match=r"session_id|sequence|timestamp_ns"):
        tracker.observe(
            session_id=session_id,
            sequence=sequence,
            timestamp_ns=timestamp_ns,
        )
