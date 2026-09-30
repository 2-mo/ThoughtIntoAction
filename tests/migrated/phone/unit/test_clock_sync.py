import pytest

from robot_teleop.protocol.clock_sync import (
    ClockSampleError,
    ClockSyncEstimator,
    clock_sample,
)


def test_ntp_four_timestamp_sample_computes_rtt_and_offset() -> None:
    sample = clock_sample(
        client_send_time_ns=1_000,
        server_receive_time_ns=1_600,
        server_send_time_ns=1_700,
        client_receive_time_ns=1_300,
    )

    assert sample.rtt_ns == 200
    assert sample.server_minus_client_ns == 500.0


@pytest.mark.parametrize(
    ("t0", "t1", "t2", "t3"),
    [
        (10, 20, 30, 9),
        (10, 30, 20, 40),
        (10, 20, 50, 30),
        (-1, 20, 30, 40),
        (True, 20, 30, 40),
    ],
)
def test_invalid_clock_intervals_are_rejected(
    t0: int,
    t1: int,
    t2: int,
    t3: int,
) -> None:
    with pytest.raises((ClockSampleError, ValueError)):
        clock_sample(t0, t1, t2, t3)


def test_estimator_rejects_excessive_rtt() -> None:
    estimator = ClockSyncEstimator(max_rtt_ns=100, sample_capacity=8)

    accepted = estimator.add_exchange(0, 50, 50, 101)

    assert not accepted
    assert estimator.sample_count == 0
    assert estimator.rejected_samples == 1


def test_estimate_uses_median_offset_from_lowest_rtt_samples() -> None:
    estimator = ClockSyncEstimator(
        max_rtt_ns=1_000,
        sample_capacity=8,
        low_rtt_sample_count=3,
    )
    # (RTT, offset): (20, 100), (10, 101), (30, 99), (400, 900)
    assert estimator.add_exchange(0, 110, 110, 20)
    assert estimator.add_exchange(0, 106, 106, 10)
    assert estimator.add_exchange(0, 114, 114, 30)
    assert estimator.add_exchange(0, 1_100, 1_100, 400)

    assert estimator.offset_ns == 100.0
    assert estimator.minimum_rtt_ns == 10
    assert estimator.to_server_time_ns(1_000) == 1_100


def test_estimator_storage_is_bounded_and_resettable() -> None:
    estimator = ClockSyncEstimator(
        max_rtt_ns=1_000,
        sample_capacity=2,
        low_rtt_sample_count=2,
    )
    assert estimator.add_exchange(0, 10, 10, 10)
    assert estimator.add_exchange(0, 20, 20, 20)
    assert estimator.add_exchange(0, 30, 30, 30)

    assert estimator.sample_count == 2
    estimator.reset()
    assert estimator.sample_count == 0
    assert estimator.offset_ns is None
    assert estimator.minimum_rtt_ns is None


def test_estimator_configuration_is_validated() -> None:
    with pytest.raises(ValueError, match="max_rtt_ns"):
        ClockSyncEstimator(max_rtt_ns=0)
    with pytest.raises(ValueError, match="sample_capacity"):
        ClockSyncEstimator(sample_capacity=0)
    with pytest.raises(ValueError, match="cannot exceed"):
        ClockSyncEstimator(sample_capacity=2, low_rtt_sample_count=3)
