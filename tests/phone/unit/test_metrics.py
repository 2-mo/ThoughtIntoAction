import pytest

from embodied.devices.phone.runtime.metrics import CycleTiming, RuntimeMetrics


def test_metrics_report_contains_latency_percentiles_and_counts() -> None:
    metrics = RuntimeMetrics(sample_capacity=8)
    for index, compute_ns in enumerate((1_000_000, 2_000_000, 3_000_000)):
        metrics.record_cycle(
            CycleTiming(
                expected_deadline_ns=index * 10,
                actual_start_ns=index * 10 + 1,
                actual_end_ns=index * 10 + 1 + compute_ns,
                deadline_miss=index == 2,
                deadline_rebased=False,
            )
        )
    metrics.record_input_accepted()
    metrics.record_input_rejected("sequence")
    metrics.record_packet_age(12.5, "fresh")
    metrics.record_input_interval(16.0)

    report = metrics.report()

    assert report["cycle_count"] == 3
    assert report["deadline_misses"] == 1
    assert report["consecutive_deadline_misses"] == 1
    assert report["maximum_consecutive_deadline_misses"] == 1
    assert report["compute_ms"]["median"] == 2.0
    assert report["compute_ms"]["p95"] == 3.0
    assert report["packet_age_ms"]["median"] == 12.5
    assert report["sample_window"] == {
        "capacity": 8,
        "cycle_timing": {"sample_count": 3, "scope": "all_session"},
        "packet_age": {"sample_count": 1, "scope": "all_observed"},
        "input_interval": {"sample_count": 1, "scope": "all_observed"},
    }
    assert report["watchdog_stages"] == {"fresh": 1}
    assert report["accepted_inputs"] == 1
    assert report["rejected_inputs"] == 1
    assert report["input_rejections"] == {"sequence": 1}


def test_protocol_events_and_timings_are_bounded_and_reported() -> None:
    metrics = RuntimeMetrics(sample_capacity=2)
    metrics.record_protocol_event("udp.datagram", 3)
    metrics.record_protocol_event("udp.superseded", 2)
    for value in (1.0, 2.0, 3.0):
        metrics.record_protocol_timing("rtt", value)

    report = metrics.report()

    assert report["protocol_events"] == {
        "udp.datagram": 3,
        "udp.superseded": 2,
    }
    assert report["protocol_timings_ms"]["rtt"]["median"] == 2.5
    assert report["protocol_timings_ms"]["rtt"]["p99"] == 3.0


def test_metric_report_identifies_a_recent_window_after_capacity_is_exceeded() -> None:
    metrics = RuntimeMetrics(sample_capacity=2)
    for index in range(3):
        metrics.record_cycle(
            CycleTiming(
                expected_deadline_ns=index * 10,
                actual_start_ns=index * 10,
                actual_end_ns=index * 10 + 1,
                deadline_miss=False,
                deadline_rebased=False,
            )
        )
        metrics.record_packet_age(float(index), "fresh")

    report = metrics.report()

    sample_window = report["sample_window"]
    assert sample_window == {
        "capacity": 2,
        "cycle_timing": {"sample_count": 2, "scope": "recent_window"},
        "packet_age": {"sample_count": 2, "scope": "recent_window"},
        "input_interval": {"sample_count": 0, "scope": "all_observed"},
    }


def test_protocol_metric_values_are_validated() -> None:
    metrics = RuntimeMetrics()
    with pytest.raises(ValueError, match="event"):
        metrics.record_protocol_event("", 1)
    with pytest.raises(ValueError, match="count"):
        metrics.record_protocol_event("event", 0)
    with pytest.raises(ValueError, match="timing"):
        metrics.record_protocol_timing("rtt", -1.0)
    with pytest.raises(ValueError, match="interval"):
        metrics.record_input_interval(-1.0)
