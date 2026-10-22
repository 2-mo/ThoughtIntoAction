from embodied.devices.phone.runtime.latest_value import LatestValue


class FakeClock:
    def __init__(self) -> None:
        self.now = 100

    def __call__(self) -> int:
        return self.now


def test_latest_value_overwrites_without_queueing_history() -> None:
    clock = FakeClock()
    latest: LatestValue[str] = LatestValue(clock=clock)
    latest.store("old")
    clock.now = 200

    second = latest.store("new")

    assert second.generation == 2
    assert second.stored_at_ns == 200
    assert latest.load() == second
    assert latest.load().value == "new"  # type: ignore[union-attr]


def test_latest_value_clear_removes_snapshot() -> None:
    latest: LatestValue[int] = LatestValue()
    latest.store(1)

    latest.clear()

    assert latest.load() is None
