from robot_teleop.core.state_machine import (
    TeleopEvent,
    TeleopState,
    TeleopStateMachine,
)


def test_nominal_state_transitions() -> None:
    machine = TeleopStateMachine()

    assert machine.transition(TeleopEvent.CONNECT) is TeleopState.READY
    assert machine.transition(TeleopEvent.ACTIVATE) is TeleopState.ACTIVE
    assert machine.transition(TeleopEvent.RELEASE) is TeleopState.HOLD


def test_tracking_limited_and_unavailable_cannot_remain_active() -> None:
    machine = TeleopStateMachine()
    machine.transition(TeleopEvent.CONNECT)
    machine.transition(TeleopEvent.ACTIVATE)

    assert machine.transition(TeleopEvent.TRACKING_LIMITED) is TeleopState.DEGRADED
    assert machine.transition(TeleopEvent.TRACKING_UNAVAILABLE) is TeleopState.HOLD


def test_estop_is_latched_until_explicit_reset() -> None:
    machine = TeleopStateMachine()
    machine.transition(TeleopEvent.CONNECT)
    machine.transition(TeleopEvent.ESTOP)

    assert machine.transition(TeleopEvent.ACTIVATE) is TeleopState.ESTOP
    assert machine.transition(TeleopEvent.TRACKING_NORMAL) is TeleopState.ESTOP
    assert machine.transition(TeleopEvent.RESET) is TeleopState.READY


def test_disconnected_latched_state_resets_to_disconnected() -> None:
    machine = TeleopStateMachine()
    machine.transition(TeleopEvent.CONNECT)
    machine.transition(TeleopEvent.FAULT)
    machine.transition(TeleopEvent.DISCONNECT)

    assert machine.transition(TeleopEvent.RESET) is TeleopState.DISCONNECTED
