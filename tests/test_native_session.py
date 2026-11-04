"""Native source startup waits only for settling; faults never reacquire control."""
import json
from unittest.mock import Mock, patch

import pytest

from embodied.devices.inputs.idle import NativeSession


SETTLING = 'ValueError: Robot has not settled: waiting for continuous low-motion feedback'


def run_source(directory, sessions, after_sleep=None):
    now = [0.]

    def sleep(seconds):
        now[0] += seconds
        if after_sleep:
            after_sleep(now[0])

    with patch('embodied.devices.inputs.idle.ControlSession', side_effect=sessions) as factory, \
         patch('embodied.devices.inputs.idle.time.monotonic', side_effect=lambda: now[0]), \
         patch('embodied.devices.inputs.idle.time.sleep', side_effect=sleep):
        NativeSession(directory).run(duration=.2)
    return factory


def session(error=None):
    value = Mock(error=None)
    value.__enter__ = Mock(side_effect=error)
    return value


def test_waits_for_continuous_standstill_then_keeps_one_lease(tmp_path):
    attempts = [session(RuntimeError(SETTLING)), session(RuntimeError(SETTLING)), session()]
    factory = run_source(tmp_path, attempts)
    assert factory.call_count == 3
    factory.assert_called_with(tmp_path, source='quest', ingress='native', purpose='teleop')
    for attempt in attempts:
        attempt.__enter__.assert_called_once()
        attempt.close.assert_called_once()


@pytest.mark.parametrize('reason', ['Control is already owned; release it first',
                                  'Runtime stop is latched; restart explicitly',
                                  'Requested command ingress is not installed'])
def test_other_acquire_failures_are_not_retried(tmp_path, reason):
    failed = session(RuntimeError('ValueError: '+reason))
    with pytest.raises(RuntimeError, match=reason):
        run_source(tmp_path, [failed])
    failed.close.assert_called_once()


def test_disabling_quest_while_waiting_prevents_acquisition(tmp_path):
    failed = session(RuntimeError(SETTLING))
    factory = run_source(tmp_path, [failed], after_sleep=lambda _: (
        tmp_path/'native-source.json').write_text(json.dumps({'enabled': False})))
    assert factory.call_count == 1


def test_lost_lease_fails_without_reacquiring(tmp_path):
    acquired = session()
    acquired.error = 'native input stale'
    with pytest.raises(RuntimeError, match='native input stale'):
        run_source(tmp_path, [acquired])
    acquired.close.assert_called_once()
