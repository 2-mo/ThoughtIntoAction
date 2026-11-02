import pytest
from embodied.devices.robots.realman.gripper import move, Buttons, Jog, GripperNotReady, SessionJog


class Client:
    def __init__(self):
        self.sent = []
        self.ready = True
        self.mode = 1
        self.pos = 1002
    def query(self, command):
        return dict(enable=1, status=int(self.ready), error=0, mode=self.mode,
                    current_force=0, temperature=34, actpos=self.pos)
    def exchange(self, message, expected):
        self.sent.append(message)
        assert expected == 'set_gripper'
        return {'command': expected, 'state': True}


def test_gripper_requires_online_and_uses_nonpersistent_force_pick():
    c = Client()
    assert move(c, 'close')['completed'] is False
    assert c.sent == [dict(command='set_gripper_pick', speed=100, force=50, block=False)]
    c.ready = False
    with pytest.raises(RuntimeError): move(c, 'open')
    assert len(c.sent) == 1
    with pytest.raises(ValueError): move(c, 'close', force=0)


def test_buttons_release_edges_conflicting_buttons_and_stale():
    b = Buttons()
    def update(a=False, trigger=0., fresh=True):
        return b.update(a, trigger, fresh=fresh)
    assert update(trigger=1.) is None
    assert update() is None
    assert update(trigger=1.) == 'close'
    assert update(trigger=1.) == 'close'
    update()
    assert update(a=True) == 'open'
    assert b.blocked_reason is None
    assert update(a=True, trigger=1.) is None
    assert b.blocked_reason == 'conflicting_buttons'
    update(fresh=False)
    assert b.blocked_reason == 'xr_stale'
    assert update(a=True) is None


def test_busy_is_not_a_terminal_failure():
    c = Client(); c.mode = 5
    assert move(c, 'close')['accepted'] is False
    assert c.sent == []


@pytest.mark.parametrize('step', [10, 50, 100])
def test_jog_release_and_reverse_wait_for_pending_segment(step):
    c = Client(); c.pos = 500; c.mode = 3
    jog = Jog(step=step)
    assert jog.update(c, 'open', 0.)['position_target_raw'] == 500 + step
    c.mode = 5; c.pos = 504
    assert jog.update(c, None, .2)['accepted'] is False
    assert len(c.sent) == 1  # Releasing never queues more motion.
    assert jog.update(c, 'close', .4)['accepted'] is False
    c.mode = 3
    assert jog.update(c, 'close', .6)['reason'] == 'awaiting position feedback'
    c.pos = 500 + step
    assert jog.update(c, 'close', .8)['position_target_raw'] == 500
    assert c.sent[-1] == dict(command='set_gripper_position', position=500, block=False)


def test_jog_contact_requires_release_and_boundaries():
    c = Client(); c.pos = 500; c.mode = 6
    jog = Jog()
    assert jog.update(c, 'close', 0.)['accepted'] is False
    c.mode = 3
    assert jog.update(c, 'close', .2)['accepted'] is False
    jog.update(c, None, .4)
    assert jog.update(c, 'close', .6)['accepted'] is True
    jog = Jog(); c.pos = 1002
    assert jog.update(c, 'open', 0.)['reason'] == 'position boundary'
    c.pos = 1
    assert jog.update(c, 'close', .2)['reason'] == 'position boundary'


def test_jog_stale_idle_and_persistent_busy_do_not_flood_commands():
    c = Client(); c.pos = 500; c.mode = 3
    jog = Jog(); jog.update(c, 'open', 0.)
    assert jog.update(c, 'open', .2)['reason'] == 'awaiting position feedback'
    assert len(c.sent) == 1
    c.mode = 5
    with pytest.raises(RuntimeError, match='5 s'):
        jog.update(c, 'close', 6.)
    assert len(c.sent) == 1


def test_unavailable_feedback_is_distinct_from_communication_failure():
    c = Client(); c.ready = False
    with pytest.raises(GripperNotReady) as caught:
        Jog().update(c, 'open', 0.)
    assert caught.value.feedback['status'] == 0
    assert c.sent == []
    def timeout(command):
        raise TimeoutError('connection stalled')
    c.query = timeout
    with pytest.raises(TimeoutError):
        Jog().update(c, 'open', 0.)


def test_initialization_is_explicit_open_only_and_rejects_real_faults():
    c = Client()
    c.query = lambda _: dict.fromkeys(('enable','status','error','mode','current_force','temperature','actpos'), 0)
    with pytest.raises(GripperNotReady):
        move(c, 'open')
    with pytest.raises(ValueError):
        move(c, 'close', initialize_open=True)
    assert move(c, 'open', initialize_open=True)['initializing_from_zero_feedback']
    assert c.sent == [dict(command='set_gripper_release',speed=100,block=False)]
    original = c.query
    c.query = lambda _: {**original(_), 'error': 16}
    with pytest.raises(GripperNotReady):
        move(c, 'open', initialize_open=True)
    assert len(c.sent) == 1


def test_recovery_requires_two_idle_samples_neutral_controls_and_new_press():
    c = Client(); c.pos = 500
    session = SessionJog(unavailable={'status':0}, step=10)
    assert session.update(c, None, 0., controls_neutral=True)['reason'].startswith('confirming')
    assert session.update(c, 'close', 1., controls_neutral=False) is None
    assert session.update(c, None, 2., controls_neutral=True)['reason'].startswith('confirming')
    c.mode = 5
    assert session.update(c, None, 3., controls_neutral=True)['reason'].startswith('waiting')
    c.mode = 3
    session.update(c, None, 4., controls_neutral=True)
    assert session.update(c, None, 5., controls_neutral=True)['enabled_for_session']
    assert c.sent == []
    assert session.jog.step == 10  # Recovery keeps the configured segment size.
    assert session.update(c, 'close', 5.1, controls_neutral=False)['position_target_raw'] == 490


def test_pending_fault_recovery_never_replays_pending_or_held_action():
    c = Client(); c.pos = 500
    session = SessionJog()
    session.update(c, 'close', 0., controls_neutral=False)
    c.ready = False
    assert not session.update(c, 'close', .2, controls_neutral=False)['enabled_for_session']
    c.ready = True; c.pos = 490
    assert session.update(c, 'close', 2., controls_neutral=False) is None
    session.update(c, None, 3., controls_neutral=True)
    assert session.update(c, None, 4., controls_neutral=True)['enabled_for_session']
    assert len(c.sent) == 1
    assert session.jog.pending is None


@pytest.mark.parametrize('invalid', ['mode', 'position', 'timeout'])
def test_recovery_does_not_hide_invalid_feedback_or_transport_errors(invalid):
    c = Client()
    if invalid == 'mode':
        c.mode = 99
    elif invalid == 'position':
        c.pos = 2000
    else:
        def timeout(command):
            raise TimeoutError('connection lost')
        c.query = timeout
    with pytest.raises((RuntimeError, TimeoutError)):
        SessionJog(unavailable={'status':0}).update(c, None, 0., controls_neutral=True)
    assert c.sent == []


@pytest.mark.parametrize('step', [0, 4, 101, 1000, 10.5, True])
def test_segment_size_rejects_invalid_values(step):
    with pytest.raises(ValueError):
        Jog(step=step)


def test_default_segment_reduces_full_travel_command_count_and_clamps_endpoint():
    counts = []
    for jog in (Jog(step=10), Jog()):
        c = Client(); c.pos = 1000; c.mode = 3
        for tick in range(101):
            report = jog.update(c, 'close', tick * .16)
            if report and report.get('accepted'):
                target = report['position_target_raw']
                assert 1 <= target <= 1000
                assert 0 < c.pos - target <= jog.step
                c.pos = target  # Synthetic completed feedback at the next poll.
            if c.pos == 1:
                break
        counts.append(len(c.sent))
        assert c.pos == 1
    assert counts == [100, 10]
