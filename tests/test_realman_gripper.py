import pytest
from embodied.devices.robots.realman.gripper import move, Buttons, Jog


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


def test_buttons_release_edges_arm_interlock_and_stale():
    b = Buttons()
    def update(a=False, trigger=0., fresh=True, grip=0., now=0.):
        return b.update(a, trigger, fresh=fresh, grip=grip, now=now)
    assert update(trigger=1.) is None
    assert update() is None
    assert update(trigger=1., now=1.) == 'close'
    assert update(trigger=1., now=3.) == 'close'
    update(now=4.)
    assert update(a=True, grip=1., now=5.) is None
    assert update(a=True, now=6.) is None
    update(now=7.)
    assert update(a=True, now=8.) == 'open'
    update(fresh=False, now=9.)
    assert update(a=True, now=10.) is None


def test_busy_is_not_a_terminal_failure():
    c = Client(); c.mode = 5
    assert move(c, 'close')['accepted'] is False
    assert c.sent == []


def test_jog_release_and_reverse_wait_for_pending_segment():
    c = Client(); c.pos = 500; c.mode = 3
    jog = Jog()
    assert jog.update(c, 'open', 0.)['position_target_raw'] == 510
    c.mode = 5; c.pos = 504
    assert jog.update(c, None, .2)['accepted'] is False
    assert len(c.sent) == 1  # Releasing never queues more motion.
    assert jog.update(c, 'close', .4)['accepted'] is False
    c.mode = 3
    assert jog.update(c, 'close', .6)['reason'] == 'awaiting position feedback'
    c.pos = 510
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
