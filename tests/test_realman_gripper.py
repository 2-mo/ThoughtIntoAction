import pytest
from embodied.devices.robots.realman.gripper import move, Buttons


class Client:
    def __init__(self):
        self.sent = []
        self.ready = True
    def query(self, command):
        return dict(enable=1, status=int(self.ready), error=0, mode=1,
                    current_force=0, temperature=34, actpos=1002)
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
    assert update(trigger=1., now=3.) is None
    update(now=4.)
    assert update(a=True, grip=1., now=5.) is None
    assert update(a=True, now=6.) is None
    update(now=7.)
    assert update(a=True, now=8.) == 'open'
    update(fresh=False, now=9.)
    assert update(a=True, now=10.) is None
