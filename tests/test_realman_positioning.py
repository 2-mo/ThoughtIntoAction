"""SDK joint positioning lifecycle using fake hardware only."""
from types import SimpleNamespace
import numpy as np
import pytest
from embodied.devices.robots.realman.positioning import JOINT_POSES, execute_joint_target


@pytest.mark.parametrize('failure', [None, 'feedback', 'move', 'read', 'stop'])
@pytest.mark.parametrize('target', JOINT_POSES.values())
def test_sdk_checks_feedback_and_stops_failed_motion(monkeypatch, failure, target):
    monkeypatch.setattr('embodied.devices.robots.realman.positioning.time.sleep', lambda _: None)
    monkeypatch.setattr('embodied.devices.robots.realman.positioning.joint_preview',
                        lambda backend, target_deg: {'target_joint_deg': target_deg})
    events = []
    q = np.array([10., 20., 30., 40., 50., 60.])
    class Arm:
        def rm_create_robot_arm(self, host, port):
            events.append('connect')
            return SimpleNamespace(id=1)
        def rm_get_current_arm_state(self):
            return 0, {'joint': (q + (1 if failure == 'feedback' else 0)).tolist()}
        def rm_movej(self, joint, speed, blend, connect, block):
            assert (joint, speed, blend, connect, block) == (target, 5, 0, 0, 0)
            events.append('move')
            return 1 if failure == 'move' else 0
        def rm_set_arm_stop(self):
            events.append('stop'); return 1 if failure == 'stop' else 0
        def rm_delete_robot_arm(self):
            events.append('disconnect')
    def read(now):
        if 'move' in events and failure in ('read', 'stop'):
            raise TimeoutError('feedback lost')
        return SimpleNamespace(q=np.deg2rad(target if 'move' in events else q), fault='')
    backend = SimpleNamespace(config={'host': 'fake', 'port': 8080}, read_state=read)
    sdk = SimpleNamespace(RoboticArm=lambda mode: Arm(), rm_thread_mode_e=SimpleNamespace(RM_DUAL_MODE_E=1))
    if failure:
        with pytest.raises((RuntimeError, TimeoutError)) as caught:
            execute_joint_target(backend, target, sdk=sdk)
        if failure == 'stop':
            assert isinstance(caught.value, TimeoutError)
            assert 'Stop delivery failed' in caught.value.__notes__[0]
        assert ('move' in events) == (failure != 'feedback')
        assert ('stop' in events) == (failure != 'feedback')
    else:
        result = execute_joint_target(backend, target, sdk=sdk)
        assert result['completed'] is True
        np.testing.assert_allclose(result['joint_deg'], target, atol=1e-8)
        assert 'stop' not in events
    assert events[-1] == 'disconnect'
