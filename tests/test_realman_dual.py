"""Dual routing and failure containment with deterministic fake XR/robots."""
import sys
from types import SimpleNamespace

import numpy as np
import pytest

from embodied.devices.robots.realman import cli, quest_control
from embodied.devices.robots.realman.backend import load_config


POSE = [0., 0., 0., 0., 0., 0., 1.]


class Session:
    def __init__(self, monkeypatch, frames, *, execute=True, gripper=False):
        self.frames, self.index, self.events = frames, 0, []
        self.execute, self.gripper = execute, gripper
        self.fault = self.read_error = self.send_error = self.stop_error = None
        self.gripper_error = self.gripper_offline = None
        self.stall_at = None
        self.backends = {}
        self.xr = SimpleNamespace(init=lambda: self.event('xr', 'init'),
            close=lambda: self.event('xr', 'close'),
            get_time_stamp_ns=lambda: (1 + min(self.index, self.stall_at)
                if self.stall_at is not None else 1 + self.index) * 40_000_000,
            get_B_button=lambda: self.index >= len(frames) or self.value('B', False),
            get_Y_button=lambda: self.value('Y', False))
        for hand, button in [('left', 'X'), ('right', 'A')]:
            setattr(self.xr, f'get_{hand}_grip', lambda h=hand: self.value(h+'_grip', 0.))
            setattr(self.xr, f'get_{hand}_trigger', lambda h=hand: self.value(h+'_trigger', 0.))
            setattr(self.xr, f'get_{hand}_controller_pose', lambda h=hand: self.value(h+'_pose', POSE))
            setattr(self.xr, f'get_{button}_button', lambda b=button: self.value(b, False))
        monkeypatch.setattr(quest_control, 'time', SimpleNamespace(
            monotonic=lambda: 1. + self.index * .04, sleep=self.advance))
        monkeypatch.setattr(cli, 'Backend', self.backend)
        monkeypatch.setitem(sys.modules, 'xrobotoolkit_sdk', self.xr)

    def value(self, key, default):
        return self.frames[self.index].get(key, default) if self.index < len(self.frames) else default

    def event(self, host, kind, value=None):
        self.events.append((self.index, host, kind, value))

    def advance(self, seconds):
        assert seconds >= 0
        self.index += 1

    def backend(self, config, allow_motion):
        session = self
        host = config['host']

        class FakeBackend:
            def __init__(self):
                # Fake startup supplies the same effective limits as the onsite controller.
                self.config = {**config, 'linear_speed_m_s': .125, 'angular_speed_rad_s': .30}
                self.allow_motion = allow_motion
                self.origin = np.array([-.3, 0., .3, 0., 0., 0., 1.])
                self.state = SimpleNamespace(pose=self.origin.copy(), fault='')
                self.info = {'get_current_tool_frame': {}, 'get_current_work_frame': {}}
                self.active, self.closed = False, False
                self.opening = 500
                self.client = SimpleNamespace(query=self.query, exchange=self.exchange, allow_gripper=False)

            def query(self, key):
                if key == 'get_gripper_state':
                    if session.gripper_error == (host, session.index):
                        raise TimeoutError('gripper feedback timeout')
                    ready = int(session.gripper_offline != (host, session.index))
                    return dict(enable=ready, status=ready, error=0, mode=3,
                        temperature=33, current_force=0, actpos=self.opening)
                return self.info[key]

            def exchange(self, message, expected):
                assert self.client.allow_gripper
                session.event(host, 'gripper', message.copy())
                self.opening = message.get('position', self.opening)
                return {'state': True}

            def read_state(self, now):
                if session.read_error == (host, session.index):
                    raise OSError('feedback timeout')
                self.state.fault = 'robot fault' if session.fault == (host, session.index) else ''
                return self.state

            def step(self, command, now):
                assert self.allow_motion
                if session.send_error == (host, session.index):
                    raise RuntimeError('IK/send rejected')
                self.active = True
                session.event(host, 'target', command.pose_target.copy())

            def telemetry(self):
                return {'command': {}}

            def stop(self):
                session.event(host, 'stop_attempt')
                if self.active:
                    self.active = False
                    session.event(host, 'stop')
                    if session.stop_error == (host, session.index):
                        raise OSError('stop unavailable')

            def close(self):
                self.stop()
                self.closed = True
                session.event(host, 'close')

        result = self.backends[host] = FakeBackend()
        return result

    def run(self, monkeypatch, *extra):
        args = ['realman', 'quest-dual', '--position-only', '--duration', '0']
        if self.execute:
            args += ['--execute']
        if self.gripper:
            args += ['--gripper']
        monkeypatch.setattr(sys, 'argv', args + list(extra))
        cli.run()

    def targets(self, host):
        return [e for e in self.events if e[1:3] == (host, 'target')]


LEFT, RIGHT = '192.168.1.19', '192.168.1.18'
BOTH = dict(left_grip=1., right_grip=1.)


def test_default_hosts_independent_pose_clutch_and_reanchor(monkeypatch):
    session = Session(monkeypatch, [{}, BOTH,
        dict(BOTH, left_pose=[.02, 0., 0., 0., 0., 0., 1.],
            right_pose=[0., 0., -.03, 0., 0., 0., 1.]),
        dict(right_grip=1., right_pose=[0., 0., -.04, 0., 0., 0., 1.]),
        dict(BOTH, left_pose=[.2, 0., 0., 0., 0., 0., 1.])])
    session.run(monkeypatch)
    left, right = session.targets(LEFT), session.targets(RIGHT)
    assert [e[0] for e in left] == [1, 2, 4]
    assert [e[0] for e in right] == [1, 2, 3, 4]
    np.testing.assert_allclose(left[1][3][:3], [-.3, .02, .3])
    np.testing.assert_allclose(right[1][3][:3], [-.33, 0., .3])
    np.testing.assert_allclose(left[-1][3], session.backends[LEFT].state.pose)
    assert any(e[:3] == (3, LEFT, 'stop') for e in session.events)
    assert sum(e[1:3] == ('xr', 'init') for e in session.events) == 1
    assert sum(e[1:3] == ('xr', 'close') for e in session.events) == 1
    assert all(b.closed for b in session.backends.values())


def test_readonly_preview_never_moves_either_arm_or_gripper(monkeypatch, capsys):
    session = Session(monkeypatch, [{}, BOTH, dict(BOTH, X=True, A=True)], execute=False)
    session.run(monkeypatch)
    assert not any(e[2] in ('target', 'gripper', 'stop') for e in session.events)
    assert '"hand": "left", "host": "192.168.1.19", "preview": true' in capsys.readouterr().out


@pytest.mark.parametrize('moving_hand', ['left', 'right'])
def test_both_grippers_work_while_either_arm_is_following(monkeypatch, moving_hand):
    closing_hand = 'right' if moving_hand == 'left' else 'left'
    moving_host = LEFT if moving_hand == 'left' else RIGHT
    held = {closing_hand+'_trigger': 1., moving_hand+'_grip': 1.,
            moving_hand+'_trigger': 1.}
    session = Session(monkeypatch, [{}, {'X': True, 'A': True}, {}, {}, {}, {}] +
        [held]*3, gripper=True)
    session.run(monkeypatch)
    moves = [e for e in session.events if e[2] == 'gripper']
    assert [(e[0], e[1], e[3]['position']) for e in moves] == [
        (1, LEFT, 600), (1, RIGHT, 600), (6, LEFT, 500), (6, RIGHT, 500),
        (8, LEFT, 400), (8, RIGHT, 400)]
    assert session.targets(moving_host)
    other_host = RIGHT if moving_host == LEFT else LEFT
    assert not session.targets(other_host)


def test_both_arms_follow_while_both_grippers_open_then_close(monkeypatch):
    session = Session(monkeypatch, [{}, dict(BOTH, X=True, A=True), BOTH,
        dict(BOTH, left_trigger=1., right_trigger=1.)], gripper=True)
    session.run(monkeypatch)
    moves = [e for e in session.events if e[2] == 'gripper']
    assert [(e[0], e[1], e[3]['position']) for e in moves] == [
        (1, LEFT, 600), (1, RIGHT, 600), (3, LEFT, 500), (3, RIGHT, 500)]
    for host in (LEFT, RIGHT):
        assert [e[0] for e in session.targets(host)] == [1, 2, 3]


def test_gripper_recovers_with_buttons_neutral_while_arm_keeps_following(monkeypatch):
    session = Session(monkeypatch, [{}] + [BOTH]*51 +
        [dict(BOTH, left_trigger=1.)], gripper=True)
    session.gripper_offline = (LEFT, 0)
    session.run(monkeypatch)
    assert len(session.targets(LEFT)) == 52
    assert [(e[0], e[1], e[3]['position']) for e in session.events if e[2] == 'gripper'] == [
        (52, LEFT, 400)]


def test_gripper_timeout_during_following_stops_both_arms(monkeypatch):
    session = Session(monkeypatch, [{}, BOTH, dict(BOTH, left_trigger=1.)], gripper=True)
    session.gripper_error = (LEFT, 2)
    with pytest.raises(TimeoutError, match='gripper feedback timeout'):
        session.run(monkeypatch)
    assert {e[1] for e in session.events if e[2] == 'stop'} == {LEFT, RIGHT}


def test_explicit_dual_startup_open_applies_selected_speed_to_both_grippers(monkeypatch):
    session = Session(monkeypatch, [{}], gripper=True)
    session.run(monkeypatch, '--gripper-initialize-open', '--gripper-speed', '300')
    moves = [e for e in session.events if e[2] == 'gripper']
    assert {e[1] for e in moves} == {LEFT, RIGHT}
    assert len(moves) == 2
    assert all(e[3] == dict(command='set_gripper_release', speed=300, block=False) for e in moves)


def test_dual_startup_open_defaults_to_half_gripper_speed(monkeypatch):
    session = Session(monkeypatch, [{}], gripper=True)
    session.run(monkeypatch, '--gripper-initialize-open')
    moves = [e for e in session.events if e[2] == 'gripper']
    assert {e[1] for e in moves} == {LEFT, RIGHT}
    assert len(moves) == 2
    assert all(e[3] == dict(command='set_gripper_release', speed=500, block=False) for e in moves)


@pytest.mark.parametrize('extra', [
    ['--gripper-speed', '300'],
    ['--gripper-initialize-open', '--gripper-speed', '0'],
    ['--gripper-initialize-open', '--gripper-speed', '1001'],
])
def test_speed_option_rejects_silent_ineffectiveness_and_invalid_range(monkeypatch, extra):
    session = Session(monkeypatch, [], gripper=True)
    with pytest.raises(SystemExit):
        session.run(monkeypatch, *extra)
    assert session.backends == {}


@pytest.mark.parametrize('host', [LEFT, RIGHT])
@pytest.mark.parametrize('failure', ['fault', 'read_error', 'send_error'])
def test_either_robot_failure_stops_both_before_xr_close(monkeypatch, host, failure):
    session = Session(monkeypatch, [{}, BOTH, BOTH, BOTH])
    setattr(session, failure, (host, 3))
    with pytest.raises((RuntimeError, OSError)):
        session.run(monkeypatch)
    for robot in (LEFT, RIGHT):
        assert any(e[:3] == (3, robot, 'stop') for e in session.events)
    close = next(i for i, e in enumerate(session.events) if e[1:3] == ('xr', 'close'))
    assert all(i < close for i, e in enumerate(session.events) if e[2] == 'stop')
    if failure in ('fault', 'read_error'):
        assert not any(e[0] == 3 and e[2] == 'target' for e in session.events)


def test_invalid_right_pose_prevents_left_command_that_tick(monkeypatch):
    session = Session(monkeypatch, [{}, BOTH, BOTH, dict(BOTH, right_pose=[float('nan')]*7)])
    with pytest.raises(ValueError, match='finite pose'):
        session.run(monkeypatch)
    assert not any(e[0] == 3 and e[2] == 'target' for e in session.events)
    assert {e[1] for e in session.events if e[:1] == (3,) and e[2] == 'stop'} == {LEFT, RIGHT}


@pytest.mark.parametrize('button', ['B', 'Y'])
def test_either_exit_button_stops_both(monkeypatch, button):
    session = Session(monkeypatch, [{}, BOTH, dict(BOTH, **{button: True})])
    session.run(monkeypatch)
    assert {e[1] for e in session.events if e[0] == 2 and e[2] == 'stop'} == {LEFT, RIGHT}
    assert not any(e[0] == 2 and e[2] == 'target' for e in session.events)


def test_global_xr_stall_stops_both_without_resuming(monkeypatch):
    session = Session(monkeypatch, [{}] + [BOTH]*20)
    session.stall_at = 2
    with pytest.raises(RuntimeError, match='XR data stalled'):
        session.run(monkeypatch)
    assert {e[1] for e in session.events if e[2] == 'stop'} == {LEFT, RIGHT}
    assert all(e[0] < session.index for e in session.events if e[2] == 'target')


def test_failed_stop_does_not_skip_other_arm_or_hide_original_fault(monkeypatch, capsys):
    session = Session(monkeypatch, [{}, BOTH, BOTH, BOTH])
    session.fault, session.stop_error = (RIGHT, 3), (LEFT, 3)
    with pytest.raises(RuntimeError, match='robot fault') as error:
        session.run(monkeypatch)
    assert {e[1] for e in session.events if e[2] == 'stop'} == {LEFT, RIGHT}
    assert 'stop unavailable' in capsys.readouterr().err
    assert any('Stop delivery failed' in note for note in error.value.__notes__)


def test_second_connection_failure_closes_first_without_motion(monkeypatch):
    session = Session(monkeypatch, [])
    def connect(config, allow_motion):
        if config['host'] == LEFT:
            raise ConnectionRefusedError('left connection refused')
        return session.backend(config, allow_motion)
    monkeypatch.setattr(cli, 'Backend', connect)
    with pytest.raises(ConnectionRefusedError):
        session.run(monkeypatch)
    assert session.backends[RIGHT].closed
    assert not any(e[2] in ('target', 'gripper') for e in session.events)


@pytest.mark.parametrize('invalid', ['same_host', 'different_hz'])
def test_invalid_dual_configs_rejected_before_connection(monkeypatch, invalid):
    session = Session(monkeypatch, [])
    def config(path):
        result = load_config(path)
        if invalid == 'same_host':
            result['host'] = LEFT
        elif result['host'] == LEFT:
            result['control_hz'] = 20
        return result
    monkeypatch.setattr(cli, 'load_config', config)
    with pytest.raises(SystemExit):
        session.run(monkeypatch)
    assert session.backends == {}
