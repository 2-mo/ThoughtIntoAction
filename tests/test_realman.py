"""Protocol and motion guards use fake devices only; no robot motion in tests."""
import copy
import json
import socket
import threading
import time

import numpy as np
import pytest

from embodied.contracts.control import ControlCommand, Mode
from embodied.contracts.registry import Registry
from embodied.devices.robots.realman.backend import Backend, load_config, decode_pose, encode_pose
from embodied.devices.robots.realman.teleop import RelativeTarget
from embodied.devices.robots.realman.protocol import Client


class FakeClient:
    def __init__(self, config):
        self.sent = []
        self.closed = False
        self.responses = {
            'get_DH_data': {},
            'get_joint_min_pos': {'min_pos': [-178000]*6},
            'get_joint_max_pos': {'max_pos': [178000]*6},
            'get_arm_software_info': {'Product_version': 'RM65-BI'},
            'get_current_tool_frame': {'tool_name': config['expected_tool'], 'pose': config['expected_tool_pose']},
            'get_current_work_frame': {'frame_name': config['expected_work_frame'], 'pose': config['expected_work_pose']},
            'get_current_arm_state': {'arm_state': {'joint': [1000, 0, 0, 0, 0, 0],
                                                   'pose': [300000, 0, 400000, 0, 0, 0], 'err': [0]}},
        }

    def query(self, command):
        return copy.deepcopy(self.responses[command])

    def send(self, message):
        self.sent.append(message)

    def close(self):
        self.closed = True


@pytest.fixture
def setup(monkeypatch):
    class FakeCheck:
        def __init__(self, info, state):
            self.fit = {}
        def check(self, target, state, **kwargs):
            return {'singularity_code': 0, 'ik_joint_deg': [0.]*6}
    monkeypatch.setattr('embodied.devices.robots.realman.backend.MotionCheck', FakeCheck)
    config = load_config()
    config['host'] = 'test-realman'
    return config, FakeClient(config)


def command(pose, **kwargs):
    return ControlCommand(Mode.COMMAND_FOLLOWING, pose_target=pose, engaged=True, **kwargs)


def test_units_and_quaternion_roundtrip():
    raw = [-313200, 1229, 398102, 3147, 581, -8]
    pose = decode_pose(raw)
    assert np.allclose(pose[:3], [-.3132, .001229, .398102])
    assert np.isclose(np.linalg.norm(pose[3:]), 1)
    # Euler representations may differ by 2pi; compare physical rotations.
    from embodied.devices.robots.realman.geometry import matrix
    assert np.allclose(matrix(decode_pose(encode_pose(pose))[3:]), matrix(pose[3:]), atol=.001)


def test_read_only_never_sends_and_unknown_feedback_is_nan(setup):
    config, client = setup
    backend = Backend(config, client=client)
    try:
        s = backend.read_state(time.monotonic())
        assert s.q[0] == pytest.approx(np.pi/180)
        assert np.isnan(s.dq).all() and np.isnan(s.gripper_width)
        with pytest.raises(PermissionError):
            backend.step(command(s.pose), time.monotonic())
    finally:
        backend.close()
    assert client.sent == []


def test_identity_mismatch_closes_connection_and_releases_lease(setup):
    config, client = setup
    client.responses['get_current_tool_frame']['tool_name'] = 'other'
    with pytest.raises(ValueError):
        Backend(config, client=client)
    assert client.closed
    client.responses['get_current_tool_frame']['tool_name'] = config['expected_tool']
    Backend(config, client=client).close()


def test_speed_limit_deadman_and_no_enable(setup):
    config, client = setup
    backend = Backend(config, client=client, allow_motion=True)
    try:
        target = backend.state.pose.copy(); target[0] += .04
        backend.step(command(target), time.monotonic())
        assert client.sent[-1]['command'] == 'movep_canfd'
        assert client.sent[-1]['follow'] is False
        assert client.sent[-1]['pose'][0] == 302400
        assert target[0] == pytest.approx(.34)  # Caller-owned target is not mutated.
        backend.step(ControlCommand(Mode.DISABLED), time.monotonic())
        assert client.sent[-1] == {'command': 'set_arm_stop'}
    finally:
        backend.close()
    assert {r['command'] for r in client.sent} == {'movep_canfd', 'set_arm_stop'}


def test_watchdog_stops_when_loop_stalls(setup):
    config, client = setup
    config['stale_s'] = .05
    backend = Backend(config, client=client, allow_motion=True)
    try:
        backend.step(command(backend.state.pose), time.monotonic())
        deadline = time.monotonic() + .6
        while not backend.fault and time.monotonic() < deadline:
            time.sleep(.01)
        assert 'timeout' in backend.fault
        assert client.sent[-1]['command'] == 'set_arm_stop'
        with pytest.raises(RuntimeError):
            backend.step(command(backend.state.pose), time.monotonic())
    finally:
        backend.close()


@pytest.mark.parametrize('kind', ['stale', 'error', 'radius', 'gripper', 'free-drive'])
def test_unsafe_targets_rejected(setup, kind):
    config, client = setup
    backend = Backend(config, client=client, allow_motion=True)
    try:
        target = backend.state.pose.copy()
        c = command(target)
        if kind == 'stale': backend.feedback_at -= 1
        if kind == 'error': backend.state.fault = 'fault'
        if kind == 'radius': target[0] += .2
        if kind == 'gripper': c.gripper_target = .01
        if kind == 'free-drive': c.mode = Mode.FREE_DRIVE
        with pytest.raises((RuntimeError, ValueError)):
            backend.step(c, time.monotonic())
        assert not any(m['command'] == 'movep_canfd' for m in client.sent)
    finally:
        backend.close()


def test_relative_reference_and_basis(setup):
    config, _ = setup
    tracker = RelativeTarget(config)
    robot = np.array([.3, 0, .4, 0, 0, 0, 1.])
    controller = np.array([0., 0, 0, 0, 0, 0, 1])
    assert np.allclose(tracker.update(controller, robot), robot)
    controller[2] = -.1
    assert np.allclose(tracker.update(controller, robot)[:3], [.24, 0, .4])
    tracker.reset()
    assert np.allclose(tracker.update(controller, robot), robot)
    controller[0] += .01
    assert np.allclose(tracker.update(controller, robot)[:3], [.3, .006, .4])
    controller[1] += .01
    assert np.allclose(tracker.update(controller, robot)[:3], [.3, .006, .406])


def test_protocol_fragmentation_and_readonly_whitelist():
    listener = socket.socket(); listener.bind(('127.0.0.1', 0)); listener.listen()
    def server():
        with listener.accept()[0] as conn:
            request = b''
            while not request.endswith(b'\r\n'): request += conn.recv(4096)
            assert json.loads(request)['command'] == 'get_current_arm_state'
            conn.sendall(b'{"command":"movep_canfd","state":true}\r\n{"sta')
            conn.sendall(b'te":"current_arm_state","arm_state":{"err":[0]}}\r\n')
    thread = threading.Thread(target=server); thread.start()
    client = Client('127.0.0.1', listener.getsockname()[1])
    try:
        with pytest.raises(PermissionError): client.send({'command':'set_arm_stop'})
        assert client.query('get_current_arm_state')['arm_state']['err'] == [0]
    finally:
        client.close(); listener.close(); thread.join(1)


def test_registry_loads_without_opening_device():
    assert Registry().load('arm', 'realman') is Backend
    with pytest.raises(ValueError): Registry().load('arm', 'realman', allow_hardware=False)


def test_xr_clock_freeze_rollback_and_recovery():
    from embodied.devices.robots.realman.teleop import XRClock
    clock = XRClock(.3)
    assert not clock.fresh(0, 0)
    assert clock.fresh(100, 1.)
    assert clock.fresh(100, 1.1)  # Stationary pose is valid while XR time progresses.
    assert not clock.fresh(100, 1.31)
    assert not clock.fresh(90, 1.32)
    assert clock.fresh(110, 1.4)


def test_feedback_error_stops_active_session(setup):
    config, client = setup
    backend = Backend(config, client=client, allow_motion=True)
    try:
        backend.step(command(backend.state.pose), time.monotonic())
        client.responses['get_current_arm_state']['arm_state']['err'] = [10]
        state = backend.read_state(time.monotonic())
        with pytest.raises(RuntimeError):
            backend.step(command(state.pose), time.monotonic())
        assert client.sent[-1]['command'] == 'set_arm_stop'
    finally:
        backend.close()


@pytest.mark.parametrize('duration', ['.15', '0'])
@pytest.mark.parametrize('execute', [False, True])
@pytest.mark.parametrize('position_only', [False, True])
def test_cli_preview_and_explicit_execute_use_same_mapping(setup, monkeypatch, capsys, execute, duration, position_only):
    import sys
    from types import SimpleNamespace
    from embodied.devices.robots.realman import cli
    config, client = setup
    ticks = [0]
    def grip():
        ticks[0] += 1
        return 0. if ticks[0] == 1 else 1.
    xr = SimpleNamespace(init=lambda: None, close=lambda: None,
        get_time_stamp_ns=time.monotonic_ns, get_right_grip=grip,
        get_right_controller_pose=lambda: [0., 0., -.001*ticks[0], np.sin(.01*ticks[0]), 0., 0., np.cos(.01*ticks[0])],
        get_B_button=lambda: duration == '0' and ticks[0] > 3)
    monkeypatch.setitem(sys.modules, 'xrobotoolkit_sdk', xr)
    monkeypatch.setattr(cli, 'load_config', lambda _: config)
    monkeypatch.setattr(cli, 'Backend', lambda config, allow_motion: Backend(config, client=client, allow_motion=allow_motion))
    monkeypatch.setattr(sys, 'argv', ['realman', 'quest', '--duration', duration] +
                        (['--execute'] if execute else []) + (['--position-only'] if position_only else []))
    cli.main()
    output = capsys.readouterr().out
    assert 'target_m_xyzw' in output
    assert ('"mode": "position_only"' if position_only else '"mode": "position_and_rotation"') in output
    if execute:
        assert client.sent[0]['command'] == 'movep_canfd'
        assert client.sent[-1]['command'] == 'set_arm_stop'
        orientations = [m['pose'][3:] for m in client.sent if m['command'] == 'movep_canfd']
        assert any(any(angle != 0 for angle in angles) for angles in orientations) is not position_only
    else:
        assert client.sent == []
    assert client.closed


def test_partial_motion_send_failure_still_attempts_stop(setup):
    config, client = setup
    original_send = client.send
    def send(message):
        if message['command'] == 'movep_canfd':
            raise ConnectionError('partial write')
        original_send(message)
    client.send = send
    backend = Backend(config, client=client, allow_motion=True)
    try:
        with pytest.raises(ConnectionError, match='partial write'):
            backend.step(command(backend.state.pose), time.monotonic())
        assert client.sent == [{'command': 'set_arm_stop'}]
        assert backend.fault
    finally:
        backend.close()


def test_grip_hysteresis_and_rearm_after_tracking_loss():
    from embodied.devices.robots.realman.teleop import GripGate
    gate = GripGate()
    assert not gate.update(1., True)  # Held at startup cannot engage.
    assert not gate.update(0., True)
    assert gate.update(1., True)
    assert gate.update(.5, True)  # Pressure fluctuation must not reset reference.
    assert not gate.update(.1, True)
    assert not gate.update(.5, True)
    assert gate.update(.9, True)
    assert not gate.update(.9, False)
    assert not gate.update(1., True)  # Must release after losing tracking.
    assert not gate.update(0., True)
    assert gate.update(1., True)


def test_reclutch_cannot_expand_session_workspace_and_can_return(setup):
    from embodied.devices.robots.realman.geometry import rotation_vector
    from meshcat import transformations as tf
    config, _ = setup
    tracker = RelativeTarget(config)
    start = np.array([.3, 0., .4, 0., 0., 0., 1.])
    tracker.origin = start.copy()
    current = start.copy()
    controller = np.array([0., 0., 0., 0., 0., 0., 1.])
    for _ in range(4):
        tracker.reset()
        tracker.update(controller, current)
        moved = controller.copy()
        moved[2] = -1.
        moved[3:] = np.roll(tf.quaternion_from_euler(0., .5, 0.), -1)
        current = tracker.update(moved, current)
        assert np.linalg.norm(current[:3] - start[:3]) <= config['translation_radius_m'] + 1e-10
        assert np.linalg.norm(rotation_vector(current[3:], start[3:])) <= config['rotation_radius_rad'] + 1e-10
        assert 'translation_boundary' in tracker.limits
    # A fresh grip at the boundary can move back into the permitted workspace.
    tracker.reset()
    tracker.update(controller, current)
    moved = controller.copy(); moved[2] = .02
    returned = tracker.update(moved, current)
    assert np.linalg.norm(returned[:3] - start[:3]) < np.linalg.norm(current[:3] - start[:3])


def test_cli_error_is_readable_without_traceback(monkeypatch, capsys):
    from embodied.devices.robots.realman import cli
    def fail():
        raise RuntimeError('XR stalled')
    monkeypatch.setattr(cli, 'run', fail)
    with pytest.raises(SystemExit) as err:
        cli.main()
    assert err.value.code == 1
    assert 'XR stalled' in capsys.readouterr().err


def test_transient_following_lag_holds_then_resumes(setup):
    config, client = setup
    backend = Backend(config, client=client, allow_motion=True)
    try:
        previous = backend.state.pose.copy(); previous[0] += .0099
        backend.last_target = previous.copy()
        goal = previous.copy(); goal[0] += .01
        backend.last_command = time.monotonic() - 1/config['control_hz']
        backend.step(command(goal), time.monotonic())
        assert backend.command_diagnostics['lead_limited']
        assert np.allclose(backend.last_target, previous)
        backend.state.pose[0] += .005  # Measured robot catches up.
        backend.last_command -= 1/config['control_hz']
        backend.step(command(goal), time.monotonic())
        assert not backend.command_diagnostics['lead_limited']
        assert backend.last_target[0] > previous[0]
        assert backend.lead_hold_since is None
    finally:
        backend.close()


def test_persistent_following_lag_stops_without_widening_limits(setup):
    config, client = setup
    backend = Backend(config, client=client, allow_motion=True)
    try:
        previous = backend.state.pose.copy(); previous[0] += .0099
        backend.last_target = previous.copy()
        backend.last_command = time.monotonic() - .04
        backend.active = True
        backend.lead_hold_since = time.monotonic() - 1.1
        goal = previous.copy(); goal[0] += .01
        with pytest.raises(RuntimeError, match='lag persisted'):
            backend.step(command(goal), time.monotonic())
        assert client.sent == [{'command': 'set_arm_stop'}]
        assert backend.command_diagnostics['candidate_lead_m'] > .01
    finally:
        backend.close()


def test_offline_rejection_stops_before_transmitting_candidate(setup):
    config, client = setup
    backend = Backend(config, client=client, allow_motion=True)
    try:
        backend.step(command(backend.state.pose), time.monotonic())
        client.sent.clear()
        def reject(target, state, **kwargs):
            raise RuntimeError('Offline IK rejected candidate')
        backend.motion_check.check = reject
        with pytest.raises(RuntimeError, match='Offline IK rejected'):
            backend.step(command(backend.state.pose), time.monotonic())
        assert client.sent == [{'command': 'set_arm_stop'}]
    finally:
        backend.close()


def test_joint_command_baseline_commits_only_after_send_and_resets_on_stop(setup):
    config, client = setup
    backend = Backend(config, client=client, allow_motion=True)
    seen = []
    def check(target, state, *, previous_joint_deg):
        seen.append(previous_joint_deg)
        return {'ik_joint_deg': [len(seen)*.2]*6}
    backend.motion_check.check = check
    try:
        backend.step(command(backend.state.pose), time.monotonic())
        assert seen[0] is None
        backend.step(command(backend.state.pose), time.monotonic())
        assert np.allclose(seen[1], .2)
        backend.stop()
        assert backend.last_joint_target is None
        def fail_send(message):
            if message['command'] == 'movep_canfd':
                raise OSError('simulated send failure')
        client.send = fail_send
        with pytest.raises(OSError, match='send failure'):
            backend.step(command(backend.state.pose), time.monotonic())
        assert seen[2] is None
        assert backend.last_joint_target is None
    finally:
        backend.close()


def test_gripper_status_query_is_readonly_and_matches_command_with_numeric_status():
    listener = socket.socket(); listener.bind(('127.0.0.1', 0)); listener.listen()
    def server():
        with listener.accept()[0] as conn:
            request = b''
            while not request.endswith(b'\r\n'):
                request += conn.recv(4096)
            assert json.loads(request) == {'command': 'get_gripper_state'}
            conn.sendall(b'{"command":"get_gripper_state","state":false,"enable":0,"status":0,"error":0,"mode":0,"current_force":0,"temperature":0,"actpos":0}\r\n')
        listener.close()
    thread = threading.Thread(target=server); thread.start()
    client = Client('127.0.0.1', listener.getsockname()[1])
    try:
        assert client.query('get_gripper_state')['status'] == 0
        with pytest.raises(PermissionError):
            client.send({'command': 'set_gripper_release', 'speed': 100})
    finally:
        client.close(); thread.join(timeout=1)


def test_gripper_exchange_requires_separate_permission_and_matches_ack():
    listener = socket.socket(); listener.bind(('127.0.0.1', 0)); listener.listen()
    def server():
        with listener.accept()[0] as conn:
            request = b''
            while not request.endswith(b'\r\n'):
                request += conn.recv(4096)
            assert json.loads(request)['command'] == 'set_gripper_release'
            conn.sendall(b'{"command":"set_gripper","state":true}\r\n')
        listener.close()
    thread = threading.Thread(target=server); thread.start()
    client = Client('127.0.0.1', listener.getsockname()[1], allow_gripper=True)
    try:
        with pytest.raises(PermissionError):
            client.send({'command': 'movep_canfd'})
        reply = client.exchange(dict(command='set_gripper_release', speed=100, block=False), 'set_gripper')
        assert reply['state'] is True
    finally:
        client.close(); thread.join(timeout=1)


def test_joint_step_backoff_checks_wire_pose_before_send(setup):
    from embodied.devices.robots.realman.kinematics import JointStepTooLarge
    config, client = setup
    backend = Backend(config, client=client, allow_motion=True)
    try:
        calls = []
        origin = backend.state.pose.copy()
        def check(target, state, **kwargs):
            calls.append(target.copy())
            step = abs(target[0] - origin[0])*2000
            backend.motion_check.diagnostics = {'max_joint_change_deg': step}
            if step > 1:
                raise JointStepTooLarge('too large')
            return {'ik_joint_deg': [step]*6}
        backend.motion_check.check = check
        goal = origin.copy(); goal[0] += .01
        backend.step(command(goal), time.monotonic())
        assert len(calls) == 4
        assert backend.command_diagnostics['joint_step_scale'] == .125
        assert len(backend.command_diagnostics['joint_step_rejections']) == 3
        assert np.allclose(decode_pose(client.sent[-1]['pose']), calls[-1])
        assert abs(calls[-1][0] - origin[0]) <= .0005
    finally:
        backend.close()


def test_persistent_joint_step_rejection_still_stops_without_send(setup):
    from embodied.devices.robots.realman.kinematics import JointStepTooLarge
    config, client = setup
    backend = Backend(config, client=client, allow_motion=True)
    try:
        def reject(*args, **kwargs):
            raise JointStepTooLarge('persistent joint discontinuity')
        backend.motion_check.check = reject
        with pytest.raises(JointStepTooLarge):
            backend.step(command(backend.state.pose), time.monotonic())
        assert len(backend.command_diagnostics['joint_step_rejections']) == 5
        assert not client.sent
    finally:
        backend.close()


@pytest.mark.parametrize('offline_query', [1, 2, 3])
def test_gripper_offline_disables_motion_until_neutral_recovery(setup, monkeypatch, capsys, offline_query):
    import sys
    from types import SimpleNamespace
    from embodied.devices.robots.realman import cli
    config, client = setup
    original_query = client.query
    gripper_queries = []
    def query(key):
        if key != 'get_gripper_state':
            return original_query(key)
        gripper_queries.append(key)
        # Recovery reads may observe ready, but two neutral samples are required.
        if len(gripper_queries) == offline_query:
            return dict(enable=0, status=0, error=0, mode=0,
                        current_force=0, temperature=0, actpos=0)
        return dict(enable=1, status=1, error=0, mode=3,
                    current_force=0, temperature=34, actpos=500)
    client.query = query
    client.exchange = lambda message, expected: (client.sent.append(message) or {'state': True})
    ticks = [0]
    def grip():
        ticks[0] += 1
        return 1. if ticks[0] >= 7 else 0.
    xr = SimpleNamespace(init=lambda: None, close=lambda: None,
        get_time_stamp_ns=time.monotonic_ns, get_right_grip=grip,
        get_right_controller_pose=lambda: [0., 0., -.001*ticks[0], 0., 0., 0., 1.],
        get_A_button=lambda: ticks[0] in (2, 3), get_right_trigger=lambda: 0.,
        get_B_button=lambda: ticks[0] >= 9)
    monkeypatch.setitem(sys.modules, 'xrobotoolkit_sdk', xr)
    monkeypatch.setattr(cli, 'load_config', lambda _: config)
    monkeypatch.setattr(cli, 'Backend', lambda config, allow_motion: Backend(config, client=client, allow_motion=allow_motion))
    monkeypatch.setattr(sys, 'argv', ['realman', 'quest', '--execute', '--gripper', '--duration', '0'])
    cli.main()
    output = capsys.readouterr().out
    assert '"enabled_for_session": false' in output
    assert '"gripper_enabled_for_session": false' in output
    assert len(gripper_queries) == offline_query + int(offline_query == 1)
    assert sum(m['command'] == 'set_gripper_position' for m in client.sent) == int(offline_query == 3)
    assert any(m['command'] == 'movep_canfd' for m in client.sent)
    assert client.sent[-1] == {'command': 'set_arm_stop'}
    assert client.closed


def test_explicit_open_initialization_entry_from_all_zero_feedback(setup, monkeypatch, capsys):
    import sys
    from embodied.devices.robots.realman import cli
    config, client = setup
    client.responses['get_gripper_state'] = dict.fromkeys(
        ('enable', 'status', 'error', 'mode', 'current_force', 'temperature', 'actpos'), 0)
    client.exchange = lambda message, expected: (client.sent.append(message) or {'state': True})
    monkeypatch.setattr(cli, 'load_config', lambda _: config)
    monkeypatch.setattr(cli, 'Backend', lambda config, allow_motion: Backend(config, client=client, allow_motion=allow_motion))
    monkeypatch.setattr(sys, 'argv', ['realman', 'gripper-open', '--execute', '--gripper-initialize-open'])
    cli.main()
    assert client.sent == [{'command':'set_gripper_release','speed':100,'block':False}]
    assert '"initializing_from_zero_feedback": true' in capsys.readouterr().out
    assert client.closed


@pytest.mark.parametrize('step', [None, 100])
def test_cli_gripper_uses_default_and_explicit_segment_size(setup, monkeypatch, capsys, step):
    import sys
    from types import SimpleNamespace
    from embodied.devices.robots.realman import cli
    config, client = setup
    client.responses['get_gripper_state'] = dict(enable=1,status=1,error=0,mode=3,
        current_force=0,temperature=33,actpos=500)
    client.exchange = lambda message, expected: (client.sent.append(message) or {'state':True})
    ticks = [0]
    def grip():
        ticks[0] += 1
        return 0.
    xr = SimpleNamespace(init=lambda: None,close=lambda: None,get_time_stamp_ns=time.monotonic_ns,
        get_right_grip=grip,get_A_button=lambda: False,
        get_right_controller_pose=lambda: [0.,0.,0.,0.,0.,0.,1.],
        get_right_trigger=lambda: 1. if ticks[0] in (2,3) else 0.,get_B_button=lambda: ticks[0]>=4)
    monkeypatch.setitem(sys.modules, 'xrobotoolkit_sdk', xr)
    monkeypatch.setattr(cli, 'load_config', lambda _: config)
    monkeypatch.setattr(cli, 'Backend', lambda config, allow_motion: Backend(config,client=client,allow_motion=allow_motion))
    monkeypatch.setattr(sys, 'argv', ['realman','quest','--execute','--gripper','--duration','0'] +
        ([] if step is None else ['--gripper-step',str(step)]))
    cli.main()
    effective = 50 if step is None else step
    assert client.sent == [{'command':'set_gripper_position','position':500-effective,'block':False}]
    assert f'{effective}/1000 position segments' in capsys.readouterr().out
    assert client.closed
