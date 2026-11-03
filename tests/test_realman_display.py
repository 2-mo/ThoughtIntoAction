"""One-way display and snapshot freshness; no robot or XR connection."""
import json
import os
from pathlib import Path
import time
from types import SimpleNamespace

import numpy as np

from embodied.applications.monitor.realman_view import RealManView, display_model
from embodied.applications.monitor.scene import Scene
from embodied.devices.robots.realman.telemetry import StateMirror, latest_state


def feedback():
    return {'schema': 1, 'stream_id': 'test-dual', 'sequence': 1, 'pid': os.getpid(),
        'boot_id': Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
        'sample_monotonic_ns': time.monotonic_ns(), 'reason': 'waiting_for_B',
        'xr_timestamp_ns': 42, 'xr_fresh': True, 'paused': True,
        'arms': [{'hand': h, 'host': host, 'q': [v, 0., 0., 0., 0., 0.],
            'target_q_deg': None, 'feedback_at': time.monotonic(), 'fault': ''}
            for h, host, v in [('left', '192.168.1.19', .2), ('right', '192.168.1.18', -.3)]]}


def test_both_measured_arms_render_and_disappear_when_feedback_is_lost(tmp_path):
    state = feedback()
    view = RealManView(reader=lambda: state, model_factory=lambda hands: display_model(hands, tmp_path))
    scene = Scene(None)
    try:
        assert view.step(scene)
        snapshot = scene.snapshot()
        assert scene.robot.model.nq == 12
        assert len(scene._manifest['meshes']) >= 14
        assert '示意' in scene._manifest['model_file']
        assert scene.robot.names == [f'{h}_joint{i}' for h in ('left', 'right') for i in range(1, 7)]
        np.testing.assert_allclose(snapshot['actual']['q'], [.2, 0, 0, 0, 0, 0, -.3, 0, 0, 0, 0, 0])
        assert snapshot['gripper'] is None  # Uncalibrated raw position never becomes millimetres.
        assert view.monitor()['input']['control_enabled'] is False
        state = None
        assert not view.step(scene)
        assert scene.robot is None
    finally:
        scene.close()


def test_snapshot_reader_rejects_stale_or_other_boot_feedback(tmp_path):
    path = tmp_path / 'state.json'
    state = feedback()
    path.write_text(json.dumps(state))
    assert latest_state(path)['stream_id'] == 'test-dual'
    state['sample_monotonic_ns'] -= 3_000_000_000
    path.write_text(json.dumps(state))
    assert latest_state(path) is None
    state.update(sample_monotonic_ns=time.monotonic_ns(), boot_id='other-boot')
    path.write_text(json.dumps(state))
    assert latest_state(path) is None


def test_atomic_publisher_handles_numpy_targets_and_closes(tmp_path):
    b = SimpleNamespace(state=SimpleNamespace(q=np.zeros(6), fault=''),
        last_joint_target=np.ones(6), feedback_at=time.monotonic(), command_diagnostics={})
    c = SimpleNamespace(backend=b, hand='left', config={'host': 'test',
        'linear_speed_m_s': .25, 'angular_speed_rad_s': .3},
        gate=SimpleNamespace(following=False), grip=0.)
    path = tmp_path / 'state.json'
    mirror = StateMirror(path)
    try:
        mirror.update([c], fresh=True, timestamp=42, paused=True, reason='waiting_for_B')
        deadline = time.monotonic() + 2.
        while not path.exists() and time.monotonic() < deadline:
            time.sleep(.01)
        state = latest_state(path)
        assert state['arms'][0]['target_q_deg'] == [1.] * 6
        assert mirror.error is None
    finally:
        mirror.close()
    assert not mirror.thread.is_alive()
