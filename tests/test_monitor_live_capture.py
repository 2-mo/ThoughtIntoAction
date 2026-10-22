"""Offline heartbeat identity and 7-axis display fixtures; never opens a listener."""
import json
import os
from pathlib import Path
import time
from unittest.mock import Mock, patch

import pytest

from embodied.applications.monitor.capture_status import capture_snapshot, process_start_ticks
from embodied.applications.monitor.kernel_view import collection_snapshot, kernel_packet
from embodied.applications.monitor.model import RuntimeModel
from embodied.applications.monitor.scene import Scene, gripper_feedback


@pytest.fixture
def capture(tmp_path):
    identity = {'runtime_id': 'run-1', 'boot_id': 'boot-1'}
    manifest = {'id': 'episode-1', 'lifecycle': 'recording', 'task': 'test'}
    heartbeat = {'schema': 'embodied.capture_status/1', 'schema_version': 1, **identity,
        'pid': 10, 'process_start_ticks': '1234', 'host_monotonic_ns': 10_000_000_000,
        'started_monotonic_ns': 9_000_000_000, 'state': 'recording', 'episode': {'id': 'episode-1'},
        'counts': {'state': 20}, 'camera_counts': {'front': 4, 'overhead': 5, 'side': 6},
        'queue_size': 0, 'queue_capacity': 30, 'dropped': {}, 'error': None}
    def read(**changes):
        (tmp_path / 'capture-status.json').write_text(json.dumps({**heartbeat, **changes}))
        return capture_snapshot(tmp_path, identity, manifest, now_ns=10_100_000_000,
                                process_reader=lambda pid: '1234' if pid == 10 else None)
    return read, manifest


def test_live_counts_preserve_all_cameras_and_zero_queue(capture):
    read, _ = capture
    result = read()
    assert result['collection']['state'] == 'RECORDING'
    assert result['recording']['camera_counts'] == {'front': 4, 'overhead': 5, 'side': 6}
    assert result['recording']['queue_size'] == 0
    assert result['recording']['live']
    assert read(state='finalizing')['collection']['state'] == 'FINALIZING'


def test_stale_dead_and_pid_reused_cannot_claim_recording(capture):
    read, _ = capture
    assert read(host_monotonic_ns=7_000_000_000)['collection']['state'] == 'STALE'
    assert read(pid=11)['collection']['state'] == 'INTERRUPTED'
    assert read(process_start_ticks='different')['collection']['state'] == 'INTERRUPTED'
    assert not read(pid=11)['recording']['live']


@pytest.mark.parametrize('changes', [{'runtime_id': 'other'}, {'boot_id': 'other'},
    {'episode': {'id': 'different'}}, {'host_monotonic_ns': 11_000_000_000}])
def test_foreign_or_future_heartbeats_are_unverified(capture, changes):
    read, _ = capture
    result = read(**changes)
    assert result['collection']['state'] == 'UNVERIFIED'
    assert result['recording'] is None


def test_complete_requires_manifest_boundary_and_errors_remain_visible(capture):
    read, manifest = capture
    assert read(state='complete')['collection']['state'] == 'FINALIZING'
    result = read(state='failed', error='disk full')
    assert result['collection']['state'] == 'ERROR'
    assert result['recording']['error'] == 'disk full'
    manifest['lifecycle'] = 'complete'
    assert read() is None


def test_active_capture_is_not_hidden_by_a_later_completed_episode(tmp_path):
    identity = {'runtime_id': 'run', 'boot_id': 'boot'}
    now = time.monotonic_ns()
    for index, lifecycle in enumerate(('recording', 'complete')):
        episode = tmp_path / str(index)
        episode.mkdir()
        manifest = {'schema_version': 2, 'id': str(index), 'runtime': identity, 'lifecycle': lifecycle,
                    'started_monotonic_ns': now + index, 'cameras': {'front': {}, 'side': {}, 'wrist_rgb': {}},
                    'anchor_camera': 'front', 'counts': {'state': 3, 'frame:front': 1, 'frame:side': 2, 'frame': 3}}
        (episode / 'episode.json').write_text(json.dumps(manifest))
    heartbeat = {'schema': 'embodied.capture_status/1', 'schema_version': 1, **identity,
        'episode': {'id': '0'}, 'pid': os.getpid(), 'process_start_ticks': process_start_ticks(os.getpid()),
        'host_monotonic_ns': now, 'state': 'recording', 'camera_counts': {'front': 4, 'side': 6}}
    (tmp_path / '0/capture-status.json').write_text(json.dumps(heartbeat))
    result = collection_snapshot(identity, [tmp_path])
    assert result['collection']['state'] == 'RECORDING'
    assert result['recording']['id'] == '0'
    assert result['collection_count'] == 1
    heartbeat['host_monotonic_ns'] = now - 3_000_000_000
    (tmp_path / '0/capture-status.json').write_text(json.dumps(heartbeat))
    result = collection_snapshot(identity, [tmp_path])
    assert result['collection']['state'] == 'COMPLETE'
    assert result['episode_metadata']['camera_counts'] == {'front': 1, 'side': 2, 'wrist_rgb': 3}


def model_descriptor(tmp_path):
    links = ['base_link'] + [f'link{i}' for i in range(1, 7)] + ['tcp_link']
    shapes = ['<box size=".1 .1 .1"/>', '<sphere radius=".04"/>', '<cylinder radius=".02" length=".1"/>']
    body = [f'<link name="{name}"><visual><geometry>{shapes[i % 3]}</geometry></visual></link>' for i, name in enumerate(links)]
    for i in range(7):
        body.append(f'<joint name="joint{i + 1}" type="revolute"><parent link="{links[i]}"/><child link="{links[i + 1]}"/><origin xyz="0 0 .1"/><axis xyz="0 0 1"/><limit lower="-3" upper="3" effort="20" velocity="2"/></joint>')
    path = tmp_path / 'seven.urdf'
    text = '<robot name="seven">' + ''.join(body) + '</robot>'
    path.write_text(text)
    contract = {'schema': 'embodied.joint_vectors/1', 'representation': 'absolute_joint_position',
                'joint_names': [f'joint{i + 1}' for i in range(7)], 'joint_unit': 'rad', 'gripper': None}
    return {'joint_count': 7, 'data_contract': contract,
            'parameters': {'model': {'urdf_path': str(path), 'urdf': text, 'manifest': {'tcp_link': 'tcp_link'}}}}


def test_primitive_model_and_seven_axis_feedback_without_gripper(tmp_path):
    descriptor = model_descriptor(tmp_path)
    model = RuntimeModel(descriptor)
    with patch('embodied.applications.monitor.scene.ThreadingHTTPServer') as server, \
            patch('embodied.applications.monitor.scene.threading.Thread'):
        server.return_value.server_port = 18081
        scene = Scene(None)
        waiting_id = scene._manifest['viewer_id']
        scene.set_model(model)
        assert scene._manifest['viewer_id'] != waiting_id
        assert {item['primitive']['kind'] for item in scene._manifest['meshes']} == {'box', 'sphere', 'cylinder'}
        assert len(scene._manifest['reference']['actual']['q']) == 7
        assert len(scene._manifest['torque_limits']) == 7
        tick = {'runtime_id': 'seven', 'sequence': 0,
                'timing': {'feedback_received_ns': time.monotonic_ns()},
                'feedback': {'q': [.1] * 7, 'dq': [.2] * 7, 'gripper_width': 0}}
        packet = kernel_packet(model, tick, {'backend': 'sim'}, data_contract=descriptor['data_contract'])
        assert packet['gripper'] is None
        scene.accept(packet)
        assert scene.snapshot()['actual']['q'] == [.1] * 7
        assert scene.snapshot()['actual_vel'] == [.2] * 7
        assert scene.snapshot()['gripper'] is None
        scene.set_model(None)
        assert scene.snapshot()['actual'] is None
        assert scene._manifest['meshes'] == []
        scene.close()


def test_model_contract_mismatch_and_unknown_gripper_range(tmp_path):
    descriptor = model_descriptor(tmp_path)
    descriptor['data_contract']['joint_names'].reverse()
    with pytest.raises(ValueError, match='关节顺序'):
        RuntimeModel(descriptor)
    assert gripper_feedback({'actual': .025, 'max': None}) == {'actual': .025, 'target': None, 'max': None}
