"""Offline task/collector/event/export integration; no controller or camera service."""
import copy
import json
from pathlib import Path
import time
from types import SimpleNamespace

import numpy as np
import pytest

from embodied.applications.operations.recording import record_task
from embodied.applications.operations.tasks import Task
from embodied.data.capture.session import CaptureSession
from embodied.data.episode.annotations import read_annotations
from embodied.data.export.lerobot import prepare_camera_episode
from embodied.data.export.runtime_v2 import load_episode
from test_runtime_collection import tick


CONTRACT = {'schema': 'embodied.joint_vectors/1', 'representation': 'absolute_joint_position',
            'joint_names': [f'axis_{i}' for i in range(7)], 'joint_unit': 'rad', 'gripper': None,
            'robot_type': 'offline-seven'}
ROLES = {'base_0_rgb': 'front_rgb'}


class Reader:
    mode, terminal = 'full', False

    def __init__(self, *args, **kwargs):
        self.descriptor = {'runtime_id': 'test', 'boot_id': 'test-boot', 'joint_count': 7,
            'backend': 'sim', 'layout': {'version': 3}, 'data_contract': copy.deepcopy(CONTRACT),
            'parameters': {'model': {'manifest': {'sha256': 'offline-model'}}}}
        self.sequence, self.closed, self.error = 0, False, None

    def read(self):
        time.sleep(.003)
        if self.closed:
            raise EOFError('observer closed')
        row = tick(self.sequence, time.monotonic_ns() - 3_000_000)
        row.update(phase=4, source='robot-api', owner_epoch=1, version=3, joint_count=7, error=self.error)
        for key in ('feedback', 'command'):
            row[key].update(q=[.1] * 7, dq=[0.] * 7, gripper_width=None, gripper_mode=None)
        row['command'].update(kp=[1.] * 7, kd=[.1] * 7, feedforward=[0.] * 7)
        self.sequence += 1
        return row

    def close(self):
        self.closed = True


class Camera:
    metadata = {'stream_id': 'front-stream', 'width': 8, 'height': 8, 'fps': 30}

    def __init__(self):
        self.index = 0

    def read(self):
        time.sleep(1 / 30)
        self.index += 1
        return {'image': np.zeros((8, 8, 3), np.uint8), 'stream_id': 'front-stream',
                'frame_number': self.index, 'host_received_monotonic_ns': time.monotonic_ns()}


class Robot:
    def __init__(self, run, *, fail=False):
        self.run, self.fail, self.cancelled = run, fail, False

    def _connected(self):
        return SimpleNamespace(identity={'runtime_id': 'test', 'boot_id': 'test-boot'})

    def _acquire(self):
        pass

    def _cancel_on_error(self):
        self.cancelled = True

    def task(self, name):
        return Task(self, name)

    def movel(self, *args, **kwargs):
        def wait(**kwargs):
            time.sleep(.32)
            if self.fail:
                raise RuntimeError('explicit offline motion failure')
            return {'completed': True}
        return SimpleNamespace(operation_id='offline-op', wait=wait)


@pytest.mark.parametrize('with_scene', [False, True])
def test_scripted_phase_four_task_to_camera_annotation_and_export(tmp_path, monkeypatch, with_scene):
    monkeypatch.setattr('embodied.data.capture.session.RuntimeReader', Reader)
    scene = SimpleNamespace(describe=lambda: {'language': 'return to initial pose', 'objects': []},
        simulation=SimpleNamespace(snapshot=lambda: {'schema': 'offline-snapshot', 'q': [0.] * 7}),
        check_success=lambda: True) if with_scene else None
    recording = record_task(Robot(tmp_path / 'run'), 'scripted demonstration',
        cameras={'front_rgb': Camera()}, camera_roles=ROLES, scene=scene, output=tmp_path / 'data',
        annotator='offline task events', confidence=.8, video=False)
    with recording as task:
        task.retreat(offset=(0., 0., .01))
        status = json.loads((recording.path / 'capture-status.json').read_text())
        assert status['state'] == 'recording'
        assert status['camera_counts']['front_rgb'] >= 2
        assert status['counts']['state'] > 2
        assert status['queue_capacity'] == 64 and status['dropped'] == {'front_rgb': 0}
        assert status['pid'] > 0 and status['process_start_ticks'].isdigit()
        assert time.monotonic_ns() - status['host_monotonic_ns'] < 2_000_000_000
    meta, rows, report, raw = prepare_camera_episode(recording.path)
    assert meta['lifecycle'] == 'complete'
    assert meta['outcome'] == ('success' if with_scene else 'unknown')
    assert meta['task_execution']['status'] == 'completed'
    assert report['accepted_sources'] == ['robot-api'] and 4 in report['accepted_phases']
    assert all(row['phase'] == 4 and row['source'] == 'robot-api' for row in raw['tick'])
    assert rows[0][0].shape == rows[0][1].shape == (7,)
    assert report['camera_roles'] == ROLES
    assert (recording.path / 'task/events.jsonl').is_file()
    annotation = read_annotations(recording.path)
    assert annotation['annotation_status'] == 'needs_review'
    assert annotation['segments'][0]['phase'] == 'retreat'
    status = json.loads((recording.path / 'capture-status.json').read_text())
    assert status['state'] == 'complete' and status['stop_reason'] == 'context-exit'
    assert status['camera_counts']['front_rgb'] == meta['counts']['frame:front_rgb']
    assert status['counts'] == meta['counts']
    if with_scene:
        assert json.loads((recording.path / 'scene.json').read_text())['recording_start_snapshot']['q'] == [0.] * 7

    # Removing the authorization cannot make robot-api traffic human demonstrations.
    meta.pop('demonstration')
    (recording.path / 'episode.json').write_text(json.dumps(meta))
    with pytest.raises(ValueError, match='Insufficient causal'):
        prepare_camera_episode(recording.path)


def test_failed_task_retains_events_frames_and_failed_annotation(tmp_path, monkeypatch):
    monkeypatch.setattr('embodied.data.capture.session.RuntimeReader', Reader)
    robot = Robot(tmp_path / 'run', fail=True)
    recording = record_task(robot, 'failed attempt', cameras={'front_rgb': Camera()}, camera_roles=ROLES,
        output=tmp_path / 'data', annotator='offline events', confidence=.7, video=False)
    with pytest.raises(RuntimeError, match='offline motion failure'):
        with recording as task:
            task.retreat(offset=(0., 0., .01))
    meta, raw = load_episode(recording.path)
    assert meta['lifecycle'] == 'complete' and meta['outcome'] == 'failure'
    assert robot.cancelled and meta['task_execution']['status'] == 'failed'
    assert raw['frame:front_rgb']
    assert any('status:failed' in evidence for evidence in read_annotations(recording.path)['segments'][0]['evidence'])
    with pytest.raises(ValueError, match='failure/discarded'):
        prepare_camera_episode(recording.path)


def test_capture_stream_failure_final_status_is_failed_not_complete(tmp_path):
    reader = Reader()
    session = CaptureSession(tmp_path, output=tmp_path / 'data', task='failure',
        cameras={'front_rgb': Camera()}, video=False, reader=reader)
    with pytest.raises(RuntimeError, match='Capture failed'):
        with session:
            reader.error = 'offline observer fault'
            assert session.finished.wait(2)
    meta = json.loads((session.path / 'episode.json').read_text())
    status = json.loads((session.path / 'capture-status.json').read_text())
    assert meta['lifecycle'] == status['state'] == 'failed'
    assert 'observer fault' in status['error']


def test_final_manifest_write_failure_retains_uncommitted_episode_and_stops_heartbeat(tmp_path, monkeypatch):
    from embodied.data.episode import runtime_episode
    original = runtime_episode.save_json
    def fail_final_manifest(path, value):
        if value.get('lifecycle') == 'complete':
            raise OSError('offline final manifest disk error')
        return original(path, value)
    monkeypatch.setattr(runtime_episode, 'save_json', fail_final_manifest)
    session = CaptureSession(tmp_path, output=tmp_path / 'data', task='disk failure',
        cameras={'front_rgb': Camera()}, video=False, reader=Reader())
    with pytest.raises(RuntimeError, match='final manifest disk error'):
        with session:
            pass
    assert json.loads((session.path / 'episode.json').read_text())['lifecycle'] == 'recording'
    status = json.loads((session.path / 'capture-status.json').read_text())
    assert status['state'] == 'failed' and 'disk error' in status['error']
    assert not session.thread.is_alive() and session.writer.closed


def test_camera_queue_overflow_is_counted_and_fails_recording(tmp_path):
    from embodied.data.capture.capture import capture
    class FastCamera(Camera):
        def read(self):
            self.index += 1
            return {'image': np.zeros((8, 8, 3), np.uint8), 'stream_id': 'front-stream',
                    'frame_number': self.index, 'host_received_monotonic_ns': time.monotonic_ns()}
    path, meta = capture(Reader(), tmp_path, 'overflow', 0, cameras={'front_rgb': FastCamera()})
    status = json.loads((path / 'capture-status.json').read_text())
    assert meta['lifecycle'] == status['state'] == 'failed'
    assert status['dropped'] == {'front_rgb': 1}
    assert 'queue overflow' in status['error']


def test_scripted_provenance_does_not_bypass_submitted_or_owner_boundary():
    from embodied.data.alignment.runtime_alignment import align
    from embodied.data.episode.provenance import training_sources, training_phases, validate_demonstration
    metadata = {'demonstration': {'schema': 'embodied.demonstration/1', 'kind': 'scripted',
                                'sources': ['robot-api'], 'task_id': 'task'},
                'task_execution': {'task_id': 'task', 'status': 'completed'}}
    sources, phases = training_sources(metadata), training_phases(metadata)
    state = {'host_monotonic_ns': 1, 'sequence': 0, 'phase': 4, 'source': 'robot-api', 'owner_epoch': 1,
             'error': None, 'q': [.1] * 7, 'mode': 3}
    command = dict(state, host_monotonic_ns=2, submitted=True)
    frames = [{'host_received_monotonic_ns': 3}]
    for changes, reason in [({'submitted': False}, 'command-not-submitted'),
                            ({'owner_epoch': 2}, 'owner-boundary'),
                            ({'phase': 2}, 'phase-source-or-error'), ({'source': 'policy'}, 'owner-boundary')]:
        rows, excluded = align([state], [{**command, **changes}], frames, sources=sources, phases=phases, contract=CONTRACT)
        assert not rows and excluded == {reason: 1}
    metadata['task_execution']['status'] = 'failed'
    with pytest.raises(ValueError, match='completed task'):
        training_sources(metadata)
    with pytest.raises(ValueError, match='sources'):
        validate_demonstration({**metadata['demonstration'], 'sources': ['policy']})
