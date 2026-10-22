"""Offline projections and review dispatch: no HTTP listener, SDK, or hardware."""
from contextlib import nullcontext
import io
import json
from pathlib import Path
from types import SimpleNamespace
import threading
from unittest.mock import Mock, patch

import pytest

from embodied.applications.monitor.quality import MotionQuality, camera_quality
from embodied.applications.monitor.review import ReviewStore


def tick(sequence=0, stamp=1_000_000_000):
    return {'runtime_id': 'fixture', 'sequence': sequence,
            'feedback': {'q': [.1, .2], 'dq': [0., -.3]},
            'command': {'q': [.2, .4], 'dq': [9., 9.]},
            'timing': {'feedback_received_ns': stamp, 'command_submitted_ns': stamp + 2_000_000,
                       'tick_ns': stamp}, 'producer_dropped': 0, 'observer_gaps': 0}


def test_speed_is_feedback_and_different_command_time_is_explicit():
    quality = MotionQuality()
    quality.accept(tick())
    result = quality.snapshot(1_001_000_000)
    assert result['joint_velocity_rad_s'] == [0., -.3]
    assert result['peak_joint_speed_rad_s'] == .3
    assert result['target_feedback_error_rad'] == pytest.approx(.2)
    assert result['command_after_feedback_ms'] == 2.
    assert result['cycle_std_ms'] is None
    assert quality.snapshot(2_000_000_000)['peak_joint_speed_rad_s'] is None
    quality.accept({**tick(1), 'feedback': {'q': [.1, .2], 'dq': None}})
    assert quality.snapshot(1_001_000_000)['peak_joint_speed_rad_s'] is None


def test_cycle_statistics_exclude_latest_observer_skips_and_reset_on_runtime_change():
    quality = MotionQuality()
    for sequence, delta in [(0, 0), (1, 4), (4, 40), (5, 46)]:
        quality.accept(tick(sequence, 1_000_000_000 + delta * 1_000_000))
    result = quality.snapshot(1_047_000_000)
    assert result['cycle_mean_ms'] == 5.
    assert result['cycle_std_ms'] == 1.
    assert result['cycle_max_ms'] == 6.
    assert result['cycle_samples'] == 2
    assert result['monitor_skipped_ticks'] == 2
    quality.accept({**tick(), 'runtime_id': 'new'})
    assert quality.snapshot(1_001_000_000)['cycle_samples'] == 0
    assert quality.snapshot(1_001_000_000)['monitor_skipped_ticks'] == 0


def test_camera_quality_uses_capture_timestamp_not_preview_or_poll_number_gaps():
    payload = {'motion_quality': {'feedback_monotonic_ns': 1_000_000_000, 'valid': True},
               'cameras': {'test': {'camera': {'enabled': True, 'age_s': .01,
                   'host_received_monotonic_ns': 1_003_000_000, 'frame_number': 400,
                   'source_missing_frames': 2}, 'preview': {'age_s': .15}}}}
    camera = camera_quality(payload)['test']
    assert camera['valid']
    assert camera['robot_receipt_delta_ms'] == 3
    assert camera['source_missing_frames'] == 2
    assert camera['frame_age_s'] != camera['preview_age_s']
    payload['cameras']['test']['camera']['age_s'] = 1
    assert not camera_quality(payload)['test']['valid']
    assert camera_quality(payload)['test']['robot_receipt_delta_ms'] is None
    del payload['cameras']['test']['camera']['source_missing_frames']
    assert camera_quality(payload)['test']['source_missing_frames'] is None


def test_source_frame_loss_is_counted_at_capture_and_resets_with_stream():
    from embodied.devices.sensors.service import CameraFeed
    feed = CameraFeed()
    feed.enabled = True
    camera = Mock(metadata={})
    registry = Mock()
    registry.spec.return_value.hardware = False
    registry.create.side_effect = lambda *a, **kw: nullcontext(camera)
    feed.registry = registry
    for numbers, expected in [([10, 13], 2), ([100, 101], 0)]:
        stop = threading.Event()
        def read():
            number = numbers.pop(0)
            if not numbers:
                stop.set()
            return {'frame_number': number, 'host_received_monotonic_ns': 123,
                    'image': SimpleNamespace(shape=(1, 1))}
        camera.read.side_effect = read
        with patch.dict('sys.modules', {'cv2': Mock()}), patch('embodied.devices.sensors.service.time.monotonic', return_value=0):
            feed.capture({}, stop)
        snapshot = feed.snapshot()['camera']
        assert snapshot['source_missing_frames'] == expected
        assert snapshot['host_received_monotonic_ns'] == 123


def test_episode_review_is_confined_and_only_accepts_complete_recordings(tmp_path):
    root = tmp_path / 'episodes'
    root.mkdir()
    episode = root / 'episode-one'
    episode.mkdir()
    manifest = episode / 'episode.json'
    manifest.write_text(json.dumps({'schema_version': 2, 'lifecycle': 'complete', 'started_monotonic_ns': 2}))
    store = ReviewStore(SimpleNamespace(roots=[root], directory=None))
    assert store.episode_path('episode-one') == episode
    assert [row['id'] for row in store.episodes()] == ['episode-one']
    with pytest.raises(ValueError):
        store.episode_path('../outside')
    outside = tmp_path / 'outside'
    outside.mkdir()
    (outside / 'episode.json').write_text(manifest.read_text())
    (root / 'linked').symlink_to(outside, target_is_directory=True)
    with pytest.raises(ValueError):
        store.episode_path('linked')
    manifest.write_text(json.dumps({'schema_version': 2, 'lifecycle': 'recording'}))
    with pytest.raises(ValueError, match='complete'):
        store.episode_path('episode-one')


def test_task_timeline_reads_phases_without_submitting_any_operation(tmp_path):
    task = tmp_path / 'tasks' / 'task-one'
    task.mkdir(parents=True)
    document = {'schema_version': 1, 'task_id': 'task-one', 'status': 'failed', 'started_unix_ns': 5,
                'phases': [{'phase': 'verify_grasp', 'status': 'failed', 'operation_id': 'move-1'}]}
    (task / 'result.json').write_text(json.dumps(document))
    store = ReviewStore(SimpleNamespace(roots=[], directory=tmp_path))
    assert store.tasks() == [document]


def test_review_routes_do_not_start_server_and_forward_revision():
    from embodied.applications.monitor.scene import Scene
    with patch.dict('sys.modules', {'pinocchio': Mock()}), \
            patch('embodied.applications.monitor.scene.ThreadingHTTPServer') as server, \
            patch('embodied.applications.monitor.scene.threading.Thread'), patch.object(Scene, 'set_model'):
        server.return_value.server_port = 18081
        scene = Scene(Mock(), runtime_monitor=SimpleNamespace(roots=[], directory=None))
        handler_type = server.call_args.args[1]
        scene.review_store = Mock()
        scene.review_store.review.return_value = {'revision': 3, 'annotation_status': 'approved'}
        handler = handler_type.__new__(handler_type)
        request = {'episode_id': 'e', 'expected_revision': 2, 'decision': 'approved', 'reviewer': 'tester'}
        body = json.dumps(request).encode()
        handler.path = '/api/annotations/review'
        handler.headers = {'Content-Length': str(len(body)), 'Origin': scene.url.rstrip('/')}
        handler.rfile, handler.wfile = io.BytesIO(body), io.BytesIO()
        handler.send_response = handler.send_header = handler.end_headers = Mock()
        handler.do_POST()
        scene.review_store.review.assert_called_once_with(request)
        assert json.loads(handler.wfile.getvalue())['annotations']['revision'] == 3
        scene.close()
