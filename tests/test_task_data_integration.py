"""Actual task events -> recorded camera frames -> versioned console review, offline."""
from types import SimpleNamespace

import numpy as np
import pytest

from embodied.applications.monitor.review import ReviewStore
from embodied.data.episode.annotations import annotate_task_events
from embodied.data.episode.runtime_episode import RuntimeEpisodeWriter
from test_task_primitives import FakeRobot
from test_runtime_collection import tick


def test_task_events_reach_review_without_rewriting_raw_data(tmp_path):
    robot = FakeRobot(tmp_path / 'run')
    root = tmp_path / 'episodes'
    descriptor = {**robot.identity, 'layout': {'version': 1}, 'joint_count': 6,
                  'parameters': {'effective_robot_config': {'park_q': [0.] * 6, 'gripper_max_width': .08}}}
    writer = RuntimeEpisodeWriter(root, descriptor, 'approach and retreat',
        cameras={'external_rgb': {'stream_id': 'fixture', 'width': 8, 'height': 8, 'fps': 30}})
    frame_number = 0
    def record_frame(event):
        nonlocal frame_number
        writer.add_tick(tick(frame_number, event['host_monotonic_ns'] - 3_000_000))
        writer.add_frame({'stream_id': 'fixture', 'frame_number': frame_number,
            'host_received_monotonic_ns': event['host_monotonic_ns'],
            'image': np.zeros((8, 8, 3), dtype=np.uint8)}, 'external_rgb')
        frame_number += 1
    try:
        with robot.task('approach and retreat', on_event=record_frame) as task:
            task.approach(xyz=robot.xyz, offset=[0., 0., .02])
            task.retreat(offset=[0., 0., -.02])
    finally:
        writer.close('operator-stop')
    assert writer.manifest['lifecycle'] == 'complete'
    raw = {name: (writer.path / name).read_bytes() for name in ('episode.json', 'episode.mcap')}
    labels = annotate_task_events(writer.path, task.directory / 'events.jsonl', annotator='test', confidence=.8)
    assert [segment['phase'] for segment in labels['segments']] == ['approach', 'retreat']
    assert [(segment['start_frame'], segment['end_frame']) for segment in labels['segments']] == [(0, 1), (2, 3)]
    store = ReviewStore(SimpleNamespace(directory=robot.run, roots=[root]))
    assert store.tasks()[0]['task_id'] == task.result['task_id']
    assert store.annotations(writer.path.name)['annotation_status'] == 'needs_review'
    updated = store.review({'episode_id': writer.path.name, 'reviewer': 'reviewer',
                            'decision': 'approved', 'expected_revision': 1})
    assert updated['revision'] == 2 and updated['annotation_status'] == 'approved'
    with pytest.raises(ValueError, match='revision changed'):
        store.review({'episode_id': writer.path.name, 'reviewer': 'late-reviewer',
                      'decision': 'rejected', 'expected_revision': 1})
    assert {name: (writer.path / name).read_bytes() for name in raw} == raw
