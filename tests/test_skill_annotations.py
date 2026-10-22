"""Explicit intervals, immutable review history and source identity; no sensor inference."""
import json

import pytest

from embodied.data.episode.annotations import (read_annotations, write_annotations, review_annotations,
                                                annotate_task_events)
from embodied.data.episode.mcap_io import read_camera_records
from test_dataset_contracts import record_episode


def segment(**changes):
    return {'start_frame': 0, 'end_frame': 1, 'phase': 'grasp', 'object': 'cup',
            'evidence': ['operator observed contact'], 'confidence': .8, **changes}


def test_annotation_review_is_versioned_and_preserves_raw_episode(tmp_path):
    episode, _ = record_episode(tmp_path)
    raw = {name: (episode / name).read_bytes() for name in ('episode.json', 'episode.mcap')}
    assert read_annotations(episode) is None
    _, cameras = read_camera_records(episode / 'episode.mcap')
    first = write_annotations(episode, [segment()], camera_id='external_rgb', annotator='operator')
    assert first['annotation_status'] == 'needs_review'
    assert first['segments'][0]['start_monotonic_ns'] == cameras['external_rgb'][0]['host_received_monotonic_ns']
    review = review_annotations(episode, reviewer='reviewer', decision='approved', expected_revision=1, note='checked')
    assert review['revision'] == 2 and review['annotation_status'] == 'approved'
    assert review['segments'] == first['segments']
    assert read_annotations(episode) == review
    assert len(list((episode / 'annotations').glob('skills.v*.json'))) == 2
    assert json.loads((episode / 'annotations/skills.v000001.json').read_text()) == first
    assert {name: (episode / name).read_bytes() for name in raw} == raw
    with pytest.raises(ValueError, match='revision changed'):
        review_annotations(episode, reviewer='late reviewer', decision='rejected', expected_revision=1)
    with pytest.raises(ValueError, match='revision changed'):
        write_annotations(episode, [segment()], camera_id='external_rgb', annotator='late', expected_revision=1)


@pytest.mark.parametrize('segments', [[segment(confidence=float('nan'))], [segment(end_frame=100)],
    [segment(evidence=[])], [segment(start_monotonic_ns=1)], [segment(), segment(start_frame=1, end_frame=3)]])
def test_invalid_annotations_are_rejected_without_creating_revision(tmp_path, segments):
    episode, _ = record_episode(tmp_path)
    with pytest.raises(ValueError):
        write_annotations(episode, segments, camera_id='external_rgb', annotator='operator')
    assert read_annotations(episode) is None


def test_label_review_does_not_change_source_but_replaced_records_do(tmp_path):
    episode, _ = record_episode(tmp_path)
    write_annotations(episode, [segment()], camera_id='external_rgb', annotator='operator')
    metadata = json.loads((episode / 'episode.json').read_text())
    metadata.update(outcome='success', outcome_history=[{'outcome': 'success'}])
    (episode / 'episode.json').write_text(json.dumps(metadata))
    assert read_annotations(episode)['revision'] == 1
    with (episode / 'episode.mcap').open('ab') as stream:
        stream.write(b'changed source')
    with pytest.raises(ValueError, match='Source episode changed'):
        read_annotations(episode)


def test_explicit_task_events_keep_failed_attempt_evidence_and_source_digest(tmp_path):
    episode, _ = record_episode(tmp_path)
    _, cameras = read_camera_records(episode / 'episode.mcap')
    stamps = [frame['host_received_monotonic_ns'] for frame in cameras['external_rgb']]
    events = []
    for index, (phase, status, first, last) in enumerate([('grasp', 'completed', 0, 1), ('verify_grasp', 'failed', 2, 3)]):
        shared = {'schema_version': 1, 'task_id': 'task-one', 'phase_index': index,
                  'runtime_id': 'test', 'boot_id': 'test-boot',
                  'phase': phase, 'operation_id': 'op-one'}
        events.extend([{**shared, 'status': 'started', 'host_monotonic_ns': stamps[first]},
                       {**shared, 'status': status, 'host_monotonic_ns': stamps[last],
                        'error': 'No contact evidence' if status == 'failed' else None}])
    source = tmp_path / 'events.jsonl'
    source.write_text('\n'.join(json.dumps(event) for event in events))
    result = annotate_task_events(episode, source, annotator='task operator', confidence=.9, object_name='cup')
    assert result['input_kind'] == 'task_events' and result['annotation_status'] == 'needs_review'
    assert [(item['start_frame'], item['end_frame']) for item in result['segments']] == [(0, 1), (2, 3)]
    assert result['segments'][1]['phase'] == 'verify_grasp'
    assert any('status:failed' in item for item in result['segments'][1]['evidence'])
    assert len(result['source_events']['sha256']) == 64
    for identity in ({'runtime_id': 'another-run'}, {'boot_id': 'another-boot'}):
        source.write_text('\n'.join(json.dumps({**event, **identity}) for event in events))
        with pytest.raises(ValueError, match='different runtime or host boot'):
            annotate_task_events(episode, source, annotator='operator', confidence=.9)
    source.write_text(json.dumps(events[0]))
    with pytest.raises(ValueError, match='Unclosed task phases'):
        annotate_task_events(episode, source, annotator='operator', confidence=.9)
