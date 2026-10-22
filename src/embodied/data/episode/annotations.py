"""Versioned, reviewable skill intervals supplied by people or explicit task events.

Frame indices are zero-based and both endpoints are inclusive. Timestamps come
from recorded camera receipt clocks, never sensor inference. Raw episodes and
their outcome labels are not changed. This module performs no robot operations.
"""
from contextlib import contextmanager
from bisect import bisect_left, bisect_right
import fcntl
import hashlib
import json
import math
from pathlib import Path
import time

from embodied.core.storage import save_json
from embodied.data.episode.cameras import camera_metadata
from embodied.data.episode.schema import read

SCHEMA = 'embodied.skill_annotations/1'


def _digest(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(chunk)
    return digest.hexdigest()


def _source(episode):
    _, metadata = read(episode)
    if metadata.get('lifecycle') == 'recording':
        raise ValueError('Finish recording before annotating its frames')
    if metadata.get('record_storage', {}).get('format') != 'mcap':
        raise ValueError('Skill intervals currently require an MCAP episode')
    # Outcome/review history is editable metadata, not a new recording identity.
    identity = {key: metadata.get(key) for key in ('id', 'schema', 'schema_version', 'kind', 'task',
        'started_monotonic_ns', 'data_contract', 'camera', 'cameras', 'anchor_camera', 'action_definition')}
    source = {'id': metadata['id'], 'manifest_sha256': _digest(episode / 'episode.json'),
              'records_sha256': _digest(episode / 'episode.mcap'),
              'identity_sha256': hashlib.sha256(json.dumps(identity, sort_keys=True).encode()).hexdigest()}
    return metadata, source


def _same_source(before, after):
    return all(before.get(key) == after.get(key) for key in ('id', 'records_sha256', 'identity_sha256'))


def _latest(episode):
    paths = sorted((episode / 'annotations').glob('skills.v[0-9]*.json'))
    if not paths:
        return None
    result = json.loads(paths[-1].read_text())
    if result.get('schema') != SCHEMA or paths[-1].name != f"skills.v{result['revision']:06d}.json":
        raise ValueError('Unknown or inconsistent skill annotation revision')
    return result


def read_annotations(episode):
    """Return the latest derived revision, or None; reject replaced source records."""
    episode = Path(episode)
    result = _latest(episode)
    if result is not None:
        _, source = _source(episode)
        if not _same_source(result['source_episode'], source):
            raise ValueError('Source episode changed after annotation; annotate the new source separately')
    return result


@contextmanager
def _locked(episode):
    directory = episode / 'annotations'
    directory.mkdir(exist_ok=True)
    with (directory / '.lock').open('a') as stream:
        fcntl.flock(stream.fileno(), fcntl.LOCK_EX)
        yield


def _text(value, field):
    if not isinstance(value, str) or not value.strip() or len(value) > 2000:
        raise ValueError(f'{field} requires nonempty text (at most 2000 characters)')
    return value.strip()


def _segments(segments, frames):
    if not isinstance(segments, list) or not segments or len(segments) > 10000:
        raise ValueError('Provide 1..10000 explicitly labelled intervals')
    result, previous = [], -1
    for segment in segments:
        if not isinstance(segment, dict):
            raise ValueError('Each interval must be an object')
        start, end = segment.get('start_frame'), segment.get('end_frame')
        if type(start) is not int or type(end) is not int or not previous < start <= end < len(frames):
            raise ValueError('Intervals must be ordered, nonoverlapping and inside the recorded camera frames')
        confidence = segment.get('confidence')
        if type(confidence) not in (float, int) or not math.isfinite(confidence) or not 0 <= confidence <= 1:
            raise ValueError('confidence must be explicitly supplied in [0, 1]')
        evidence = segment.get('evidence')
        if not isinstance(evidence, list) or not evidence or len(evidence) > 100:
            raise ValueError('Each interval needs explicit evidence or task event references')
        item = {'start_frame': start, 'end_frame': end,
                'start_monotonic_ns': frames[start]['host_received_monotonic_ns'],
                'end_monotonic_ns': frames[end]['host_received_monotonic_ns'],
                'phase': _text(segment.get('phase'), 'phase'),
                'object': _text(segment['object'], 'object') if segment.get('object') is not None else None,
                'evidence': [_text(value, 'evidence') for value in evidence], 'confidence': float(confidence)}
        for key in ('start_monotonic_ns', 'end_monotonic_ns'):
            if key in segment and segment[key] != item[key]:
                raise ValueError('Interval timestamp disagrees with the actual camera frame')
        result.append(item)
        previous = end
    return result


def _publish(episode, result):
    destination = episode / 'annotations' / f"skills.v{result['revision']:06d}.json"
    if destination.exists():
        raise ValueError('Annotation revision already exists')
    save_json(destination, result)
    return result


def write_annotations(episode, segments, *, camera_id, annotator, input_kind='manual', expected_revision=None,
                      source_events=None):
    """Save explicit intervals as a new needs_review revision; never infer a phase."""
    from embodied.data.episode.mcap_io import read_camera_records
    episode = Path(episode)
    annotator = _text(annotator, 'annotator')
    if input_kind not in ('manual', 'task_events'):
        raise ValueError('input_kind must be manual or task_events')
    metadata, source = _source(episode)
    if camera_id not in camera_metadata(metadata):
        raise ValueError('Annotation camera is not recorded in this episode')
    _, cameras = read_camera_records(episode / 'episode.mcap')
    frames = cameras.get(camera_id, [])
    stamps = [frame['host_received_monotonic_ns'] for frame in frames]
    if not stamps or any(type(value) is not int or value <= 0 for value in stamps) or any(a >= b for a, b in zip(stamps, stamps[1:])):
        raise ValueError('Annotation source camera timestamps must be present and increasing')
    segments = _segments(segments, frames)
    with _locked(episode):
        current = read_annotations(episode)
        revision = current['revision'] if current else 0
        if expected_revision is not None and expected_revision != revision:
            raise ValueError('Annotation revision changed; reload before saving')
        _, latest_source = _source(episode)
        if not _same_source(source, latest_source):
            raise ValueError('Source episode changed while annotating')
        return _publish(episode, {'schema': SCHEMA, 'revision': revision + 1,
            'source_episode': source, 'camera_id': camera_id, 'input_kind': input_kind,
            'annotator': annotator, 'created_unix_ns': time.time_ns(),
            'annotation_status': 'needs_review', 'segments': segments, 'review': None,
            'source_events': source_events})


def annotate_task_events(episode, events_path, *, annotator, confidence, camera_id=None, object_name=None):
    """Import explicit task phase boundaries, not inferred grasp/contact/success.

    Completed and failed phase attempts are both retained, with status in the
    evidence. Events outside the camera interval are listed in source_events.
    Unclosed phases are rejected rather than inventing an end time.
    """
    from embodied.data.episode.cameras import anchor_camera
    from embodied.data.episode.mcap_io import read_camera_records
    episode, events_path = Path(episode), Path(events_path)
    metadata, _ = _source(episode)
    camera_id = camera_id or anchor_camera(metadata)
    if camera_id not in camera_metadata(metadata):
        raise ValueError('Task annotation camera is not recorded')
    _, camera_frames = read_camera_records(episode / 'episode.mcap')
    frames = camera_frames.get(camera_id, [])
    stamps = [frame['host_received_monotonic_ns'] for frame in frames]
    if not stamps or any(type(value) is not int or value <= 0 for value in stamps) or any(a >= b for a, b in zip(stamps, stamps[1:])):
        raise ValueError('Task annotation needs ordered recorded camera timestamps')
    # Read once: the source digest identifies exactly the imported event bytes.
    event_bytes = events_path.read_bytes()
    events = [json.loads(line) for line in event_bytes.decode().splitlines() if line.strip()]
    pending, finished, intervals, omitted = {}, set(), [], []
    previous_stamp = 0
    for event in events:
        if not isinstance(event, dict) or event.get('schema_version') != 1:
            raise ValueError('Expected task event schema_version=1')
        key = (_text(event.get('task_id'), 'task_id'), event.get('phase_index'))
        phase, stamp = _text(event.get('phase'), 'phase'), event.get('host_monotonic_ns')
        if type(key[1]) is not int or key[1] < 0 or type(stamp) is not int or stamp <= 0 or stamp < previous_stamp:
            raise ValueError('Invalid or regressing task event index/timestamp')
        previous_stamp = stamp
        if event.get('status') == 'started':
            if key in pending or key in finished:
                raise ValueError('Duplicate task phase start')
            pending[key] = event
            continue
        if event.get('status') not in ('completed', 'failed') or key not in pending:
            raise ValueError('Task phase must have one start and one explicit completed/failed event')
        start = pending.pop(key)
        finished.add(key)
        if start['phase'] != phase or stamp < start['host_monotonic_ns']:
            raise ValueError('Task phase identity or interval changed')
        first, last = bisect_left(stamps, start['host_monotonic_ns']), bisect_right(stamps, stamp) - 1
        evidence = [f"task:{key[0]} phase:{key[1]} started:{start['host_monotonic_ns']}",
                    f"status:{event['status']} finished:{stamp} operation:{event.get('operation_id')}"]
        if event.get('error'):
            evidence.append('task error: ' + str(event['error']))
        if first > last or first >= len(stamps) or last < 0:
            omitted.append({'task_id': key[0], 'phase_index': key[1], 'reason': 'no-recorded-camera-frame-in-interval'})
            continue
        if start['host_monotonic_ns'] < stamps[0] or stamp > stamps[-1]:
            evidence.append('Interval clipped to recorded camera coverage')
        intervals.append({'start_frame': first, 'end_frame': last, 'phase': phase,
                          'object': object_name, 'evidence': evidence, 'confidence': confidence})
    if pending:
        raise ValueError('Unclosed task phases: no end time may be inferred')
    intervals.sort(key=lambda value: value['start_frame'])
    return write_annotations(episode, intervals, camera_id=camera_id, annotator=annotator,
        input_kind='task_events', source_events={'file': events_path.name,
            'sha256': hashlib.sha256(event_bytes).hexdigest(), 'events': len(events), 'omitted_intervals': omitted})


def review_annotations(episode, *, reviewer, decision, expected_revision, note=''):
    """Append a review revision without changing original intervals or episode bytes."""
    episode = Path(episode)
    reviewer = _text(reviewer, 'reviewer')
    if decision not in ('approved', 'rejected'):
        raise ValueError('Review decision must be approved or rejected')
    if type(expected_revision) is not int or expected_revision < 1:
        raise ValueError('Review requires an explicit positive expected_revision')
    if not isinstance(note, str) or len(note) > 2000:
        raise ValueError('Review note must be text up to 2000 characters')
    with _locked(episode):
        current = read_annotations(episode)
        if current is None:
            raise FileNotFoundError('No skill annotations to review')
        if current['revision'] != expected_revision:
            raise ValueError('Annotation revision changed; reload before reviewing')
        return _publish(episode, {**current, 'revision': current['revision'] + 1,
            'annotation_status': decision, 'review': {'reviewer': reviewer, 'decision': decision,
                'note': note, 'reviewed_unix_ns': time.time_ns(), 'source_revision': expected_revision}})
