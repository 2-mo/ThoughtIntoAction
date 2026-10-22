"""Versioned episode adapters for the existing LeRobot conversion pipeline."""
import json
import math
import numpy as np
from embodied.data.episode.cameras import camera_metadata, anchor_camera
from embodied.data.episode.contract import episode_contract, vector

# Kept camera anchors further apart than this many frame periods form a gap.
GAP_PERIODS = 1.5


def anchor_gaps(rows, fps=30):
    """Gaps between kept camera anchors (todo D-01).

    Exports stay spliced at 1/fps for LeRobot; each gap is recorded as an anomaly so
    training can filter by it later. Reasons are counted per episode
    (`exclusion_reasons`); a gap with no excluded anchors is a camera drop.
    """
    stamps = [row[3]['host_received_monotonic_ns'] for row in rows]
    period_ns = 1e9 / fps
    gaps = []
    for index in range(1, len(stamps)):
        delta = stamps[index] - stamps[index - 1]
        if delta > GAP_PERIODS * period_ns:
            gaps.append({'after_sample': index - 1, 'duration_ms': round(delta / 1e6, 1),
                         'missing_frames': int(round(delta / period_ns)) - 1})
    return gaps


def match_cameras(rows, others, max_skew_ns=60_000_000, *, anchor_id='wrist_rgb'):
    """Attach each camera's latest frame at or before the reference anchor, within max_skew_ns.

    Anchors without a frame from every camera are dropped (counted as camera-unmatched).
    """
    clocks = {key: np.asarray([f['host_received_monotonic_ns'] for f in frames], np.int64) for key, frames in others.items()}
    kept, unmatched = [], 0
    for observation, action, root, wrist, source in rows:
        stamp, matched = wrist['host_received_monotonic_ns'], {anchor_id: wrist}
        for key, clock in clocks.items():
            index = int(np.searchsorted(clock, stamp, side='right') - 1)
            if index < 0 or stamp - clock[index] > max_skew_ns:
                break
            matched[key] = others[key][index]
        if len(matched) == len(others) + 1:
            kept.append((observation, action, root, {**wrist, 'camera_frames': matched}, source))
        else:
            unmatched += 1
    return kept, unmatched


def camera_report(rows, frames, *, anchor_id='wrist_rgb'):
    """Per camera: source frames, repeated images and the age behind the reference anchor."""
    report = {}
    for key, source in frames.items():
        used = [row[3].get('camera_frames', {anchor_id: row[3]})[key] for row in rows]
        report[key] = {'source_frames': len(source),
                       'repeated_images': len(used) - len({f['host_received_monotonic_ns'] for f in used}),
                       'max_age_ms': max(r[3]['host_received_monotonic_ns'] - f['host_received_monotonic_ns']
                                         for r, f in zip(rows, used)) / 1e6}
    return report


def annotate(report, rows, *, control_stack, backend):
    gaps = anchor_gaps(rows)
    report.update(control_stack=control_stack, backend=backend, gaps=gaps, anomaly=bool(gaps))
    return report


def runtime_timing(metadata, raw):
    """Recorded receipt/submission rates, not an assumed control or hardware rate."""
    configured = metadata['runtime'].get('parameters', {}).get('effective_robot_config', {}).get('control_hz')
    if configured is not None and (type(configured) not in (int, float) or not math.isfinite(configured) or configured <= 0):
        raise ValueError('Recorded control_hz must be finite and positive')

    def rate(rows):
        if len(rows) < 2:
            return None
        duration = rows[-1]['host_monotonic_ns'] - rows[0]['host_monotonic_ns']
        return (len(rows) - 1) * 1e9 / duration if duration > 0 else None

    return {'configured_control_hz': configured, 'feedback_mean_hz': rate(raw['state']),
            'command_mean_hz': rate(raw['command']),
            'measurement': '(records - 1) / (last - first) host monotonic timestamp interval; gaps lower the mean'}


class Schema1:
    identity = (1, 'requested_joint_position + requested_gripper_width')
    layout = 'aligned30_raw240_v1'
    raw_hz = 240

    def prepare(self, path):
        from embodied.data.export.lerobot import prepare_schema1_camera_episode
        meta, rows, report, states = prepare_schema1_camera_episode(path)
        # The arm service records its Ruckig settings; other recorders sent IK targets directly.
        stack = ('legacy.robot_control.service: requested_pos is the Ruckig waypoint per cycle'
                 if (meta.get('control') or {}).get('trajectory_settings') else
                 'unknown: requested_pos may be the raw IK target (no arm-service trajectory settings)')
        annotate(report, rows, control_stack=stack, backend='real')
        return meta, rows, report, states

    def save_sidecar(self, path, rows, *, contract):
        from embodied.data.export.multirate import save_robot_sidecar
        save_robot_sidecar(path, rows)


class Schema2:
    identity = (2, 'sent_joint_position + sent_gripper_width')
    layout = 'aligned30_runtime_v2'
    raw_hz = None  # Independently timestamped streams; per-episode robot_timing reports actual means.

    def __init__(self, action_definition='sent_joint_position + sent_gripper_width'):
        self.identity = (2, action_definition)

    def prepare(self, path):
        from embodied.data.export.runtime_v2 import load_episode
        from embodied.data.alignment.runtime_alignment import align
        meta, raw = load_episode(path)
        self.identity = (2, meta['action_definition'])
        cameras = camera_metadata(meta)
        anchor_id = anchor_camera(meta)
        contract = episode_contract(meta)
        if meta['kind'] != 'quest_runtime_episode' or cameras[anchor_id]['fps'] != 30:
            raise ValueError('Training requires a runtime episode with a 30 FPS camera')
        if meta.get('outcome') in ('failure', 'discarded'):
            raise ValueError('Episode is labelled failure/discarded')
        from embodied.data.episode.runtime_episode import frame_kind
        reference = raw[frame_kind(anchor_id)]
        anchors, excluded = align(raw['state'], raw['command'], reference, sources=('quest', 'phone'), contract=contract)
        if len(anchors) < 2:
            raise ValueError('Insufficient causal position-following RGB anchors')
        parameters = meta['runtime'].get('parameters', {})
        # Identity uses content hashes, not machine-specific model paths.
        meta['robot'] = {'model': parameters['model']['manifest'], 'backend': meta['runtime']['backend'],
                         'action_definition': self.identity[1], 'data_contract': contract}
        meta['data_contract'] = contract
        rows = []
        for anchor in anchors:
            source = {'host_monotonic_ns': anchor['state_monotonic_ns'],
                      'requested_monotonic_ns': anchor['action_monotonic_ns'],
                      'state_sequence': anchor['state_sequence'], 'action_sequence': anchor['action_sequence']}
            rows.append((np.asarray(anchor['observation'], np.float32), np.asarray(anchor['action'], np.float32),
                         path, anchor['frame'], source))
        frames = {key: raw[frame_kind(key)] for key in cameras}
        rows, unmatched = match_cameras(rows, {k: v for k, v in frames.items() if k != anchor_id}, anchor_id=anchor_id)
        if unmatched:
            excluded = {**excluded, 'camera-unmatched': unmatched}
        if len(rows) < 2:
            raise ValueError('Insufficient anchors with every camera')
        report = {'id': meta['id'], 'task': meta['task'], 'source': str(path), 'outcome': meta['outcome'],
                  'samples': len(rows), 'robot_samples': len(raw['state']), 'observation_samples': len(rows),
                  'excluded_camera_anchors': len(reference)-len(rows), 'exclusion_reasons': excluded,
                  'start_monotonic_ns': rows[0][3]['host_received_monotonic_ns'],
                  'end_monotonic_ns': rows[-1][3]['host_received_monotonic_ns'],
                  'state_max_age_ms': max(r[3]['host_received_monotonic_ns']-r[4]['host_monotonic_ns'] for r in rows)/1e6,
                  'action_max_age_ms': max(r[3]['host_received_monotonic_ns']-r[4]['requested_monotonic_ns'] for r in rows)/1e6,
                  'image_max_age_ms': 0, 'repeated_images': 0, 'action_definition': self.identity[1],
                  'cameras': camera_report(rows, frames, anchor_id=anchor_id), 'anchor_camera': anchor_id,
                  'hand_eye': meta.get('hand_eye'), 'robot_timing': runtime_timing(meta, raw)}
        annotate(report, rows, control_stack='runtime kernel: submitted joint position commands',
                 backend=meta['runtime']['backend'])
        return meta, rows, report, raw

    def save_sidecar(self, path, rows, *, contract):
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(path, schema_version=np.asarray(2),
            feedback_timestamp_ns=np.asarray([r['host_monotonic_ns'] for r in rows['state']], np.int64),
            command_timestamp_ns=np.asarray([r['host_monotonic_ns'] for r in rows['command']], np.int64),
            state=np.asarray([vector(r, contract) for r in rows['state']], np.float64),
            action=np.asarray([vector(r, contract) for r in rows['command']], np.float64),
            state_sequence=np.asarray([r['sequence'] for r in rows['state']], np.int64),
            command_sequence=np.asarray([r['sequence'] for r in rows['command']], np.int64),
            raw_ticks_json=np.asarray([json.dumps(r, separators=(',', ':')) for r in rows['tick']]))


def for_episode(path, registry=None):
    from embodied.data.episode.schema import GROOT_RAW, RUNTIME, LEROBOT_EXPORTERS, read
    schema, metadata = read(path)
    if schema not in LEROBOT_EXPORTERS:
        hint = '; export it with ./run.sh collect groot export' if schema == GROOT_RAW else ''
        raise ValueError(f'{path.name}: {schema} episodes have no LeRobot adapter{hint}')
    # Data selects a schema, never a Python factory. Factory choices belong to
    # the trusted project registry, so a deployment can replace its exporter.
    from embodied.contracts.registry import Registry
    options = {'action_definition': metadata['action_definition']} if schema == RUNTIME else {}
    return (registry or Registry()).create('exporter', LEROBOT_EXPORTERS[schema], **options)
