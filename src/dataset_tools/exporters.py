"""Versioned episode adapters for the existing LeRobot conversion pipeline."""
import json
import numpy as np


class Schema1:
    identity = (1, 'requested_joint_position + requested_gripper_width')
    layout = 'aligned30_raw240_v1'
    raw_hz = 240

    def prepare(self, path):
        from quest_teleop.export_lerobot import prepare_schema1_camera_episode
        return prepare_schema1_camera_episode(path)

    def save_sidecar(self, path, rows):
        from quest_teleop.multirate import save_robot_sidecar
        save_robot_sidecar(path, rows)


class Schema2:
    identity = (2, 'sent_joint_position + sent_gripper_width')
    layout = 'aligned30_raw250_v2'
    raw_hz = 250

    def prepare(self, path):
        from dataset_tools.runtime_v2 import load_episode
        from observations.runtime_alignment import align
        meta, raw = load_episode(path)
        if meta['kind'] != 'quest_runtime_episode' or meta['camera']['fps'] != 30:
            raise ValueError('Training requires a runtime episode with a 30 FPS camera')
        if meta.get('outcome') in ('failure', 'discarded'):
            raise ValueError('Episode is labelled failure/discarded')
        anchors, excluded = align(raw['state'], raw['command'], raw['frame'], sources=('quest', 'phone'))
        if len(anchors) < 2:
            raise ValueError('Insufficient causal position-following RGB anchors')
        parameters = meta['runtime']['parameters']
        # Identity uses content hashes, not machine-specific model paths.
        meta['robot'] = {'model': parameters['model']['manifest'], 'backend': meta['runtime']['backend'],
                         'action_definition': self.identity[1]}
        rows = []
        for anchor in anchors:
            source = {'host_monotonic_ns': anchor['state_monotonic_ns'],
                      'requested_monotonic_ns': anchor['action_monotonic_ns'],
                      'state_sequence': anchor['state_sequence'], 'action_sequence': anchor['action_sequence']}
            rows.append((np.asarray(anchor['observation'], np.float32), np.asarray(anchor['action'], np.float32),
                         path, {**anchor['frame'], 'wrist_valid': True}, source))
        report = {'id': meta['id'], 'task': meta['task'], 'source': str(path), 'outcome': meta['outcome'],
                  'samples': len(rows), 'robot_samples': len(raw['state']), 'observation_samples': len(rows),
                  'excluded_camera_anchors': len(raw['frame'])-len(rows), 'exclusion_reasons': excluded,
                  'start_monotonic_ns': rows[0][3]['host_received_monotonic_ns'],
                  'end_monotonic_ns': rows[-1][3]['host_received_monotonic_ns'],
                  'state_max_age_ms': max(r[3]['host_received_monotonic_ns']-r[4]['host_monotonic_ns'] for r in rows)/1e6,
                  'action_max_age_ms': max(r[3]['host_received_monotonic_ns']-r[4]['requested_monotonic_ns'] for r in rows)/1e6,
                  'image_max_age_ms': 0, 'repeated_images': 0, 'action_definition': self.identity[1]}
        return meta, rows, report, raw

    def save_sidecar(self, path, rows):
        path.parent.mkdir(parents=True, exist_ok=True)
        np.savez_compressed(path, schema_version=np.asarray(2),
            feedback_timestamp_ns=np.asarray([r['host_monotonic_ns'] for r in rows['state']], np.int64),
            command_timestamp_ns=np.asarray([r['host_monotonic_ns'] for r in rows['command']], np.int64),
            state=np.asarray([[*r['q'], r['gripper_width']] for r in rows['state']], np.float64),
            action=np.asarray([[*r['q'], r['gripper_width']] for r in rows['command']], np.float64),
            state_sequence=np.asarray([r['sequence'] for r in rows['state']], np.int64),
            command_sequence=np.asarray([r['sequence'] for r in rows['command']], np.int64),
            raw_ticks_json=np.asarray([json.dumps(r, separators=(',', ':')) for r in rows['tick']]))


def for_episode(path, registry=None):
    meta = json.loads((path / 'episode.json').read_text())
    version = meta.get('schema_version')
    if version not in (1, 2):
        raise ValueError('Unsupported episode schema')
    # Data selects a schema, never a Python factory. Factory choices belong to
    # the trusted project registry, so a deployment can replace its exporter.
    from component_api.registry import Registry
    return (registry or Registry()).create('exporter', f'schema{version}')
