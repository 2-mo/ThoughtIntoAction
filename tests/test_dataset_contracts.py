"""Offline MCAP -> causal rows/features/sidecar for seven axes and no wrist camera."""
import copy
import json
import time

import numpy as np
import pytest

from embodied.data.episode.contract import episode_contract, runtime_contract, feature_names
from embodied.data.episode.runtime_episode import RuntimeEpisodeWriter
from embodied.data.export.exporters import Schema2
from embodied.data.export.lerobot import export_features, prepare_camera_episode
from embodied.data.export.runtime_v2 import load_episode
from test_runtime_collection import tick


def record_episode(root, *, count=7, gripper=True, frame_numbers=None):
    names = [f'axis_{i}' for i in range(count)]
    descriptor = {'runtime_id': 'test', 'boot_id': 'test-boot', 'joint_count': count, 'layout': {'version': 3}, 'backend': 'sim',
                  'parameters': {'model': {'manifest': {'sha256': 'example'}, 'joint_names': names},
                                 'effective_robot_config': {'park_q': [0.] * count,
                                                            'gripper_max_width': .12, 'name': 'example-arm'}}}
    if not gripper:
        descriptor['data_contract'] = {**runtime_contract(descriptor), 'gripper': None}
    camera = {'stream_id': 'external', 'width': 8, 'height': 8, 'fps': 30}
    writer = RuntimeEpisodeWriter(root, descriptor, 'explicit test task', cameras={'external_rgb': camera})
    origin = time.monotonic_ns()
    ticks = []
    for index in range(41):
        row = tick(index, origin + index * 4_000_000, terminal=index == 40)
        row.update(version=3, joint_count=count)
        if not row['terminal']:
            for key in ('feedback', 'command'):
                row[key]['q'] = [index * .001 + i * .01 for i in range(count)]
                row[key]['dq'] = [.01] * count
                row[key]['gripper_width'] = .1 if gripper else None
                row[key]['gripper_mode'] = 3 if gripper else None
            row['command'].update(kp=[1.] * count, kd=[.1] * count, feedforward=[0.] * count)
        ticks.append(copy.deepcopy(row))
        writer.add_tick(row)
    numbers = list(frame_numbers) if frame_numbers else list(range(4))
    for index, number in enumerate(numbers):
        # .037 s is after tick 9 feedback but before tick 9 command submission.
        writer.add_frame({'stream_id': 'external', 'frame_number': number,
            'host_received_monotonic_ns': origin + 37_000_000 + index * 33_000_000,
            'image': np.full((8, 8, 3), index, np.uint8)}, 'external_rgb')
    writer.close('runtime-terminal')
    return writer.path, ticks


@pytest.mark.parametrize('gripper', [True, False])
def test_seven_axes_external_camera_export_keeps_exact_dimensions_and_clocks(tmp_path, gripper):
    path, original = record_episode(tmp_path, gripper=gripper)
    metadata, raw = load_episode(path)
    assert raw['tick'] == original
    assert 'frame' not in metadata['counts']  # No phantom wrist channel.
    assert metadata['camera'] is None and metadata['anchor_camera'] == 'external_rgb'
    assert metadata['data_contract']['joint_names'] == [f'axis_{i}' for i in range(7)]
    meta, rows, report, raw = prepare_camera_episode(path)
    assert len(rows) == 4
    dimension = 8 if gripper else 7
    assert rows[0][0].shape == rows[0][1].shape == (dimension,)
    assert rows[0][0][:7].tolist() == pytest.approx(original[9]['feedback']['q'])
    assert rows[0][1][:7].tolist() == pytest.approx(original[8]['command']['q'])
    assert rows[0][4]['requested_monotonic_ns'] < rows[0][4]['host_monotonic_ns']
    assert set(rows[0][3]['camera_frames']) == {'external_rgb'}
    assert report['anchor_camera'] == 'external_rgb' and not report['exclusion_reasons']
    assert report['control_stack'] == 'runtime kernel: submitted joint position commands'
    assert report['robot_timing']['configured_control_hz'] is None
    assert report['robot_timing']['feedback_mean_hz'] == report['robot_timing']['command_mean_hz'] == 250.
    features = export_features(episode_contract(meta), meta['cameras'])
    assert features['action']['shape'] == (dimension,)
    assert features['action']['names'] == feature_names(meta['data_contract'])
    assert 'observation.images.external' in features and 'observation.images.wrist' not in features
    assert 'observation.wrist_valid' not in features
    sidecar = tmp_path / 'sidecar.npz'
    Schema2().save_sidecar(sidecar, raw, contract=meta['data_contract'])
    with np.load(sidecar) as data:
        assert data['state'].shape == data['action'].shape == (40, dimension)
        assert data['action'][8, :7].tolist() == original[8]['command']['q']


def test_declared_vectors_reject_wrong_dimensions_and_action_semantics(tmp_path):
    path, _ = record_episode(tmp_path)
    manifest_path = path / 'episode.json'
    manifest = json.loads(manifest_path.read_text())
    manifest['data_contract']['joint_names'].pop()
    manifest_path.write_text(json.dumps(manifest))
    with pytest.raises(ValueError, match='joint_count'):
        prepare_camera_episode(path)
    with pytest.raises(ValueError, match='Unsupported robot vector contract'):
        runtime_contract({'data_contract': {'schema': 'embodied.joint_vectors/1', 'representation': 'cartesian_delta'}})


def test_six_axes_alone_do_not_imply_arx_gripper_calibration():
    contract = runtime_contract({'joint_count': 6, 'layout': {'version': 3}})
    assert contract['gripper']['range'] == [0., None]


def test_measured_width_just_past_the_nominal_maximum_is_recordable():
    from embodied.data.episode.contract import vector

    contract = runtime_contract({'joint_count': 6, 'layout': {'version': 3},
                                 'parameters': {'effective_robot_config': {'park_q': [0.] * 6,
                                                                           'gripper_max_width': .082}}})
    # The X5 reports 82.15 mm against an 82 mm nominal limit.
    row = {'q': [0.] * 6, 'gripper_width': .08215163959228675}
    assert vector(row, contract)[-1] == .08215163959228675


def test_original_schema_two_without_contract_remains_readable(tmp_path):
    from test_data_review_runtime import episode
    path = episode(tmp_path)
    manifest = json.loads((path / 'episode.json').read_text())
    for key in ('data_contract', 'cameras', 'anchor_camera'):
        manifest.pop(key)
    (path / 'episode.json').write_text(json.dumps(manifest))
    meta, rows, _, _ = prepare_camera_episode(path)
    assert rows[0][0].shape == (7,)
    # 82 mm nominal plus the margin that lets measured widths past the limit in.
    assert meta['data_contract']['gripper']['range'] == [0., pytest.approx(.084)]


@pytest.mark.parametrize('outcome', ['failure', 'discarded'])
def test_generic_export_preserves_outcome_rejection(tmp_path, outcome):
    path, _ = record_episode(tmp_path)
    metadata = json.loads((path / 'episode.json').read_text())
    metadata['outcome'] = outcome
    (path / 'episode.json').write_text(json.dumps(metadata))
    with pytest.raises(ValueError, match='failure/discarded'):
        prepare_camera_episode(path)


def test_large_camera_gap_still_fails_the_take(tmp_path):
    """A long blind stretch is not a usable demonstration."""
    from embodied.data.episode.runtime_episode import MAX_TOLERATED_FRAME_GAP
    with pytest.raises(ValueError, match='exceeds the tolerated'):
        record_episode(tmp_path, frame_numbers=(0, 1, 1 + MAX_TOLERATED_FRAME_GAP + 2))  # skip MAX+1 frames


def test_camera_frame_gap_is_recorded_not_fatal(tmp_path):
    """A dropped device frame number must not discard the whole demonstration."""
    path, _ticks = record_episode(tmp_path, frame_numbers=(0, 1, 4))
    meta = json.loads((path / 'episode.json').read_text())
    assert meta['lifecycle'] == 'complete'
    assert meta['camera_frame_gaps'] == {'external_rgb': 2}
    loaded, raw = load_episode(path)
    assert len(raw['frame:external_rgb']) == 3
