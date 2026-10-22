"""Contract binding tests without OpenPI, a model, GPU, or LeRobot dependencies."""
import copy
import importlib.util
import json
from pathlib import Path

import numpy as np
import pytest

from embodied.data.episode.contract import feature_names, action_definition
from embodied.data.export.lerobot import export_features
from embodied.policy.openpi.adapter import adapter_spec, from_dataset, asset_id, transforms
from embodied.policy.pi05_sft.checkpoint import inspect_jax_checkpoint, checkpoint_adapter
from test_task_recording import CONTRACT, ROLES


@pytest.mark.parametrize('gripper', [None, {'name': 'width', 'unit': 'm', 'range': [0., .12]}])
def test_exact_seven_axis_training_inference_mapping(gripper):
    contract = {**CONTRACT, 'gripper': gripper}
    spec = adapter_spec(contract, ROLES)
    inputs, outputs, mask = transforms(spec)
    dimension = 7 + bool(gripper)
    values = np.arange(dimension, dtype=np.float32)
    result = inputs({'state': values, 'actions': np.stack([values, values]),
                     'front_rgb': np.ones((3, 8, 9), np.float32), 'front_rgb_valid': True, 'prompt': 'reach'})
    assert result['state'].tolist() == values.tolist()
    assert result['image']['base_0_rgb'].shape == (8, 9, 3)
    assert result['image_mask'] == {'base_0_rgb': True, 'left_wrist_0_rgb': False, 'right_wrist_0_rgb': False}
    assert mask.tolist() == [True] * 7 + ([False] if gripper else [])
    model = np.arange(32, dtype=np.float32)[None, :]
    assert outputs({'actions': model})['actions'].tolist() == [values.tolist()]
    with pytest.raises(ValueError, match='width'):
        outputs({'actions': np.zeros((2, 31))})
    with pytest.raises(ValueError, match='dimensions'):
        inputs({'state': values[:-1], 'front_rgb': np.zeros((8, 9, 3), np.uint8)})
    with pytest.raises(KeyError, match='front_rgb'):
        inputs({'state': values})


def test_training_entry_checks_joint_order_and_roles_before_selecting_stats():
    layout = {'layout': 'aligned30_runtime_v2', 'data_contract': CONTRACT, 'camera_roles': ROLES,
              'action_definition': action_definition(CONTRACT)}
    info = {'fps': 30, 'features': export_features(CONTRACT, {'front_rgb': {'height': 8, 'width': 9}})}
    spec = from_dataset(layout, info)
    module = importlib.util.spec_from_file_location('training_entry', Path(__file__).resolve().parents[1] / 'scripts/training/openpi/run.py')
    entry = importlib.util.module_from_spec(module)
    module.loader.exec_module(entry)
    assert entry.dataset_config(layout, info)[0] == asset_id(spec)
    different = copy.deepcopy(spec)
    different['data_contract']['joint_names'].reverse()
    assert asset_id(different) != asset_id(spec)
    info['features']['action']['names'] = list(reversed(feature_names(CONTRACT)))
    with pytest.raises(ValueError, match='ordered feature names'):
        from_dataset(layout, info)
    with pytest.raises(ValueError, match='camera IDs'):
        adapter_spec(CONTRACT, {'base_0_rgb': 'front_rgb', 'left_wrist_0_rgb': 'front_rgb'})


def test_seven_dimensional_jax_checkpoint_cannot_be_misidentified_as_x5(tmp_path):
    spec = adapter_spec(CONTRACT, ROLES)
    root = tmp_path / '12'
    (root / 'params').mkdir(parents=True)
    (root / 'params/_METADATA').write_text('{}')
    assets = root / 'assets' / asset_id(spec)
    assets.mkdir(parents=True)
    (assets / 'norm_stats.json').write_text(json.dumps({'norm_stats': {
        key: {field: [0.] * 7 for field in ('mean', 'std', 'q01', 'q99')} for key in ('state', 'actions')}}))
    with pytest.raises(ValueError, match='explicit adapter_spec'):
        inspect_jax_checkpoint(root)
    actual = inspect_jax_checkpoint(root, adapter_spec=spec)
    assert actual['cameras'] == ['front_rgb']
    assert actual['adapter_spec']['data_contract']['gripper'] is None
    wrong = copy.deepcopy(spec)
    wrong['data_contract']['joint_names'].reverse()
    with pytest.raises(ValueError, match='normalization asset identity'):
        inspect_jax_checkpoint(root, adapter_spec=wrong)
    with pytest.raises(ValueError, match='training metadata'):
        checkpoint_adapter(spec, wrong, asset_id(spec))


def test_named_x5_preset_rejects_unexpected_output_dimension():
    from embodied.policy.openpi.x5 import X5Outputs
    with pytest.raises(ValueError, match='width'):
        X5Outputs()({'actions': np.zeros((3, 33))})
