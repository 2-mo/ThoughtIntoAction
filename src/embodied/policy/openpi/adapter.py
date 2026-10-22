"""Explicit robot/camera semantics shared by OpenPI training and inference.

OpenPI pads logical vectors to model_action_dim. Only that declared padding is
removed on output; an unexpected vector width or joint order is an error.
"""
from copy import deepcopy
from dataclasses import dataclass
import hashlib
import json

import numpy as np

from embodied.data.episode.contract import validate_contract, feature_names, action_definition

SLOTS = ('base_0_rgb', 'left_wrist_0_rgb', 'right_wrist_0_rgb')


def adapter_spec(contract, camera_roles, *, model_action_dim=32):
    contract = validate_contract(contract)
    dimension = len(feature_names(contract))
    if type(model_action_dim) is not int or not dimension <= model_action_dim:
        raise ValueError('Robot dimension exceeds the declared model action width')
    if (not isinstance(camera_roles, dict) or not camera_roles or
            any(role not in SLOTS or not isinstance(camera, str) or not camera.endswith('_rgb')
                for role, camera in camera_roles.items()) or len(set(camera_roles.values())) != len(camera_roles)):
        raise ValueError('Declare unique camera IDs for supported OpenPI image roles')
    return {'schema': 'embodied.openpi_adapter/1', 'data_contract': contract,
            'camera_roles': dict(camera_roles), 'model_action_dim': model_action_dim,
            'action_indices': list(range(dimension)),
            'delta_joints': True, 'gripper_absolute': True}


def validate_spec(spec):
    if not isinstance(spec, dict) or spec.get('schema') != 'embodied.openpi_adapter/1':
        raise ValueError('Expected a versioned OpenPI adapter descriptor')
    expected = adapter_spec(spec['data_contract'], spec['camera_roles'], model_action_dim=spec['model_action_dim'])
    if spec != expected:
        raise ValueError('Unsupported action mapping; use declared joint order followed by optional absolute gripper')
    return deepcopy(expected)


def asset_id(spec):
    digest = hashlib.sha256(json.dumps(validate_spec(spec), sort_keys=True, separators=(',', ':')).encode()).hexdigest()[:16]
    return 'local/joint_contract_' + digest


def from_dataset(layout, info):
    """Validate names as well as dimensions before training/norm-stat computation."""
    spec = adapter_spec(layout['data_contract'], layout.get('camera_roles'))
    if layout.get('action_definition') != action_definition(spec['data_contract']):
        raise ValueError('Training action definition disagrees with the vector contract')
    names = feature_names(spec['data_contract'])
    features = info.get('features', {})
    for key in ('action', 'observation.state'):
        feature = features.get(key, {})
        if tuple(feature.get('shape', ())) != (len(names),) or feature.get('names') != names:
            raise ValueError(f'{key}: dimensions and ordered feature names must match data_contract')
    for camera in spec['camera_roles'].values():
        shape = features.get('observation.images.' + camera[:-4], {}).get('shape', ())
        if len(shape) != 3 or shape[-1] != 3 or any(type(n) is not int or n <= 0 for n in shape):
            raise ValueError(f'{camera}: missing RGB training feature')
        if 'observation.' + camera[:-4] + '_valid' not in features:
            raise ValueError(f'{camera}: missing validity feature')
    return spec


def delta_mask(spec):
    contract = validate_spec(spec)['data_contract']
    return np.asarray([True] * len(contract['joint_names']) + ([False] if contract['gripper'] else []))


def _vector(value, dimension, *, state=False):
    value = np.asarray(value, dtype=np.float32)
    valid_shape = value.shape == (dimension,) if state else value.ndim >= 1 and value.shape[-1] == dimension
    if not valid_shape or not np.isfinite(value).all():
        raise ValueError('State/action vector does not match the declared dimensions and finite values')
    return value


def _image(value):
    value = np.asarray(value)
    if value.ndim != 3:
        raise ValueError('Expected an RGB image')
    if value.shape[-1] != 3 and value.shape[0] == 3:
        value = value.transpose(1, 2, 0)
    if value.shape[-1] != 3:
        raise ValueError('Expected three RGB channels')
    if np.issubdtype(value.dtype, np.floating):
        if not np.isfinite(value).all() or np.any(value < 0) or np.any(value > 1):
            raise ValueError('Float RGB values must be finite and normalized to [0, 1]')
        value = (value * 255).astype(np.uint8)
    if value.dtype != np.uint8:
        raise ValueError('RGB must be uint8 or normalized float')
    return value


@dataclass(frozen=True)
class JointInputs:
    spec: dict

    def __call__(self, data):
        spec = validate_spec(self.spec)
        dimension = len(feature_names(spec['data_contract']))
        images, masks = {}, {}
        for role, camera in spec['camera_roles'].items():
            images[role] = _image(data[camera])  # A declared view must exist, even when invalid.
            valid = np.asarray(data.get(camera + '_valid', True))
            if valid.size != 1 or valid.dtype != np.bool_:
                raise ValueError('Camera validity must be a single boolean')
            masks[role] = np.bool_(valid.reshape(-1)[0])
        reference = next(iter(images.values()))
        for role in SLOTS:
            if role not in images:
                images[role], masks[role] = np.zeros_like(reference), np.False_
        result = {'state': _vector(data['state'], dimension, state=True), 'image': images, 'image_mask': masks}
        if 'actions' in data:
            result['actions'] = _vector(data['actions'], dimension)
        if 'prompt' in data:
            result['prompt'] = data['prompt']
        return result


@dataclass(frozen=True)
class JointOutputs:
    spec: dict

    def __call__(self, data):
        spec = validate_spec(self.spec)
        values = np.asarray(data['actions'])
        if values.ndim < 1 or values.shape[-1] != spec['model_action_dim'] or not np.isfinite(values).all():
            raise ValueError('Model output width/values disagree with declared model padding')
        return {'actions': np.take(values, spec['action_indices'], axis=-1)}


def transforms(spec):
    """Device-free input/output transforms and delta mask; no OpenPI import needed."""
    return JointInputs(validate_spec(spec)), JointOutputs(validate_spec(spec)), delta_mask(spec)
