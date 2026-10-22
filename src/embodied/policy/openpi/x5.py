"""Named six-joint + gripper preset, backed by the shared contract adapter."""
from dataclasses import dataclass

from embodied.policy.openpi.adapter import adapter_spec, JointInputs, JointOutputs


def x5_spec(*, exterior=False, model_action_dim=32):
    roles = {'left_wrist_0_rgb': 'wrist_rgb'}
    if exterior:
        roles['base_0_rgb'] = 'exterior_rgb'
    return adapter_spec({'schema': 'embodied.joint_vectors/1', 'representation': 'absolute_joint_position',
        'joint_names': [f'joint{i + 1}' for i in range(6)], 'joint_unit': 'rad',
        'gripper': {'name': 'gripper_m', 'unit': 'm', 'range': [0., .082]}, 'robot_type': 'stanford_x5'},
        roles, model_action_dim=model_action_dim)


@dataclass(frozen=True)
class X5Inputs:
    def __call__(self, data):
        values = dict(data, wrist_rgb=data['wrist_image'], wrist_rgb_valid=data.get('wrist_valid', True))
        exterior = 'exterior_image' in data
        if exterior:
            values.update(exterior_rgb=data['exterior_image'], exterior_rgb_valid=data.get('exterior_valid', True))
        return JointInputs(x5_spec(exterior=exterior))(values)


@dataclass(frozen=True)
class X5Outputs:
    model_action_dim: int = 32

    def __call__(self, data):
        return JointOutputs(x5_spec(model_action_dim=self.model_action_dim))(data)
