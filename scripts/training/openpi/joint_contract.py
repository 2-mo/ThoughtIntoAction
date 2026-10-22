"""Train a declared joint-vector/camera contract with the inference adapter."""
import dataclasses

from openpi import transforms
from openpi.models import pi0_config
from openpi.training import config, weight_loaders

from embodied.policy.openpi.adapter import validate_spec, JointInputs, JointOutputs, delta_mask


@dataclasses.dataclass(frozen=True)
class JointDataConfig(config.DataConfigFactory):
    adapter_spec: dict = dataclasses.field(default_factory=dict)

    def create(self, assets_dirs, model_config):
        spec = validate_spec(self.adapter_spec)
        if model_config.action_dim != spec['model_action_dim']:
            raise ValueError('Training model width disagrees with adapter')
        fields = {'state': 'observation.state', 'actions': 'action', 'prompt': 'prompt'}
        for camera in spec['camera_roles'].values():
            fields[camera] = 'observation.images.' + camera[:-4]
            fields[camera + '_valid'] = 'observation.' + camera[:-4] + '_valid'
        mask = delta_mask(spec)
        processing = transforms.Group(inputs=[JointInputs(spec)], outputs=[JointOutputs(spec)])
        processing = processing.push(inputs=[transforms.DeltaActions(mask)], outputs=[transforms.AbsoluteActions(mask)])
        return dataclasses.replace(self.create_base_config(assets_dirs, model_config),
            repack_transforms=transforms.Group(inputs=[transforms.RepackTransform(fields)]),
            data_transforms=processing, model_transforms=config.ModelTransformFactory()(model_config),
            action_sequence_keys=('action',), prompt_from_task=True)


def train_config(spec, repo_id, name, fps=30):
    spec = validate_spec(spec)
    if fps != 30:
        raise ValueError('Training video and actions must use the declared 30 FPS export')
    return config.TrainConfig(name=name,
        model=pi0_config.Pi0Config(pi05=True, action_dim=spec['model_action_dim'], action_horizon=round(fps * .5)),
        data=JointDataConfig(repo_id=repo_id, adapter_spec=spec, base_config=config.DataConfig(prompt_from_task=True)),
        weight_loader=weight_loaders.CheckpointWeightLoader('gs://openpi-assets/checkpoints/pi05_base/params'),
        num_train_steps=30000)
