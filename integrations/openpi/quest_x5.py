"""OpenPI pi05 adapter for single-arm X5, six joints + gripper metres.
Copy to openpi/src/openpi/policies/quest_x5.py and register train_config().
"""
import dataclasses
import numpy as np
from openpi import transforms
from openpi.models import pi0_config
from openpi.training import config, weight_loaders


from x5_policy import X5Inputs, X5Outputs


@dataclasses.dataclass(frozen=True)
class X5DataConfig(config.DataConfigFactory):
    def create(self, assets_dirs, model_config):
        repack = transforms.Group(inputs=[transforms.RepackTransform({
            'state': 'observation.state', 'wrist_image': 'observation.images.wrist',
            'actions': 'action', 'prompt': 'prompt', 'wrist_valid': 'observation.wrist_valid'})])
        # Delta joints relative to the first observed state of the chunk;
        # gripper remains absolute metres. Outputs restore absolute joint targets.
        mask = transforms.make_bool_mask(6, -1)
        processing = transforms.Group(inputs=[X5Inputs()], outputs=[X5Outputs()])
        processing = processing.push(inputs=[transforms.DeltaActions(mask)],
                                     outputs=[transforms.AbsoluteActions(mask)])
        return dataclasses.replace(self.create_base_config(assets_dirs, model_config),
            repack_transforms=repack, data_transforms=processing,
            model_transforms=config.ModelTransformFactory()(model_config),
            action_sequence_keys=('action',), prompt_from_task=True)


def train_config(repo_id='local/quest_x5', fps=240):
    return config.TrainConfig(name='pi05_quest_x5',
        model=pi0_config.Pi0Config(pi05=True, action_horizon=round(fps * .5)),
        data=X5DataConfig(repo_id=repo_id, base_config=config.DataConfig(prompt_from_task=True)),
        weight_loader=weight_loaders.CheckpointWeightLoader('gs://openpi-assets/checkpoints/pi05_base/params'),
        num_train_steps=30000)
