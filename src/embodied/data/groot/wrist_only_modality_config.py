"""Isaac-GR00T modality registration for ARX5 wrist-only training."""

from gr00t.configs.data.embodiment_configs import register_modality_config
from gr00t.data.embodiment_tags import EmbodimentTag
from gr00t.data.types import (
    ActionConfig,
    ActionFormat,
    ActionRepresentation,
    ActionType,
    ModalityConfig,
)


arx5_wrist_only_config = {
    "video": ModalityConfig(
        delta_indices=[0],
        modality_keys=["wrist_rgb"],
    ),
    "state": ModalityConfig(
        delta_indices=[0],
        modality_keys=["eef_9d", "gripper_position", "joint_position"],
    ),
    "action": ModalityConfig(
        delta_indices=list(range(16)),
        modality_keys=["eef_9d", "gripper_position", "joint_position"],
        action_configs=[
            ActionConfig(
                rep=ActionRepresentation.RELATIVE,
                type=ActionType.EEF,
                format=ActionFormat.XYZ_ROT6D,
                state_key="eef_9d",
            ),
            ActionConfig(
                rep=ActionRepresentation.ABSOLUTE,
                type=ActionType.NON_EEF,
                format=ActionFormat.DEFAULT,
                state_key="gripper_position",
            ),
            ActionConfig(
                rep=ActionRepresentation.RELATIVE,
                type=ActionType.NON_EEF,
                format=ActionFormat.DEFAULT,
                state_key="joint_position",
            ),
        ],
    ),
    "language": ModalityConfig(
        delta_indices=[0],
        modality_keys=["annotation.human.task_description"],
    ),
}

register_modality_config(
    arx5_wrist_only_config,
    embodiment_tag=EmbodimentTag.NEW_EMBODIMENT,
)
