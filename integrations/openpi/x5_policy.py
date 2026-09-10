"""Device-free X5 observation/action adaptation, shared by train and inference."""
import dataclasses
import numpy as np

@dataclasses.dataclass(frozen=True)
class X5Inputs:
    def __call__(self, data):
        wrist = np.asarray(data['wrist_image'])
        if wrist.ndim != 3:
            raise ValueError('Expected a wrist RGB image')
        if wrist.shape[0] == 3:
            wrist = wrist.transpose(1, 2, 0)
        if np.issubdtype(wrist.dtype, np.floating):
            wrist = (np.clip(wrist, 0, 1) * 255).astype(np.uint8)
        result = {'state': np.asarray(data['state'], dtype=np.float32),
                  'image': {'base_0_rgb': np.zeros_like(wrist), 'left_wrist_0_rgb': wrist,
                            'right_wrist_0_rgb': np.zeros_like(wrist)},
                  'image_mask': {'base_0_rgb': np.False_, 'left_wrist_0_rgb': np.bool_(np.asarray(data.get('wrist_valid', True)).reshape(-1)[0]), 'right_wrist_0_rgb': np.False_}}
        if result['state'].shape != (7,): raise ValueError('X5 state must have 7 dimensions')
        if 'actions' in data: result['actions'] = np.asarray(data['actions'], dtype=np.float32)
        if 'prompt' in data: result['prompt'] = data['prompt']
        return result


@dataclasses.dataclass(frozen=True)
class X5Outputs:
    def __call__(self, data):
        return {'actions': np.asarray(data['actions'])[..., :7]}
