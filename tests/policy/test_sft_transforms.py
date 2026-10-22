"""OpenPI transforms with the project X5 adapter; needs the pi05-sft environment (check-all group policy)."""
import importlib.util
import unittest
import numpy as np
if importlib.util.find_spec('flax') is None:
    raise unittest.SkipTest('needs outputs/envs/pi05-sft (./run.sh check-all --only policy)')
from embodied.policy.pi05_sft.checkpoint import DEFAULT_CHECKPOINT,inspect_checkpoint
from openpi import transforms as t
from openpi.shared.normalize import load
from embodied.policy.openpi.x5 import X5Inputs,X5Outputs
from pathlib import Path


class TransformTests(unittest.TestCase):
    def test_x5_delta_quantile_inverse_round_trip(self):
        stats=load(Path(inspect_checkpoint(DEFAULT_CHECKPOINT)['stats']).parent)
        state=np.array([0,1.5,1.45,-1.52,0,0,.078],dtype=np.float32)
        actions=np.tile(state,(15,1))
        actions[:,:6]+=.02
        actions[:,6]=.04
        original=actions.copy()
        data={'state':state.copy(),'actions':actions}
        mask=t.make_bool_mask(6,-1)
        data=t.DeltaActions(mask)(data)
        np.testing.assert_allclose(data['actions'][:,:6],.02,atol=1e-6)
        np.testing.assert_allclose(data['actions'][:,6],.04)
        data=t.Normalize(stats,use_quantiles=True)(data)
        data=t.PadStatesAndActions(32)(data)
        data=t.Unnormalize(stats,use_quantiles=True)(data)
        data=t.AbsoluteActions(mask)(data)
        data=X5Outputs()(data)
        np.testing.assert_allclose(data['actions'],original,atol=1e-5)

    def test_camera_masks_and_seven_state_tokens_before_padding(self):
        class Tokenizer:
            def tokenize(self,prompt,state):
                self.state=state.copy()
                return np.array([2,3]),np.array([True,True])
        data=X5Inputs()({'wrist_image':np.zeros((32,32,3),np.uint8),'state':np.zeros(7),'prompt':'task'})
        self.assertEqual({k:bool(v) for k,v in data['image_mask'].items()},
            {'base_0_rgb':False,'left_wrist_0_rgb':True,'right_wrist_0_rgb':False})
        tok=Tokenizer()
        data=t.TokenizePrompt(tok,discrete_state_input=True)(data)
        data=t.PadStatesAndActions(32)(data)
        self.assertEqual(tok.state.shape,(7,))
        self.assertEqual(data['state'].shape,(32,))


if __name__=='__main__':unittest.main()
