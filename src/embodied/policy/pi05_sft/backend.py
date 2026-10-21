"""Native OpenPI PyTorch inference with checkpoint-owned normalization and the training X5 adapter."""
import os
os.environ.setdefault('JAX_PLATFORMS', 'cpu')
import sys
from pathlib import Path
from embodied.policy.pi05_sft.checkpoint import ROOT, OPENPI, inspect_checkpoint
sys.path[:0] = [str(OPENPI/'src'), str(OPENPI/'packages/openpi-client/src'), str(ROOT/'integrations/openpi')]


def load_backend(checkpoint):
    import numpy as np
    import torch
    from safetensors.torch import load_model
    from openpi.models.pi0_config import Pi0Config
    from openpi.models_pytorch.pi0_pytorch import PI0Pytorch
    from openpi.policies.policy import Policy
    from openpi import transforms as t
    from openpi.shared.normalize import load as load_stats
    from openpi.models.tokenizer import PaligemmaTokenizer
    from x5_policy import X5Inputs, X5Outputs
    spec = inspect_checkpoint(checkpoint)
    torch.set_num_threads(8)
    (torch.ones(1, device='cuda')+1).cpu()
    config = Pi0Config(**spec['model'])
    print(f'构建 OpenPI PI05：step={spec["step"]}, horizon={config.action_horizon}', flush=True)
    model = PI0Pytorch(config)
    print('严格加载微调权重…', flush=True)
    load_model(model, str(Path(checkpoint)/'model.safetensors'), strict=True)
    model.paligemma_with_expert.to_bfloat16_for_selected_params('bfloat16')
    model.eval()
    stats = load_stats(Path(spec['stats']).parent)
    for key in ('state','actions'):
        lo, hi = stats[key].q01, stats[key].q99
        if not np.isfinite([lo,hi]).all() or np.any(hi < lo):
            raise ValueError('Invalid checkpoint quantiles')
    # Use the same public SentencePiece file as OpenPI; keep inference fully local.
    import sentencepiece
    token_path = ROOT/'outputs/zero-shot-assets/paligemma_tokenizer.model'
    import hashlib
    if hashlib.sha256(token_path.read_bytes()).hexdigest() != '8986bb4f423f07f8c7f70d0dbe3526fb2316056c17bae71b1ea975e77a168fc6':
        raise ValueError('PaliGemma tokenizer checksum mismatch')
    tokenizer = object.__new__(PaligemmaTokenizer)
    tokenizer._max_len = config.max_token_len
    tokenizer._tokenizer = sentencepiece.SentencePieceProcessor(model_file=str(token_path))
    mask = t.make_bool_mask(6, -1)
    inputs = [X5Inputs(), t.DeltaActions(mask), t.Normalize(stats,use_quantiles=True),
              t.ResizeImages(224,224), t.TokenizePrompt(tokenizer,discrete_state_input=True),
              t.PadStatesAndActions(config.action_dim)]
    outputs = [t.Unnormalize(stats,use_quantiles=True), t.AbsoluteActions(mask), X5Outputs()]
    policy = Policy(model, transforms=inputs, output_transforms=outputs,
                    sample_kwargs={'num_steps':10}, is_pytorch=True, pytorch_device='cuda')
    return policy, spec, stats
