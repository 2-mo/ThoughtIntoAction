"""Native OpenPI JAX inference with checkpoint-owned normalization and the training X5 adapter."""
import os
os.environ.setdefault('JAX_PLATFORMS', 'cuda')
os.environ.setdefault('XLA_PYTHON_CLIENT_PREALLOCATE', 'false')
import sys
from pathlib import Path
from embodied.policy.pi05_sft.checkpoint import ROOT, OPENPI, inspect_jax_checkpoint
# OpenPI is installed in the selected policy environment by setup.sh/setup_jax.sh.


def load_backend(checkpoint, horizon=15):
    import numpy as np
    import jax
    import jax.numpy as jnp
    from openpi.models.model import restore_params
    from openpi.models.pi0_config import Pi0Config
    from openpi.policies.policy import Policy
    from openpi import transforms as t
    from openpi.shared.normalize import load as load_stats
    from openpi.models.tokenizer import PaligemmaTokenizer
    from embodied.policy.openpi.x5 import X5Inputs, X5Outputs
    spec = inspect_jax_checkpoint(checkpoint, horizon=horizon)
    if not any(d.platform == 'gpu' for d in jax.devices()):
        raise RuntimeError('JAX CUDA device unavailable')
    config = Pi0Config(**spec['model'])
    print(f'加载 JAX PI05 params：{checkpoint}，horizon={horizon}', flush=True)
    model = config.load(restore_params(Path(spec['checkpoint'])/'params', dtype=jnp.bfloat16), remove_extra_params=False)
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
                    sample_kwargs={'num_steps':10})
    return policy, spec, stats
