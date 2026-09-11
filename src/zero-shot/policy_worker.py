"""Persistent PI05 worker: load once, infer from each fresh observation."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time

import cv2
import numpy as np
import sentencepiece
import torch
from infer import load_policy
from tokenizer import DEFAULT_PATH, verify, tokenize
from x5_mapping import X5Mapping


def emit(value):
    print('@pi05 ' + json.dumps(value), flush=True)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--stats', type=Path, required=True)
    parser.add_argument('--model', type=Path, default=Path('/home/r2/work/models/pi05_base'))
    args = parser.parse_args()
    mapping = X5Mapping(args.stats)
    stats_hash = hashlib.sha256(args.stats.read_bytes()).hexdigest()
    torch.set_num_threads(8)
    (torch.ones(1, device='cuda') + 1).cpu()
    verify(DEFAULT_PATH)
    tokenizer = sentencepiece.SentencePieceProcessor(model_file=str(DEFAULT_PATH))
    policy = load_policy(args.model, 'cuda', 'bfloat16', 10)
    if mapping.data['horizon'] != policy.config.chunk_size:
        raise ValueError('Statistics horizon differs from checkpoint chunk_size')
    emit({'ready': True, 'stats_sha256': stats_hash})
    for line in sys.stdin:
        request = json.loads(line)
        output = Path(request['output'])
        obs = json.loads((output / 'observation.json').read_text())
        if obs['robot'] != mapping.data['robot']:
            raise ValueError('Statistics robot metadata differs from observation')
        rgb = cv2.imread(str(output / 'wrist.png'))
        if rgb is None:
            raise ValueError('Cannot read live image')
        rgb = rgb[..., ::-1].copy()
        state = np.asarray(obs['state_physical'])
        normalized, clipped = mapping.normalize_state(state)
        tokens, mask = tokenize(tokenizer, request['prompt'], normalized, policy.config.tokenizer_max_length)
        batch = {'observation.images.left_wrist_0_rgb':
                 torch.from_numpy(rgb.transpose(2, 0, 1).copy()).float().unsqueeze(0).cuda() / 255,
                 'observation.state': torch.from_numpy(normalized).unsqueeze(0).cuda(),
                 'observation.language.tokens': torch.tensor([tokens], device='cuda'),
                 'observation.language.attention_mask': torch.tensor([mask], device='cuda')}
        torch.manual_seed(0)
        torch.cuda.synchronize()
        start = time.perf_counter()
        with torch.inference_mode():
            raw = policy.predict_action_chunk(batch)[0].float().cpu().numpy()
        elapsed = time.perf_counter() - start
        if raw.shape != (mapping.data['horizon'], 32) or not np.isfinite(raw).all():
            raise ValueError('Invalid policy action chunk')
        decoded = mapping.absolute_actions(raw, state)
        np.savez_compressed(output / 'prediction.npz', actions_model_space=raw,
                            actions_x5_experimental=decoded, state_model_input=normalized)
        result = {'model': str(args.model.resolve()), 'stats_sha256': stats_hash,
                  'prompt': request['prompt'], 'inference_seconds': elapsed,
                  'state_clipped_dimensions': clipped, 'state_quantile_excess': mapping.state_excess(state), 'first_action_x5_experimental': decoded[0].tolist(),
                  'rgb_monotonic_ns': obs['rgb_monotonic_ns'], 'state_physical': state.tolist(),
                  'arm_service_id': obs['arm_service_id'], 'seed': 0, 'dtype': 'bfloat16', 'denoise_steps': 10}
        (output / 'prediction.json').write_text(json.dumps(result, indent=2) + '\n')
        emit({'prediction': result, 'request_id': request.get('request_id')})



if __name__ == '__main__':
    try:
        main()
    except Exception as exc:
        emit({'error': str(exc)})
        raise
