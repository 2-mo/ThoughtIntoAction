"""Offline PI05 base probe. Deliberately has no robot command interface."""
from __future__ import annotations

import argparse
import gzip
import json
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[4]
sys.path.insert(0, str(ROOT / 'src'))


def rows(directory, name):
    path = directory / f'{name}.jsonl'
    if path.exists():
        stream = path.open()
    else:
        stream = gzip.open(str(path) + '.gz', 'rt')
    with stream:
        return [json.loads(line) for line in stream if line.strip()]


def episode_observations(directory, indices):
    import numpy as np
    from embodied.data.episode.video import read_frame

    meta = json.loads((directory / 'episode.json').read_text())
    if meta.get('lifecycle') != 'complete':
        raise ValueError('Choose a completed episode')
    states, frames = rows(directory, 'states'), rows(directory, 'frames')
    times = np.array([s['host_monotonic_ns'] for s in states], dtype=np.int64)
    if len(times) == 0 or np.any(np.diff(times) <= 0):
        raise ValueError('State timestamps must be nonempty and strictly increasing')
    observations = []
    for index in indices:
        if not 0 <= index < len(frames):
            raise ValueError(f'Frame {index} out of range [0, {len(frames)})')
        frame = frames[index]
        stamp = frame['host_received_monotonic_ns']
        selected = int(np.searchsorted(times, stamp, side='right')) - 1
        if selected < 0 or stamp - int(times[selected]) > 100_000_000:
            raise ValueError(f'Frame {index} has no preceding state within 100 ms')
        state = states[selected]
        bgr = read_frame(directory, frame)
        if bgr is None:
            raise ValueError(f'Cannot decode frame {index}')
        observations.append((bgr[..., ::-1].copy(),
            np.array([*state['actual_pos'], state['gripper_actual_pos']], dtype=np.float32),
            {'episode': str(directory), 'frame_index': index,
             'rgb_monotonic_ns': stamp, 'state_monotonic_ns': int(times[selected])}))
    return observations, float(meta['control']['gripper_width_m'])


def prepare_state(state, mode, gripper_width):
    import numpy as np

    state = np.asarray(state, dtype=np.float32)
    if state.shape != (7,) or not np.isfinite(state).all():
        raise ValueError('State must contain 6 finite joint angles (rad) and gripper width (m)')
    if not np.isfinite(gripper_width) or gripper_width <= 0:
        raise ValueError('Gripper width must be positive')
    if mode == 'joint-limits':
        robot = ET.parse(ROOT / 'assets/robots/x5/X5_2025_hybrid.urdf').getroot()
        limits = [robot.find(f"joint[@name='joint{i}']/limit") for i in range(1, 7)]
        lower = np.array([float(x.attrib['lower']) for x in limits] + [0])
        upper = np.array([float(x.attrib['upper']) for x in limits] + [gripper_width])
        state = 2 * (state - lower) / (upper - lower) - 1
    outside = np.flatnonzero((state < -1) | (state > 1)).tolist()
    # Match the 32 state tokens expected by this base checkpoint, before tokenization.
    return np.pad(np.clip(state, -1, 1), (0, 25)).astype(np.float32), outside


def load_policy(model_path, device, dtype, steps):
    from lerobot.policies.pi05.configuration_pi05 import PI05Config
    from lerobot.policies.pi05.modeling_pi05 import PI05Policy
    from safetensors.torch import load_file

    config = PI05Config.from_pretrained(str(model_path))
    config.device, config.dtype = device, dtype
    config.num_inference_steps = steps
    config.compile_model = False
    print("Initializing PI05 architecture...", flush=True)
    policy = PI05Policy(config)
    # Upstream from_pretrained catches load errors and can return random weights.
    # Use its key migration, but require strict loading and propagate every failure.
    print("Loading local weights (strict)...", flush=True)
    weights = load_file(str(model_path / 'model.safetensors'))
    weights = policy._fix_pytorch_state_dict_keys(weights, config)
    weights = {k if k.startswith('model.') else f'model.{k}': v for k, v in weights.items()}
    policy.load_state_dict(weights, strict=True)
    del weights
    print(f"Weights verified; moving policy to {device}...", flush=True)
    return policy.to(device).eval()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--model', type=Path, default=Path('/home/r2/work/models/pi05_base'))
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument('--episode', type=Path, help='Raw episode directory (.jsonl or .jsonl.gz)')
    source.add_argument('--image', type=Path, help='Wrist image file')
    parser.add_argument('--state', type=float, nargs=7, help='For --image: q1..q6 radians, gripper metres')
    parser.add_argument('--frames', type=int, nargs='+', default=[0], help='Zero-based raw frame row indices')
    parser.add_argument('--prompt', required=True, help='Task instruction, preferably English')
    parser.add_argument('--state-mode', choices=['joint-limits', 'raw'], default='joint-limits',
                        help='Heuristic physical-range scaling, or raw values clipped to [-1,1]')
    parser.add_argument('--gripper-width', type=float, default=.082, help='Maximum width for --image')
    parser.add_argument('--device', choices=['cuda', 'cpu'], default='cuda')
    parser.add_argument('--dtype', choices=['bfloat16', 'float32'], default='bfloat16')
    parser.add_argument('--steps', type=int, default=10)
    parser.add_argument('--seed', type=int, default=0)
    parser.add_argument('--tokenizer', type=Path, help='Verified public PaliGemma .model file')
    parser.add_argument('--output', type=Path, help='New directory; existing paths are rejected')
    args = parser.parse_args()
    if not args.prompt.strip() or args.steps < 1:
        parser.error('Provide a nonempty prompt and positive --steps')
    if args.image and args.state is None:
        parser.error('--image requires --state q1 q2 q3 q4 q5 q6 gripper_m')
    if args.episode and args.state is not None:
        parser.error('--state is only valid with --image')

    import cv2
    import numpy as np
    import torch
    import sentencepiece
    from tokenizer import DEFAULT_PATH, verify, tokenize

    torch.set_num_threads(8)
    if args.device == 'cuda' and not torch.cuda.is_available():
        raise RuntimeError('CUDA is unavailable; install CUDA PyTorch or explicitly select --device cpu')
    if args.device == 'cuda':
        # Fail before loading 14 GB when the wheel lacks this GPU architecture.
        (torch.ones(1, device='cuda') + 1).cpu()
    if args.episode:
        observations, width = episode_observations(args.episode.resolve(), args.frames)
    else:
        bgr = cv2.imread(str(args.image))
        if bgr is None:
            raise ValueError(f'Cannot read {args.image}')
        observations, width = [(bgr[..., ::-1].copy(), np.array(args.state),
                               {'image': str(args.image.resolve())})], args.gripper_width
    prepared = [prepare_state(state, args.state_mode, width) for _, state, _ in observations]
    output = args.output or ROOT / 'outputs/zero-shot' / time.strftime('%Y%m%d-%H%M%S')
    tokenizer_path = args.tokenizer or DEFAULT_PATH
    verify(tokenizer_path)
    tokenizer = sentencepiece.SentencePieceProcessor(model_file=str(tokenizer_path))
    output.mkdir(parents=True, exist_ok=False)
    print('Offline probe: heuristic X5 state mapping; outputs have no verified X5 units.', flush=True)
    started = time.perf_counter()
    policy = load_policy(args.model.resolve(), args.device, args.dtype, args.steps)
    report = {'model': str(args.model.resolve()), 'prompt': args.prompt, 'seed': args.seed,
              'device': args.device, 'dtype': args.dtype, 'inference_steps': args.steps,
              'torch_version': torch.__version__, 'cuda_version': torch.version.cuda,
              'gpu': torch.cuda.get_device_name() if args.device == 'cuda' else None,
              'lerobot_revision': 'b6ec0060779550c0a157ae34feb89e0cf86012a8',
              'tokenizer': str(tokenizer_path.resolve()),
              'state_mode': args.state_mode, 'gripper_width_m': width,
              'load_seconds': time.perf_counter() - started,
              'interpretation': 'Raw model-space actions, not X5 commands. No task success measured. '
                                'Joint-limit scaling is a heuristic, not pretrained normalization.',
              'samples': []}
    if args.device == 'cuda':
        torch.cuda.reset_peak_memory_stats()
    for number, ((rgb, state, provenance), (normalized, clipped)) in enumerate(zip(observations, prepared)):
        # Omit absent camera keys: PI05 creates explicitly masked placeholders.
        tokens, mask = tokenize(tokenizer, args.prompt, normalized, policy.config.tokenizer_max_length)
        batch = {'observation.images.left_wrist_0_rgb':
                 torch.from_numpy(rgb.transpose(2, 0, 1).copy()).float().unsqueeze(0).to(args.device) / 255,
                 'observation.state': torch.from_numpy(normalized).unsqueeze(0).to(args.device),
                 'observation.language.tokens': torch.tensor([tokens], device=args.device),
                 'observation.language.attention_mask': torch.tensor([mask], device=args.device)}
        torch.manual_seed(args.seed)  # Same noise permits controlled prompt/frame comparisons.
        policy.reset()
        if args.device == 'cuda':
            torch.cuda.synchronize()
        started = time.perf_counter()
        with torch.inference_mode():
            action = policy.predict_action_chunk(batch)
        if args.device == 'cuda':
            torch.cuda.synchronize()
        seconds = time.perf_counter() - started
        action = action.float().cpu().numpy()[0]
        if action.shape != (policy.config.chunk_size, 32) or not np.isfinite(action).all():
            raise ValueError(f'Invalid model action chunk: {action.shape}')
        np.savez_compressed(output / f'sample_{number:03d}.npz', actions_model_space=action,
                            state_physical=state, state_model_input=normalized)
        if not cv2.imwrite(str(output / f'sample_{number:03d}.png'), rgb[..., ::-1]):
            raise RuntimeError('Failed to save input image')
        result = {**provenance, 'sample': number, 'inference_seconds': seconds,
                  'shape': list(action.shape), 'clipped_state_dimensions': clipped,
                  'min': float(action.min()), 'max': float(action.max()),
                  'first_action_first7_unverified': action[0, :7].tolist()}
        report['samples'].append(result)
        print(json.dumps(result, ensure_ascii=False), flush=True)
    if args.device == 'cuda':
        report['peak_gpu_allocated_gib'] = torch.cuda.max_memory_allocated() / 2**30
    (output / 'report.json').write_text(json.dumps(report, ensure_ascii=False, indent=2) + '\n')
    print(f'Results: {output.resolve()}', flush=True)


if __name__ == '__main__':
    main()
