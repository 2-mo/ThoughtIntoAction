"""OpenPI policy on the arx kernel (todo P-06).

Uses the existing policy workers and observation files; actions go to the kernel through one
ControlSession. The kernel applies joint limits, the table plane and its per-tick step limit to
every gateway source, so this loop only paces action chunks and refuses stale or mismatched
observations. Between chunks the streamed target expires and the kernel holds position.
"""
import argparse
import json
from pathlib import Path
import signal
import time

import numpy as np

from embodied.contracts.runtime.control import ControlSession

POLICY_FPS = 30
JOINTS = {'joint_names': ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6'], 'joint_unit': 'rad', 'length_unit': 'm'}


def observe(session, camera, cycle, max_skew_s=.06):
    """One wrist image and the kernel state closest in time, in the worker's file format."""
    import cv2
    before = session.status()
    frame = camera.read()
    after = session.status()
    stamp = frame['host_received_monotonic_ns']
    state = min((before, after), key=lambda s: abs(s['feedback_monotonic_ns'] - stamp))
    skew = (state['feedback_monotonic_ns'] - stamp) / 1e9
    if abs(skew) > max_skew_s:
        raise ValueError(f'Camera and kernel state are {skew * 1e3:.0f} ms apart')
    if not cv2.imwrite(str(cycle / 'wrist.png'), frame['image']):
        raise OSError('Cannot save the observation image')
    observation = {'robot': JOINTS, 'camera': camera.metadata, 'arm_service_id': session.identity['runtime_id'],
                   'rgb_monotonic_ns': stamp, 'state_monotonic_ns': state['feedback_monotonic_ns'],
                   'state_rgb_skew_s': skew, 'state_physical': [*state['q'], state['gripper_width']]}
    (cycle / 'observation.json').write_text(json.dumps(observation, indent=2) + '\n')
    return observation


def run(session, worker, camera, output, prompt, *, seconds, chunk_steps, max_age_s=1.5):
    """Observe, predict, stream `chunk_steps` actions at POLICY_FPS; repeat until `seconds` elapse."""
    summary = {'status': 'running', 'prompt': prompt, 'chunk_steps': chunk_steps, 'policy_fps': POLICY_FPS,
               'kernel_runtime_id': session.identity['runtime_id'], 'cycles': 0, 'commands': 0}
    deadline = time.monotonic() + seconds
    try:
        with (output / 'commands.jsonl').open('x') as log:
            while time.monotonic() < deadline:
                cycle = output / f'cycle_{summary["cycles"]:04d}'
                cycle.mkdir()
                observation = observe(session, camera, cycle)
                prediction = worker.predict(cycle, prompt)
                if (prediction['arm_service_id'] != observation['arm_service_id']
                        or prediction['rgb_monotonic_ns'] != observation['rgb_monotonic_ns']):
                    raise ValueError('Prediction belongs to another observation')
                with np.load(cycle / 'prediction.npz', allow_pickle=False) as data:
                    actions = data['actions_x5'].copy()
                if actions.ndim != 2 or actions.shape[1] != 7 or len(actions) < chunk_steps or not np.isfinite(actions).all():
                    raise ValueError('Invalid action chunk')
                summary['cycles'] += 1
                start = time.monotonic()
                for index, action in enumerate(actions[:chunk_steps]):
                    if (time.monotonic_ns() - observation['rgb_monotonic_ns']) / 1e9 > max_age_s:
                        raise ValueError('Action chunk observation is stale')
                    session.command('joint', q=[float(v) for v in action[:6]], gripper_width=float(action[6]),
                                    ttl_ms=120, sent_monotonic_ns=time.monotonic_ns())
                    summary['commands'] += 1
                    log.write(json.dumps({'host_monotonic_ns': time.monotonic_ns(), 'cycle': summary['cycles'] - 1,
                                          'action_index': index, 'action': action.tolist()}) + '\n')
                    remaining = start + (index + 1) / POLICY_FPS - time.monotonic()
                    if remaining > 0:
                        time.sleep(remaining)
                    if time.monotonic() >= deadline:
                        break
        summary['status'] = 'time_limit'
    except KeyboardInterrupt:
        summary['status'] = 'interrupted'
    except Exception as exc:
        summary.update(status='failed', reason=f'{type(exc).__name__}: {exc}')
        raise
    finally:
        try:
            session.command('hold')
        except Exception as exc:
            summary['hold_error'] = str(exc)
        (output / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2) + '\n')
    return summary


def load_flat(path):
    """Import a module from the flat policy directories without changing sys.path."""
    import importlib.util
    spec = importlib.util.spec_from_file_location(path.stem, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def interrupt(signum, frame):
    raise KeyboardInterrupt


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split('\n')[0])
    parser.add_argument('--run', type=Path, required=True, help='Running kernel directory control/run/<RUN>')
    parser.add_argument('--checkpoint', type=Path, help='Fine-tuned checkpoint (default: the SFT runner default)')
    parser.add_argument('--backend', choices=('pytorch', 'jax'), default='pytorch')
    parser.add_argument('--jax-horizon', type=int, default=15)
    parser.add_argument('--prompt', required=True)
    parser.add_argument('--seconds', type=float, default=60)
    parser.add_argument('--chunk-steps', type=int, default=10)
    parser.add_argument('--output', type=Path)
    args = parser.parse_args(argv)
    import os
    sft = Path(__file__).resolve().parent / 'pi05_sft'
    checkpoint_module = load_flat(sft / 'checkpoint.py')  # flat worker modules (todo P-07)
    PolicyWorker = load_flat(sft.parent / 'pi05_zeroshot' / 'worker_client.py').PolicyWorker
    ROOT = checkpoint_module.ROOT
    from embodied.core.project import load_config
    from embodied.sensors.client import SharedCamera
    checkpoint = (args.checkpoint or (checkpoint_module.DEFAULT_JAX_CHECKPOINT if args.backend == 'jax'
                                      else checkpoint_module.DEFAULT_CHECKPOINT)).expanduser()
    spec = (checkpoint_module.inspect_jax_checkpoint(checkpoint, horizon=args.jax_horizon) if args.backend == 'jax'
            else checkpoint_module.inspect_checkpoint(checkpoint))
    if not 1 <= args.chunk_steps <= spec['model']['action_horizon'] or not 0 < args.seconds <= 300:
        parser.error('chunk-steps must be within the model horizon and seconds within 0..300')
    if spec['cameras'] not in (None, ['wrist_rgb']):
        print(f'警告：模型训练相机为 {spec["cameras"]}，推理只打开腕部相机。', flush=True)
    output = args.output or ROOT / 'outputs/policy_kernel' / time.strftime('%Y%m%d-%H%M%S')
    output.mkdir(parents=True, exist_ok=False)
    (output / 'norm_stats.json').write_bytes(Path(spec['stats']).read_bytes())
    python = Path(os.environ.get('PI05_JAX_PYTHON' if args.backend == 'jax' else 'PI05_SFT_PYTHON',
                                 str(ROOT / 'outputs/envs' / ('pi05-jax' if args.backend == 'jax' else 'pi05-sft') / 'bin/python')))
    worker = PolicyWorker(output / 'norm_stats.json', python=python, script=sft / 'worker.py',
                          extra_args=('--checkpoint', str(Path(spec['checkpoint']).resolve()), '--backend', args.backend,
                                      '--jax-horizon', str(args.jax_horizon)),
                          startup_timeout=600 if args.backend == 'jax' else 180)
    signal.signal(signal.SIGTERM, interrupt)
    camera_config = load_config()['cameras']['wrist_rgb']
    try:
        with SharedCamera(camera_config['serial'], camera_config['width'], camera_config['height'], camera_config['fps']) as camera, \
                ControlSession(args.run, source='policy', purpose='operation',
                               metadata={'checkpoint': spec['checkpoint'], 'backend': args.backend}) as session:
            summary = run(session, worker, camera, output, args.prompt, seconds=args.seconds, chunk_steps=args.chunk_steps)
    finally:
        worker.close()
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0 if summary['status'] in ('time_limit', 'interrupted') else 1


if __name__ == '__main__':
    raise SystemExit(main())
