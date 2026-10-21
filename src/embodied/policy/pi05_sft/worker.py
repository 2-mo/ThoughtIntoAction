"""Persistent trained OpenPI worker; owns GPU, never connects to the arm."""
import argparse
import hashlib
import json
from pathlib import Path
import sys
import time
from contextlib import nullcontext


def emit(data):
    print('@pi05 '+json.dumps(data),flush=True)


def main():
    import cv2
    import numpy as np
    p=argparse.ArgumentParser()
    p.add_argument('--checkpoint',type=Path,required=True)
    p.add_argument('--stats',type=Path,required=True)
    p.add_argument("--backend",choices=("pytorch","jax"),default="pytorch")
    p.add_argument("--jax-horizon",type=int,default=15)
    args=p.parse_args()
    if args.backend == 'jax':
        from embodied.policy.pi05_sft.backend_jax import load_backend
        policy,spec,stats=load_backend(args.checkpoint,horizon=args.jax_horizon)
        synchronize=lambda: None  # Policy.infer converts outputs to NumPy, synchronizing JAX.
        inference_context=nullcontext
    else:
        import torch
        from embodied.policy.pi05_sft.backend import load_backend
        policy,spec,stats=load_backend(args.checkpoint)
        torch.manual_seed(42)
        synchronize=torch.cuda.synchronize
        inference_context=torch.inference_mode
    digest=hashlib.sha256(args.stats.read_bytes()).hexdigest()
    if digest != hashlib.sha256(Path(spec['stats']).read_bytes()).hexdigest():
        raise ValueError('Selected normalization differs from checkpoint assets')
    if args.backend == 'jax':
        print('JAX 首次编译预热，完成后才连接机械臂…',flush=True)
        policy.infer({'state':np.asarray(stats['state'].mean,dtype=np.float32),
                      'wrist_image':np.zeros((480,640,3),dtype=np.uint8),
                      'wrist_valid':True,'prompt':'Pick up the blue cube.'})
    emit({'ready':True,'stats_sha256':digest,'checkpoint':spec})
    for line in sys.stdin:
        request=json.loads(line)
        output=Path(request['output'])
        obs=json.loads((output/'observation.json').read_text())
        state=np.asarray(obs['state_physical'],dtype=np.float32)
        image=cv2.imread(str(output/'wrist.png'))
        if image is None or state.shape!=(7,) or not np.isfinite(state).all():
            raise ValueError('Invalid live observation')
        if obs['robot']['joint_names'] != ['joint1','joint2','joint3','joint4','joint5','joint6'] or obs['robot']['joint_unit'] != 'rad' or obs['robot']['length_unit'] != 'm':
            raise ValueError('Unexpected X5 joint order or units')
        synchronize()
        began=time.perf_counter()
        with inference_context():
            result=policy.infer({'state':state,'wrist_image':image[...,::-1].copy(),'wrist_valid':True,'prompt':request['prompt']})
        synchronize()
        elapsed=time.perf_counter()-began
        actions=np.asarray(result['actions'])
        if actions.shape!=(spec['model']['action_horizon'],7) or not np.isfinite(actions).all():
            raise ValueError('Invalid SFT action chunk')
        lo,hi=stats['state'].q01,stats['state'].q99
        normalized=2*(state-lo)/(hi-lo+1e-6)-1
        np.savez_compressed(output/'prediction.npz',actions_x5=actions,state_physical=state)
        prediction={'checkpoint':spec,'stats_sha256':digest,'prompt':request['prompt'],
                    'inference_seconds':elapsed,'first_action_x5':actions[0].tolist(),
                    'state_quantile_excess':float(np.maximum(np.abs(normalized)-1,0).max()),
                    'state_quantile_excess_by_dimension':dict(zip(
                        ['joint1','joint2','joint3','joint4','joint5','joint6','gripper_m'],
                        np.maximum(np.abs(normalized)-1,0).tolist())),
                    'rgb_monotonic_ns':obs['rgb_monotonic_ns'],'arm_service_id':obs['arm_service_id'],
                    'state_physical':state.tolist(),'seed':0 if args.backend == 'jax' else 42,'action_shape':list(actions.shape)}
        (output/'prediction.json').write_text(json.dumps(prediction,ensure_ascii=False,indent=2)+'\n')
        emit({'prediction':prediction,'request_id':request.get('request_id')})


if __name__=='__main__':
    try:main()
    except Exception as exc:
        emit({'error':str(exc)})
        raise
