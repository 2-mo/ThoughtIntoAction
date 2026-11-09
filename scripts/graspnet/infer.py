"""Headless official GraspNet baseline inference; run in the remote GPU environment."""
import argparse
import hashlib
import json
from pathlib import Path
import subprocess
import sys
import time
import types

import numpy as np


def cloud(value, name):
    value = np.asarray(value, dtype=np.float32)
    if value.ndim != 2 or value.shape[1] != 3 or len(value) < 100:
        raise ValueError(f'{name} needs at least 100 XYZ points in metres')
    if not np.isfinite(value).all() or np.any(value[:, 2] <= 0):
        raise ValueError(f'{name} contains invalid camera optical coordinates')
    return value


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input', type=Path, help='NPZ: points, scene_points in camera optical metres')
    parser.add_argument('output', type=Path)
    parser.add_argument('--root', type=Path, required=True, help='Directory containing official repositories')
    parser.add_argument('--checkpoint', type=Path, required=True)
    args = parser.parse_args()
    if args.output.suffix != '.npz':
        parser.error('output must end with .npz')
    if args.output.exists():
        raise FileExistsError(args.output)
    with np.load(args.input, allow_pickle=False) as data:
        points = cloud(data['points'], 'points')
        scene = cloud(data['scene_points'], 'scene_points')
    import torch
    if not torch.cuda.is_available():
        raise RuntimeError('CUDA is required by the official PointNet2 operators')
    baseline = args.root / 'graspnet-baseline'
    for folder in ('models', 'utils', 'pointnet2', 'knn'):
        sys.path.insert(0, str(baseline / folder))
    # Load only the official grasp representation, avoiding dataset/evaluation dependencies.
    package = types.ModuleType('graspnetAPI')
    package.__path__ = [str(args.root / 'graspnetAPI' / 'graspnetAPI')]
    sys.modules['graspnetAPI'] = package
    from graspnetAPI.grasp import GraspGroup
    from graspnet import GraspNet, pred_decode
    from collision_detector import ModelFreeCollisionDetector

    np.random.seed(0)
    torch.manual_seed(0)
    net = GraspNet(input_feature_dim=0, num_view=300, num_angle=12, num_depth=4,
                   cylinder_radius=.05, hmin=-.02, hmax_list=[.01, .02, .03, .04],
                   is_training=False).cuda()
    checkpoint = torch.load(args.checkpoint, map_location='cpu')
    net.load_state_dict(checkpoint['model_state_dict'], strict=True)
    net.eval()
    indices = np.random.choice(len(points), 20000, replace=len(points) < 20000)
    tensor = torch.from_numpy(points[indices][None]).cuda()
    torch.cuda.synchronize()
    started = time.monotonic()
    with torch.no_grad():
        raw = pred_decode(net({'point_clouds': tensor}))[0].cpu().numpy()
    torch.cuda.synchronize()
    elapsed = time.monotonic() - started
    if not np.isfinite(raw).all():
        raise ValueError('Nonfinite model output')
    group = GraspGroup(raw)
    detector = ModelFreeCollisionDetector(scene, voxel_size=.005)
    keep = []
    for start in range(0, len(group), 64):
        collision = detector.detect(group[start:start+64], approach_dist=.05, collision_thresh=.01)
        keep.extend((start + np.flatnonzero(~collision)).tolist())
    filtered = group[np.asarray(keep, dtype=int)]
    if len(filtered):
        filtered.nms()
        filtered.sort_by_score()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(args.output, raw_grasps=raw, grasps=filtered.grasp_group_array)
    report = {'model': 'official graspnet-baseline', 'seed': 0,
              'frame': 'camera_color_optical', 'units': 'metres',
              'columns': 'score,width,height,depth,R(9 row-major),translation(3),object_id',
              'axes': 'x approach, y jaw opening, z hand height; finger ends at translation+depth*x',
              'checkpoint_sha256': hashlib.sha256(args.checkpoint.read_bytes()).hexdigest(),
              'checkpoint_epoch': int(checkpoint['epoch']), 'torch': torch.__version__,
              'cuda_device': torch.cuda.get_device_name(0), 'inference_s': elapsed,
              'input_sha256': hashlib.sha256(args.input.read_bytes()).hexdigest(),
              'points': len(points), 'scene_points': len(scene), 'raw_count': len(raw),
              'collision_free_nms_count': len(filtered), 'collision_voxel_m': .005,
              'collision_threshold': .01, 'approach_distance_m': .05,
              'source_revisions': {name: subprocess.check_output(
                  ['git', '-C', str(args.root / name), 'rev-parse', 'HEAD'], text=True).strip()
                  for name in ('graspnet-baseline', 'graspnetAPI')},
              'limitations': 'Generic baseline gripper collision geometry; no robot IK or whole-arm collision check'}
    args.output.with_suffix('.json').write_text(json.dumps(report, indent=2))
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
