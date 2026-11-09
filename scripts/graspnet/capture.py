"""Capture a stationary calibrated wrist RGB-D scene for remote GraspNet inference."""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np

from embodied.applications.calibration.session import pose_matrix
from embodied.applications.calibration.target import MarkerDetector, normalize_pixels
from embodied.applications.workstation.pair_state import active_pair
from embodied.contracts.runtime.control import ControlClient
from embodied.core.transforms import pose_error
from embodied.devices.sensors.client import SharedCamera


def point_cloud(depth, scale, camera):
    depth = np.asarray(depth)
    if depth.shape != (camera['height'], camera['width']) or depth.dtype != np.uint16:
        raise ValueError('Expected aligned uint16 RGB-sized depth')
    if not np.isfinite(scale) or scale <= 0:
        raise ValueError('Invalid depth scale')
    y, x = np.indices(depth.shape)
    rays = normalize_pixels(np.stack([x, y], -1).reshape(-1, 2), camera)
    return np.column_stack([rays, np.ones(len(rays))]).reshape(*depth.shape, 3) * (depth * scale)[..., None]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('output', type=Path, help='New capture directory')
    parser.add_argument('--roi', nargs=4, type=int, required=True, metavar=('X0', 'Y0', 'X1', 'Y1'),
                        help='Visible target region in current wrist RGB; orientation is predicted by GraspNet')
    parser.add_argument('--calibration', type=Path, default=Path('outputs/calibrations/hand_eye/wrist_rgb.json'))
    args = parser.parse_args()
    if args.output.exists():
        raise FileExistsError(args.output)
    calibration = json.loads(args.calibration.read_text())
    if (calibration['status'] != 'validated' or calibration['mount'] != 'wrist'
            or calibration['camera_id'] != 'wrist_rgb'):
        raise ValueError('Require validated wrist_rgb hand-eye calibration')
    pair = active_pair()
    if pair is None:
        raise RuntimeError('No running RealMan pair')
    run = Path(pair['runs']['left'])
    if json.loads((run / 'native-source.json').read_text())['enabled']:
        raise RuntimeError('Pause left Quest before stationary capture')
    client = ControlClient(run)
    before = client.status()
    if before['model_id'] != calibration['robot']['model_id'] or before['owner'] is not None:
        raise ValueError('Calibration model mismatch or arm currently controlled')
    with SharedCamera(calibration['camera']['serial'], camera_id='wrist_rgb') as camera:
        for _ in range(3):
            camera.read()
        rgb = camera.read()
        depth = camera.read_depth()
        if (rgb['device_timestamp_domain'] != depth['device_timestamp_domain']
                or abs(rgb['device_timestamp_ms'] - depth['device_timestamp_ms']) > 50):
            raise ValueError('RGB and depth differ by more than 50ms; recapture')
        detection = MarkerDetector(calibration['target'], camera.metadata).detect(rgb['image'])
        metadata = camera.metadata
    after = client.status()
    if (after['owner'] is not None or max(np.abs(np.array(before['q']) - after['q'])) > .003
            or max(np.abs(after['dq'])) > .02):
        raise ValueError('Arm moved during capture')
    base_camera = pose_matrix(after['pose']) @ np.array(calibration['T_eef_camera'])
    base_target = base_camera @ detection['T_camera_target']
    delta = pose_error(base_target, np.array(calibration['T_base_target']))
    if delta[0] > .015 or delta[1] > 2:
        raise ValueError(f'Board moved or calibration disagrees: {delta}')
    xyz = point_cloud(depth['depth_image'], depth['depth_scale_m'], metadata)
    base = xyz @ base_camera[:3, :3].T + base_camera[:3, 3]
    target = (base - base_target[:3, 3]) @ base_target[:3, :3]
    valid = ((depth['depth_image'] > 0) & (xyz[..., 2] < 1.2)
             & (np.abs(target[..., 0]) < .4) & (np.abs(target[..., 1]) < .4)
             & (target[..., 2] > -.015) & (target[..., 2] < .2))
    x0, y0, x1, y1 = args.roi
    if not (0 <= x0 < x1 <= metadata['width'] and 0 <= y0 < y1 <= metadata['height']):
        raise ValueError('ROI outside camera image')
    mask = np.zeros(valid.shape, dtype=bool)
    mask[y0:y1, x0:x1] = True
    # Include nearby table context, as in the official workspace-mask demo.
    points = xyz[valid & mask]
    if len(points) < 100 or valid.sum() < 100:
        raise ValueError('Insufficient depth in target or scene')
    args.output.mkdir(parents=True, exist_ok=False)
    cv2.imwrite(str(args.output / 'rgb.png'), rgb.pop('image'))
    cv2.imwrite(str(args.output / 'depth.png'), depth.pop('depth_image'))
    np.savez_compressed(args.output / 'input.npz', points=points.astype(np.float32),
                        scene_points=xyz[valid].astype(np.float32))
    record = {'schema': 1, 'camera': metadata, 'rgb': rgb, 'depth': depth,
              'robot_before': before, 'robot_after': after, 'run': str(run),
              'calibration': str(args.calibration.resolve()), 'roi': args.roi,
              'T_base_camera': base_camera.tolist(), 'T_base_target': base_target.tolist(),
              'board_delta_m_deg': list(delta), 'points': len(points), 'scene_points': int(valid.sum()),
              'note': 'Operator-selected target ROI; grasp orientation and score are model outputs; no motion'}
    (args.output / 'capture.json').write_text(json.dumps(record, indent=2))
    print(json.dumps({'output': str(args.output), 'points': len(points), 'board_delta_m_deg': list(delta)}))


if __name__ == '__main__':
    main()
