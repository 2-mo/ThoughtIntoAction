"""Send a saved RGB-D point cloud to the GPU server; never command the robot."""
import argparse
import hashlib
import json
from pathlib import Path
import re
import shlex
import subprocess
import uuid

import numpy as np


def validate_result(archive, report, input_path):
    digest = hashlib.sha256(input_path.read_bytes()).hexdigest()
    if report['input_sha256'] != digest or report['frame'] != 'camera_color_optical':
        raise ValueError('Remote result does not match this input/frame')
    with np.load(archive, allow_pickle=False) as data:
        for name in ('grasps', 'raw_grasps'):
            rows = data[name]
            if rows.ndim != 2 or rows.shape[1] != 17 or not np.isfinite(rows).all():
                raise ValueError('Invalid GraspNet result array')
            if not len(rows):
                continue
            rotations = rows[:, 4:13].reshape(-1, 3, 3)
            if (not np.allclose(rotations.transpose(0, 2, 1) @ rotations, np.eye(3), atol=1e-4)
                    or not np.allclose(np.linalg.det(rotations), 1, atol=1e-4)
                    or np.any(rows[:, 1] < 0) or np.any(rows[:, 2:4] <= 0)
                    or np.any(rows[:, 15] <= 0)):
                raise ValueError('Invalid grasp dimensions, rotations or optical depth')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('capture', type=Path)
    parser.add_argument('--host', required=True, help='SSH user@host, using existing SSH authentication')
    parser.add_argument('--remote-root', required=True, help='Absolute deployment directory')
    parser.add_argument('--gpu', type=int, required=True)
    parser.add_argument('--control-path', type=Path, help='Existing SSH multiplex socket')
    args = parser.parse_args()
    if (not re.fullmatch(r'[A-Za-z0-9_.@-]+', args.host) or args.host.startswith('-')
            or not re.fullmatch(r'/[A-Za-z0-9_./-]+', args.remote_root) or args.gpu < 0):
        parser.error('Invalid SSH host, absolute remote directory, or GPU index')
    source = args.capture / 'input.npz'
    if not source.is_file():
        parser.error('capture/input.npz does not exist')
    output = args.capture / 'model'
    output.mkdir(exist_ok=False)
    root = args.remote_root.rstrip('/')
    remote = root + '/runs/' + uuid.uuid4().hex
    options = ['-o', 'ConnectTimeout=10', '-o', 'StrictHostKeyChecking=yes']
    if args.control_path:
        options += ['-o', 'ControlPath=' + str(args.control_path.resolve())]
    ssh = ['ssh', *options, args.host]
    scp = ['scp', '-q', *options]
    subprocess.run([*ssh, 'mkdir -p ' + shlex.quote(remote)], check=True, timeout=30)
    subprocess.run([*scp, str(source), args.host + ':' + remote + '/input.npz'], check=True, timeout=60)
    subprocess.run([*scp, str(Path(__file__).with_name('infer.py')),
                    args.host + ':' + remote + '/infer.py'], check=True, timeout=60)
    command = shlex.join(['env', f'CUDA_VISIBLE_DEVICES={args.gpu}', 'OMP_NUM_THREADS=2',
                          root + '/graspnet-env/bin/python', remote + '/infer.py',
                          remote + '/input.npz', remote + '/grasps.npz', '--root', root,
                          '--checkpoint', root + '/weights/checkpoint-rs.tar'])
    with (output / 'remote.log').open('w') as log:
        subprocess.run([*ssh, command], stdout=log, stderr=subprocess.STDOUT, check=True, timeout=180)
    for name in ('grasps.npz', 'grasps.json'):
        subprocess.run([*scp, args.host + ':' + remote + '/' + name, str(output / name)],
                       check=True, timeout=60)
    report = json.loads((output / 'grasps.json').read_text())
    validate_result(output / 'grasps.npz', report, source)
    (output / 'request.json').write_text(json.dumps({'host': args.host, 'remote_run': remote,
                                                  'gpu': args.gpu, 'command': command}, indent=2))
    print(json.dumps({'result': str(output), 'count': report['collision_free_nms_count'],
                      'inference_s': report['inference_s'], 'robot_motion': False}))


if __name__ == '__main__':
    main()
