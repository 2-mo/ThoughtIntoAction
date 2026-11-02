"""Compare the simulation TCP with the pinned official Algo, entirely offline."""
import json
import os
import subprocess

import numpy as np

from embodied.contracts.paths import PROJECT_ROOT, CONTROL_PYTHON
from .geometry import rotation_vector
from .kinematics import load_sdk


def check():
    sdk = load_sdk()
    algo = sdk.Algo(sdk.rm_robot_arm_model_e.RM_MODEL_RM_65_E, sdk.rm_force_type_e.RM_MODEL_RM_B_E)
    config = json.loads((PROJECT_ROOT / 'configs/robots/realman/sim.json').read_text())
    algo.rm_algo_set_toolframe(sdk.rm_frame_t('Lq', (0, 0, config['tool_offset_m'], 0, 0, 0)))
    algo.rm_algo_set_workframe(sdk.rm_frame_t('Base', (0, 0, 0, 0, 0, 0)))
    rng = np.random.default_rng(42)
    qs = np.vstack([np.zeros(6), config['park_q'], rng.uniform(config['joint_min'], config['joint_max'], (32, 6))])
    program = '''
import json, sys
from embodied.control.settings import load_settings
from embodied.devices.robots.realman.model import Model
config = load_settings('configs/robots/realman/sim.json').robot
model = Model(config)
print(json.dumps({'poses': [model.fk(q).tolist() for q in json.load(sys.stdin)], 'model_id': config.model_id}))
'''
    result = subprocess.run([str(CONTROL_PYTHON), '-c', program], input=json.dumps(qs.tolist()),
        cwd=PROJECT_ROOT, env={**os.environ, 'PYTHONPATH': str(PROJECT_ROOT / 'src')},
        check=True, capture_output=True, text=True, timeout=30)
    model = json.loads(result.stdout)
    errors = []
    for q, pose in zip(qs, model['poses']):
        raw = np.asarray(algo.rm_algo_forward_kinematics(np.rad2deg(q).tolist(), 0))
        vendor = np.r_[raw[:3], raw[4:], raw[3]]
        errors.append({'joint_rad': q.tolist(), 'simulation_pose_m_xyzw': pose,
                       'vendor_pose_m_xyzw': vendor.tolist(),
                       'position_error_m': float(np.linalg.norm(vendor[:3] - pose[:3])),
                       'angle_error_rad': float(np.linalg.norm(rotation_vector(vendor[3:], pose[3:])))})
    report = {'source': 'RM_API2 pinned Algo(RM65, B), nominal DH, Base identity',
              'model_id': model['model_id'], 'algorithm_version': str(algo.rm_algo_version()),
              'seed': 42, 'samples': len(qs), 'tool_offset_m': config['tool_offset_m'],
              'position_tolerance_m': 2e-5, 'angle_tolerance_rad': 3e-5,
              'max_position_error_m': max(row['position_error_m'] for row in errors),
              'max_angle_error_rad': max(row['angle_error_rad'] for row in errors),
              'physical_robot_queried': False, 'results': errors}
    report['passed'] = report['max_position_error_m'] < report['position_tolerance_m'] and report['max_angle_error_rad'] < report['angle_tolerance_rad']
    path = PROJECT_ROOT / 'outputs/realman/sim-kinematics.json'
    path.parent.mkdir(exist_ok=True, parents=True)
    path.write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps({**{k: v for k,v in report.items() if k != 'results'}, 'report': str(path)}, ensure_ascii=False))
    return 0 if report['passed'] else 1
