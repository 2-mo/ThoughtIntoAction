"""Real plugin metadata crosses isolated control, display, export and policy APIs."""
import json
import os
import subprocess
import unittest

import numpy as np

from embodied.contracts.paths import PROJECT_ROOT


class Reference7ContractIntegration(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        python = PROJECT_ROOT / 'outputs/envs/control/bin/python'
        if not python.is_file():
            raise unittest.SkipTest('Requires the existing control environment with MuJoCo')
        program = '''
import json
from embodied.control.settings import load_settings
from embodied.contracts.registry import Registry
from embodied.contracts.paths import PROJECT_ROOT
from embodied.devices.robots.reference7.model import Model
config = load_settings(PROJECT_ROOT / 'configs/robots/reference7/robot.json').robot
driver = Registry().create('robot', config.simulation_plugin, runtime='control')
q = list(config.park_q)
q[-1] += .2
parameters = driver.snapshot(config)
print(json.dumps({'descriptor': {'joint_count': len(q), 'parameters': parameters,
    'data_contract': parameters['data_contract']}, 'q': q, 'pose': Model(config).fk(q).tolist()}))
'''
        result = subprocess.run([str(python), '-c', program], cwd=PROJECT_ROOT,
            env={**os.environ, 'PYTHONPATH': str(PROJECT_ROOT / 'src')},
            text=True, capture_output=True, timeout=30, check=True)
        cls.fixture = json.loads(result.stdout)

    def test_runtime_urdf_and_mujoco_agree_on_tcp_and_seven_axis_order(self):
        import pinocchio as pin
        from embodied.applications.monitor.model import RuntimeModel
        robot = RuntimeModel(self.fixture['descriptor'])
        q = robot.joints(self.fixture['q'])
        data = robot.model.createData()
        pin.framesForwardKinematics(robot.model, data, q)
        pose = data.oMf[robot.frame]
        expected = self.fixture['pose']
        np.testing.assert_allclose(pose.translation, expected[:3], atol=1e-9)
        np.testing.assert_allclose(pose.rotation, pin.Quaternion(np.asarray(expected[3:])).matrix(), atol=1e-9)
        self.assertEqual(robot.names, [f'joint{i}' for i in range(1, 8)])
        self.assertIsNone(robot.data_contract['gripper'])

    def test_exported_features_feed_same_training_and_inference_contract(self):
        from embodied.data.episode.contract import action_definition, vector
        from embodied.data.export.lerobot import export_features
        from embodied.policy.openpi.adapter import from_dataset, transforms
        contract = self.fixture['descriptor']['data_contract']
        roles = {'base_0_rgb': 'front_rgb'}
        features = export_features(contract, {'front_rgb': {'width': 8, 'height': 8}})
        spec = from_dataset({'data_contract': contract, 'camera_roles': roles,
            'action_definition': action_definition(contract)}, {'features': features})
        inputs, outputs, delta_mask = transforms(spec)
        state = vector({'q': self.fixture['q'], 'gripper_width': 0.}, contract)
        observed = inputs({'state': state, 'front_rgb': np.zeros((8, 8, 3), np.uint8),
                           'front_rgb_valid': True, 'prompt': 'move joint seven'})
        self.assertEqual(observed['state'].shape, (7,))
        np.testing.assert_array_equal(delta_mask, [True] * 7)
        action = np.arange(spec['model_action_dim'], dtype=float)[None, :]
        np.testing.assert_array_equal(outputs({'actions': action})['actions'], action[:, :7])
        self.assertEqual(features['action']['names'], contract['joint_names'])
        self.assertTrue(observed['image_mask']['base_0_rgb'])
        self.assertFalse(observed['image_mask']['left_wrist_0_rgb'])


if __name__ == '__main__':
    unittest.main()
