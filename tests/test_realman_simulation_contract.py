"""The RM65 simulation and the platform's URDF monitor share the same TCP."""
import json
import os
from pathlib import Path
import subprocess
import unittest

import numpy as np

from embodied.contracts.paths import PROJECT_ROOT, CONTROL_PYTHON


class DisplayContractTests(unittest.TestCase):
    def test_runtime_urdf_matches_mujoco_pose_and_six_axis_contract(self):
        if not CONTROL_PYTHON.is_file():
            self.skipTest('Needs the existing control environment')
        code = '''
import json
from embodied.control.settings import load_settings
from embodied.devices.robots.realman.simulation import Simulation
from embodied.devices.robots.realman.model import Model
config=load_settings('configs/robots/realman/sim.json').robot
q=list(config.park_q)
q[0]+=.1
p=Simulation().snapshot(config)
print(json.dumps({'descriptor':{'joint_count':6,'parameters':p,'data_contract':p['data_contract']},
                  'q':q,'pose':Model(config).fk(q).tolist()}))
'''
        result = subprocess.run([str(CONTROL_PYTHON), '-c', code], cwd=PROJECT_ROOT,
            env={**os.environ, 'PYTHONPATH': str(PROJECT_ROOT/'src')},
            check=True, capture_output=True, text=True, timeout=30)
        fixture = json.loads(result.stdout)
        from embodied.applications.monitor.model import RuntimeModel
        import pinocchio as pin
        robot = RuntimeModel(fixture['descriptor'])
        data = robot.model.createData()
        pin.framesForwardKinematics(robot.model, data, robot.joints(fixture['q']))
        pose = data.oMf[robot.frame]
        np.testing.assert_allclose(pose.translation, fixture['pose'][:3], atol=1e-10)
        np.testing.assert_allclose(pose.rotation, pin.Quaternion(np.asarray(fixture['pose'][3:])).matrix(), atol=1e-10)
        visual = pin.buildGeomFromUrdf(robot.model, str(robot.urdf), pin.GeometryType.VISUAL,
                                       package_dirs=[str(robot.urdf.parent)])
        meshes = [g for g in visual.geometryObjects if g.meshPath.lower().endswith('.stl')]
        self.assertEqual(len(meshes), 7)
        self.assertTrue(all(Path(g.meshPath).is_file() for g in meshes))
        self.assertEqual(robot.names, [f'joint{i}' for i in range(1,7)])
        self.assertEqual(robot.data_contract['robot_type'], 'rm65_sim_simplified_gripper')


if __name__ == '__main__':
    unittest.main()
