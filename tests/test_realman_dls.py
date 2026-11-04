"""Read-back RM65 geometry and continuous local IK; no device connection."""
import unittest
from copy import deepcopy
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from embodied.devices.robots.realman.hardware_model import JointModel
from embodied.devices.robots.realman.kinematics import pose_error

# Controller readback from both installed RM65 arms, millidegrees/micrometres.
INFO = {
    'get_DH_data': {},
    'get_joint_min_pos': {'min_pos': [-178000, -130000, -135000, -178000, -128000, -360000]},
    'get_joint_max_pos': {'max_pos': [178000, 130000, 135000, 178000, 128000, 360000]},
    'get_current_tool_frame': {'pose': [0, 0, 140000, 0, 0, 0]},
    'get_current_work_frame': {'pose': [0, 0, 0, 0, 0, 0]},
}
INFO['get_DH_data'] = dict(zip((f'joint_{i}' for i in range(1, 7)), [
    [0, 0, 240500, 0], [90000, 0, 0, 90000], [0, 256000, 0, 90000],
    [90000, 0, 210000, 0], [-90000, 0, 0, 0], [90000, 0, 144000, 0]]))


class DlsTests(unittest.TestCase):
    def test_fk_matches_independent_pinned_vendor_fixtures(self):
        model = JointModel(INFO)
        fixtures = [([0]*6, [0, 0, .9905, 0, 0, 1, 0]),
            ([0, 0, 90, 0, 90, 0], [-.21, 0, .2125, 1, 0, 0, 0]),
            ([10, 20, -30, 40, 50, -60],
             [-.1564460248, -.1695856452, .89658916, -.165183112, -.3244271874, .9254686236, .1047346517])]
        for q, expected in fixtures:
            p, a = pose_error(model.fk(np.deg2rad(q)), np.array(expected))
            self.assertLess(p, 1e-6)
            self.assertLess(a, 1e-6)

    def test_calibrated_dh_tool_and_work_frames_and_jacobian(self):
        info = deepcopy(INFO)
        info['get_DH_data']['joint_3'][1] += 130
        model = JointModel(info)
        q = np.deg2rad([12, 20, 65, 13, 42, -10])
        base_pose = model.fk(q)
        tool_r = Rotation.from_euler('xyz', [.1, -.2, .3])
        work_r = Rotation.from_euler('xyz', [-.2, .1, -.4])
        tool_p, work_p = np.array([.02, -.01, .16]), np.array([.1, -.2, .03])
        flange_r = Rotation.from_quat(base_pose[3:])
        flange_p = base_pose[:3] - flange_r.apply([0, 0, .14])
        info['get_current_tool_frame']['pose'] = [20000, -10000, 160000, 100, -200, 300]
        info['get_current_work_frame']['pose'] = [100000, -200000, 30000, -200, 100, -400]
        model = JointModel(info)
        actual = model.fk(q)
        np.testing.assert_allclose(actual[:3], work_r.inv().apply(flange_p + flange_r.apply(tool_p)-work_p), atol=1e-10)
        self.assertLess((Rotation.from_quat(actual[3:]).inv()*work_r.inv()*flange_r*tool_r).magnitude(), 1e-10)
        mujoco.mj_jacSite(model.model, model.data, model.jacp, model.jacr, model.site)
        jac = np.vstack((model.jacp, model.jacr)).copy()
        for i in range(6):
            shifted = q.copy(); shifted[i] += 1e-6
            pose = model.fk(shifted)
            numerical = np.r_[(pose[:3]-actual[:3])/1e-6,
                (Rotation.from_quat(pose[3:])*Rotation.from_quat(actual[3:]).inv()).as_rotvec()/1e-6]
            np.testing.assert_allclose(jac[:, i], numerical, atol=1e-6)

    def test_continuous_solution_across_wrist_singularity(self):
        model = JointModel(INFO)
        q = np.deg2rad([10, 20, 50, 15, -8, 20])
        max_step = 0.
        for angle in np.linspace(-8, 8, 81):
            reference = np.deg2rad([10, 20, 50, 15, angle, 20])
            target = model.fk(reference)
            solved, _, residual = model.solve_ik(target, q, q)
            self.assertTrue(np.isfinite(solved).all())
            self.assertTrue(np.all((solved >= model.lower) & (solved <= model.upper)))
            max_step = max(max_step, float(np.max(np.abs(solved-q))))
            p, a = pose_error(model.fk(solved), target)
            self.assertLess(p, .001)
            self.assertLess(a, .005)
            q = solved
        self.assertLess(max_step, np.deg2rad(1))

    def test_unreachable_partial_solution_and_return_to_reachable_target(self):
        model = JointModel(INFO)
        q = np.deg2rad([0, 0, 90, 0, 90, 0])
        initial = model.fk(q)
        target = initial.copy(); target[0] -= .8
        partial, ok, residual = model.solve_ik(target, q, q)
        self.assertFalse(ok)
        self.assertTrue(np.isfinite(partial).all())
        self.assertTrue(np.all((partial >= model.lower) & (partial <= model.upper)))
        solved, _, _ = model.solve_ik(initial, partial, q)
        p, a = pose_error(model.fk(solved), initial)
        self.assertLess(p, .001)
        self.assertLess(a, .005)


if __name__ == '__main__': unittest.main()
