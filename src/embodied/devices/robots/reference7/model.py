"""MuJoCo FK and damped least-squares IK for the Reference7 model."""
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation


def vector(value, count, name):
    value = np.asarray(value, dtype=float)
    if value.shape != (count,) or not np.isfinite(value).all():
        raise ValueError(f'{name} requires {count} finite values')
    return value


class Model:
    def __init__(self, config):
        self.model = mujoco.MjModel.from_xml_path(config.model_path)
        self.data = mujoco.MjData(self.model)
        self.site = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, 'tcp')
        self.tcp_link = 'tcp_link'
        if self.model.nq != 7 or self.model.nv != 7 or self.model.nu != 7 or self.site < 0:
            raise ValueError('Reference7 requires seven joints/actuators and a tcp site')
        self.lower, self.upper = np.asarray(config.joint_min), np.asarray(config.joint_max)
        self.model.jnt_range[:] = np.column_stack((self.lower, self.upper))
        self.jacp, self.jacr = np.zeros((3, 7)), np.zeros((3, 7))

    def fk(self, q):
        self.data.qpos[:] = vector(q, 7, 'q')
        self.data.qvel[:] = 0
        mujoco.mj_forward(self.model, self.data)
        return self.pose(self.data)

    def pose(self, data):
        return np.r_[data.site_xpos[self.site],
                     Rotation.from_matrix(data.site_xmat[self.site].reshape(3, 3)).as_quat()]

    def solve_ik(self, target, q_seed):
        target = vector(target, 7, 'pose')
        if not np.isclose(np.linalg.norm(target[3:]), 1., atol=.01):
            raise ValueError('Pose quaternion must be normalized')
        goal = Rotation.from_quat(target[3:])
        q = np.clip(vector(q_seed, 7, 'seed'), self.lower, self.upper)
        for _ in range(120):
            current = self.fk(q)
            error = np.r_[target[:3] - current[:3],
                          (goal * Rotation.from_quat(current[3:]).inv()).as_rotvec()]
            residual = float(np.linalg.norm(error))
            if residual < 1e-5:
                return q, True, residual
            mujoco.mj_jacSite(self.model, self.data, self.jacp, self.jacr, self.site)
            jac = np.vstack((self.jacp, self.jacr))
            delta = jac.T @ np.linalg.solve(jac @ jac.T + 1e-5 * np.eye(6), error)
            q = np.clip(q + np.clip(delta, -.05, .05), self.lower, self.upper)
        current = self.fk(q)
        residual = float(np.linalg.norm(np.r_[target[:3] - current[:3],
            (goal * Rotation.from_quat(current[3:]).inv()).as_rotvec()]))
        return q, residual < 1e-5, residual
