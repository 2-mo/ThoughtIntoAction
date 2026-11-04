"""Official nominal RM65 geometry, MuJoCo FK/Jacobian and local DLS IK."""
import mujoco
import numpy as np
from scipy.spatial.transform import Rotation
from embodied.devices.robots.arx5.kinematics import se3_error


def vector(value, count, name):
    value = np.asarray(value, dtype=float)
    if value.shape != (count,) or not np.isfinite(value).all():
        raise ValueError(f'{name} requires {count} finite values')
    return value


class Model:
    def __init__(self, config):
        config.model_id  # Validate model, mesh and tool identity before loading.
        self.model = mujoco.MjModel.from_xml_path(config.model_path)
        self.data = mujoco.MjData(self.model)
        self.site = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_SITE, 'tcp')
        self.tcp_link = 'tcp_link'
        names = [self.model.joint(i).name for i in range(self.model.njnt)]
        if (self.model.nq != 8 or self.model.nv != 8 or self.model.nu != 8 or self.site < 0
                or names != [*[f'joint{i}' for i in range(1, 7)], 'jaw_left', 'jaw_right']):
            raise ValueError('RM65 model requires six arm joints, two simplified jaws and a tcp site')
        self.lower, self.upper = np.asarray(config.joint_min), np.asarray(config.joint_max)
        if np.any(self.lower < self.model.jnt_range[:6, 0]) or np.any(self.upper > self.model.jnt_range[:6, 1]):
            raise ValueError('Configured arm limits exceed the official URDF limits')
        self.model.jnt_range[:6] = np.column_stack((self.lower, self.upper))
        self.jacp, self.jacr = np.zeros((3, 8)), np.zeros((3, 8))

    def fk(self, q):
        self.data.qpos[:6] = vector(q, 6, 'q')
        self.data.qvel[:] = 0
        mujoco.mj_forward(self.model, self.data)
        return self.pose(self.data)

    def pose(self, data):
        return np.r_[data.site_xpos[self.site],
                     Rotation.from_matrix(data.site_xmat[self.site].reshape(3, 3)).as_quat()]

    def solve_ik(self, target, q_seed, q_anchor=None):
        target = vector(target, 7, 'pose')
        if not np.isclose(np.linalg.norm(target[3:]), 1., atol=.01):
            raise ValueError('Pose quaternion must be normalized')
        goal = Rotation.from_quat(target[3:]).as_matrix()
        q = np.clip(vector(q_seed, 6, 'seed'), self.lower, self.upper)
        anchor = None if q_anchor is None else vector(q_anchor, 6, 'anchor')
        mu_squared = 0. if anchor is None else .005 ** 2
        regularizer = (.01 ** 2 + mu_squared) * np.eye(6)
        best, best_cost, best_residual = q.copy(), float('inf'), float('inf')
        # Same local SE(3) error, damping, measured-posture regularization and
        # finite partial solution policy as the existing ARX solver.
        for _ in range(50):
            current = self.fk(q)
            rotation = self.data.site_xmat[self.site].reshape(3, 3)
            error = se3_error(current[:3], rotation, target[:3], goal)
            residual = float(np.linalg.norm(error))
            cost = residual**2 + (0. if anchor is None else mu_squared * np.sum((q-anchor)**2))
            if cost < best_cost:
                best, best_cost, best_residual = q.copy(), cost, residual
            if residual < 1e-4 and anchor is None:
                return q, True, residual
            mujoco.mj_jacSite(self.model, self.data, self.jacp, self.jacr, self.site)
            jac = np.vstack((rotation.T @ self.jacp[:, :6], rotation.T @ self.jacr[:, :6]))
            rhs = jac.T @ error
            if anchor is not None:
                rhs += mu_squared * (anchor - q)
            try:
                delta = np.linalg.solve(jac.T @ jac + regularizer, rhs)
            except np.linalg.LinAlgError:
                return best, False, best_residual
            # A numerical trust step, not a robot velocity limit. Keep a
            # singular iteration local and return the best evaluated partial
            # solution if the requested pose is unreachable.
            delta *= min(1., .1 / max(float(np.max(np.abs(delta))), 1e-12))
            q = np.clip(q + delta, self.lower, self.upper)
            if anchor is not None and residual < 1e-4:
                return q, True, residual
        return best, False, best_residual
