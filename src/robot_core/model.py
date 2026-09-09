"""URDF-driven X5 kinematics; SI units and link6 match the teleop EEF."""

from pathlib import Path
import hashlib

import numpy as np
import pinocchio as pin

from .project import PROJECT_ROOT as ROOT, X5_2025_URDF as DEFAULT_URDF


class RobotModel:
    def __init__(self, urdf=DEFAULT_URDF, eef="link6"):
        self.urdf = Path(urdf).resolve()
        self.model = pin.buildModelFromUrdf(str(self.urdf))
        if self.model.nq != 6 or self.model.nv != 6:
            raise ValueError("Expected the fixed-base, six-joint X5 model")
        self.eef = eef
        self.frame = self.model.getFrameId(eef)
        if self.frame >= len(self.model.frames):
            raise ValueError(f"Unknown EEF frame: {eef}")
        self.data = self.model.createData()
        # Position arrays use idx_q, NOT the one-based joint ID.
        self.lower = self.model.lowerPositionLimit.copy()
        self.upper = self.model.upperPositionLimit.copy()
        self.names = list(self.model.names)[1:]

    def metadata(self):
        return {"urdf_sha256": hashlib.sha256(self.urdf.read_bytes()).hexdigest(),
                "eef_frame": self.eef, "base_frame": "base_link",
                "joint_names": self.names, "joint_unit": "rad", "length_unit": "m"}

    def joints(self, q, margin=0.0):
        q = np.asarray(q, dtype=float)
        if q.shape != (6,) or not np.isfinite(q).all():
            raise ValueError("Expected six finite joint angles in radians")
        if not np.isfinite(margin) or margin < 0:
            raise ValueError("Invalid joint margin")
        lower, upper = self.lower + margin, self.upper - margin
        # QP solutions can land a few ulps beyond an active constraint.
        # Normalize only numerical noise; returned commands stay inside limits.
        tolerance = 1e-8  # rad, about 0.00000057 degrees
        if np.any(lower > upper) or np.any(q < lower - tolerance) or np.any(q > upper + tolerance):
            values = np.array2string(q, precision=17)
            raise ValueError(f"Joint target outside model limits (margin={margin}): {values}")
        return np.clip(q, lower, upper)

    def fk(self, q):
        q = self.joints(q)
        pin.framesForwardKinematics(self.model, self.data, q)
        return self.data.oMf[self.frame].homogeneous.copy()

    def ik(self, target, seed, iterations=250):
        from .transforms import transform
        target = pin.SE3(transform(target))
        q = self.joints(seed)
        for _ in range(iterations):
            pin.framesForwardKinematics(self.model, self.data, q)
            local_error = self.data.oMf[self.frame].actInv(target)
            error = pin.log6(local_error).vector
            if np.linalg.norm(error[:3]) < 1e-4 and np.linalg.norm(error[3:]) < 1e-3:
                return self.joints(q)
            jac = pin.computeFrameJacobian(self.model, self.data, q, self.frame, pin.LOCAL)
            jac = -pin.Jlog6(local_error.inverse()) @ jac
            step = -jac.T @ np.linalg.solve(jac @ jac.T + 1e-5 * np.eye(6), error)
            step *= min(1.0, 0.08 / max(np.max(np.abs(step)), 1e-12))
            # Backtracking avoids increasing the SE(3) error near a limit.
            accepted = False
            for fraction in (1.0, 0.5, 0.25, 0.1):
                trial = np.clip(q + fraction * step, self.lower, self.upper)
                pose = pin.SE3(self.fk(trial))
                if np.linalg.norm(pin.log6(pose.actInv(target)).vector) < np.linalg.norm(error):
                    q, accepted = trial, True
                    break
            if not accepted:
                break
        raise ValueError("IK did not converge within joint limits; no target accepted")
