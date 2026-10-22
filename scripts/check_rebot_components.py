"""Offline B601 model, dynamics and trajectory regression in the rebot environment."""
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
import numpy as np
import pinocchio as pin
from embodied.contracts.registry import Registry
from embodied.devices.robots.rebot.kinematics.robot_model import load_robot_model, get_joint_limits
from embodied.devices.robots.rebot.dynamics import (
    compute_generalized_gravity, compute_inverse_dynamics, compute_mass_matrix,
)
from embodied.devices.robots.rebot.trajectory.sampler import _apply_profile, TrajProfile


def main():
    registry = Registry()
    for variant, relative in {
        'dm': 'reBot-DevArm_fixend_description/urdf/reBot-DevArm_fixend.urdf',
        'rs': '00-arm-rs_asm-v3/urdf/00-arm-rs_asm-v3.urdf',
    }.items():
        device = registry.create('device', 'rebot_' + variant, runtime='rebot')
        model = load_robot_model(str(ROOT / 'assets/robots/rebot' / relative))
        q = pin.neutral(model)
        limits = get_joint_limits(model)
        assert len(limits) == len(model.names) - 1
        for name, bounds in zip(model.names[1:], limits):
            idx = model.joints[model.getJointId(name)].idx_q
            assert bounds == (model.lowerPositionLimit[idx], model.upperPositionLimit[idx])
        gravity = compute_generalized_gravity(model=model, q=q)
        torque = compute_inverse_dynamics(model=model, q=q, v=np.zeros(model.nv), a=np.zeros(model.nv))
        assert np.allclose(gravity, torque)
        inertia = compute_mass_matrix(model=model, q=q)
        assert np.allclose(inertia, inertia.T)
        assert np.linalg.eigvalsh(inertia).min() > 0
        print(variant, device.num_joints, 'motors;', model.nq, 'model coordinates; offline checks passed')
    for ratio in (0.01, 0.25, 0.49):
        values = np.array([_apply_profile(float(t), TrajProfile.TRAPEZOID, ratio)
                           for t in np.linspace(0, 1, 10001)])
        assert values[0] == 0 and values[-1] == 1
        assert values.min() >= 0 and values.max() <= 1
        assert np.min(np.diff(values)) >= 0
        for boundary in (ratio, 1 - ratio):
            assert abs(_apply_profile(boundary + 1e-7, TrajProfile.TRAPEZOID, ratio)
                       - _apply_profile(boundary - 1e-7, TrajProfile.TRAPEZOID, ratio)) < 1e-5
    print('Trajectory continuity/monotonicity regression passed; no hardware opened')


if __name__ == '__main__':
    main()
