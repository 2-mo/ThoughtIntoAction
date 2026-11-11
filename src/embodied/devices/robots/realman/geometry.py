"""Metres and xyzw quaternions, with static XYZ Euler angles on the wire."""
import numpy as np
from scipy.spatial.transform import Rotation


def matrix(quaternion):
    return Rotation.from_quat(quaternion).as_matrix()


def quaternion(rotation):
    return Rotation.from_matrix(rotation).as_quat()


def rotation_vector(first, second):
    return Rotation.from_matrix(matrix(first) @ matrix(second).T).as_rotvec()


def limited_rotation(target, previous, maximum):
    delta = rotation_vector(target, previous)
    delta *= min(1., maximum / max(np.linalg.norm(delta), 1e-12))
    return quaternion(Rotation.from_rotvec(delta).as_matrix() @ matrix(previous))
