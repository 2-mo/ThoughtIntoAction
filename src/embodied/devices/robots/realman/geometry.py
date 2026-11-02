"""Metres and xyzw quaternions, with static XYZ Euler angles on the wire."""
import cv2
import numpy as np
from meshcat import transformations as tf


def matrix(quaternion):
    return tf.quaternion_matrix(np.roll(quaternion, 1))[:3, :3]


def quaternion(rotation):
    m = np.eye(4)
    m[:3, :3] = rotation
    return np.roll(tf.quaternion_from_matrix(m), -1)


def rotation_vector(first, second):
    return cv2.Rodrigues(matrix(first) @ matrix(second).T)[0].ravel()


def limited_rotation(target, previous, maximum):
    delta = rotation_vector(target, previous)
    delta *= min(1., maximum / max(np.linalg.norm(delta), 1e-12))
    return quaternion(cv2.Rodrigues(delta)[0] @ matrix(previous))
