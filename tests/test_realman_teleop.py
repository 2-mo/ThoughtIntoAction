"""Hand mapping and boundary recovery without XR or robot connections."""
import numpy as np
import pytest
from meshcat import transformations as tf

from embodied.devices.robots.realman.backend import load_config
from embodied.devices.robots.realman.geometry import matrix, rotation_vector
from embodied.devices.robots.realman.teleop import RelativeTarget


def pose(position=(0., 0., 0.), angles=(0., 0., 0.)):
    return np.r_[position, np.roll(tf.quaternion_from_euler(*angles), -1)]


@pytest.mark.parametrize('axis', range(3))
def test_rotation_mapping_uses_world_axes_from_nonzero_hand_and_pregrasp(axis):
    config = load_config()
    tracker = RelativeTarget(config)
    robot = pose((-.21, 0., .2125), (np.pi, 0., 0.))
    hand = pose(angles=(.2, -.3, .1))
    np.testing.assert_allclose(tracker.update(hand, robot), robot, atol=1e-10)
    angles = np.zeros(3); angles[axis] = .1
    delta = tf.euler_matrix(*angles)[:3, :3]
    moved = hand.copy()
    full = np.eye(4); full[:3, :3] = delta @ matrix(hand[3:])
    moved[3:] = np.roll(tf.quaternion_from_matrix(full), -1)
    target = tracker.update(moved, robot)
    basis = np.asarray(config['xr_to_robot_basis'])
    np.testing.assert_allclose(matrix(target[3:]), basis @ delta @ basis.T @ matrix(robot[3:]), atol=1e-10)


def test_translation_boundary_discards_overshoot_and_reverses_without_reclutch():
    config = load_config()
    config.update(translation_radius_m=.15, rotation_radius_rad=.5)
    tracker = RelativeTarget(config)
    robot, hand = pose((-.21, 0., .2125), (np.pi, 0., 0.)), pose()
    tracker.update(hand, robot)
    for distance in (.5, .8, 1.):
        hand[2] = -distance
        target = tracker.update(hand, robot)
        assert 'translation_boundary' in tracker.limits
        assert target[0] == pytest.approx(robot[0] - config['translation_radius_m'])
    hand[2] += .01  # Still far beyond the original boundary, but now reversing.
    returned = tracker.update(hand, robot)
    assert returned[0] == pytest.approx(target[0] + .01 * config['position_scale'])
    assert 'translation_boundary' not in tracker.limits
    np.testing.assert_allclose(tracker.origin, robot)


def test_rotation_boundary_discards_overshoot_and_accepts_reverse_with_quaternion_sign_flip():
    config = load_config()
    config.update(translation_radius_m=.15, rotation_radius_rad=.5)
    tracker = RelativeTarget(config)
    robot, hand = pose((-.21, 0., .2125), (np.pi, 0., 0.)), pose()
    tracker.update(hand, robot)
    for angle in (.8, 1., 1.2):
        hand = pose(angles=(0., angle, 0.))
        target = tracker.update(hand, robot)
        assert 'rotation_boundary' in tracker.limits
        assert np.linalg.norm(rotation_vector(target[3:], robot[3:])) == pytest.approx(.5)
    hand = pose(angles=(0., 1.19, 0.)); hand[3:] *= -1
    returned = tracker.update(hand, robot)
    assert np.linalg.norm(rotation_vector(returned[3:], robot[3:])) == pytest.approx(.49)
    assert 'rotation_boundary' not in tracker.limits


def test_position_only_keeps_pregrasp_orientation_through_hand_rotation_and_reclutch():
    config = load_config()
    tracker = RelativeTarget(config, position_only=True)
    robot, hand = pose((-.21, 0., .2125), (np.pi, 0., 0.)), pose()
    tracker.update(hand, robot)
    moved = pose((.02, .03, -.04), (.8, -.7, .6))
    target = tracker.update(moved, robot)
    np.testing.assert_allclose(target[:3], [-.25, .02, .2425], atol=1e-10)
    np.testing.assert_allclose(matrix(target[3:]), matrix(robot[3:]), atol=1e-10)
    # The robot may lag behind the requested position: clutch to measured pose.
    measured = robot.copy(); measured[0] -= .01
    tracker.reset()
    np.testing.assert_allclose(tracker.update(moved, measured), measured, atol=1e-10)
    np.testing.assert_allclose(tracker.origin, robot)


def test_translation_saturation_does_not_change_rotation_mapping():
    config = load_config()
    config.update(translation_radius_m=.15, rotation_radius_rad=.5)
    tracker = RelativeTarget(config)
    robot = pose((-.21, 0., .2125), (np.pi, 0., 0.))
    tracker.update(pose(), robot)
    tracker.update(pose((0., 0., -.8), (0., .1, 0.)), robot)
    target = tracker.update(pose((0., 0., -.79), (0., .12, 0.)), robot)
    assert np.linalg.norm(rotation_vector(target[3:], robot[3:])) == pytest.approx(.12)
    assert target[0] == pytest.approx(robot[0] - .15 + .01)


def test_default_workspace_allows_large_translation_and_orientation_change():
    config = load_config()
    assert config['translation_radius_m'] is None
    assert config['rotation_radius_rad'] is None
    tracker = RelativeTarget(config)
    robot = pose((-.21, 0., .2125), (np.pi, 0., 0.))
    tracker.update(pose(), robot)
    target = tracker.update(pose((.4, -.3, -.5), (0., 1.5, 0.)), robot)
    np.testing.assert_allclose(target[:3] - robot[:3], [-.5, .4, -.3], atol=1e-10)
    assert np.linalg.norm(rotation_vector(target[3:], robot[3:])) == pytest.approx(1.5)
    assert tracker.limits == []


def test_incremental_mapping_discards_unsent_motion_and_rebases_on_sent_step():
    tracker = RelativeTarget(load_config(), incremental=True)
    robot = pose((-.21, 0., .2125), (np.pi, 0., 0.))
    tracker.update(pose(), robot)
    requested = tracker.update(pose((.10, 0, 0)), robot)
    assert requested[1] == pytest.approx(.10)
    # Hardware only accepts a 1 cm step; the remaining 9 cm is never carried over.
    accepted = robot.copy(); accepted[1] += .01
    next_target = tracker.update(pose((.11, 0, 0)), accepted)
    assert next_target[1] == pytest.approx(.02)
    tracker.update(pose((.11, 0, 0)), accepted)
    assert not tracker.moved
    tracker.reset()
    np.testing.assert_allclose(tracker.update(pose((.8, 0, 0)), robot), robot)


def test_incremental_mapping_accumulates_small_motion_above_tracking_noise():
    tracker = RelativeTarget(load_config(), incremental=True)
    robot = pose()
    tracker.update(pose(), robot)
    tracker.update(pose((.0001, 0, 0)), robot)
    assert not tracker.moved
    target = tracker.update(pose((.0004, 0, 0)), robot)
    assert tracker.moved
    assert target[1] == pytest.approx(.0004)
