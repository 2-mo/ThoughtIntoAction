"""Check base/tool/aligned rotation semantics through XR processing and IK targets."""
import unittest
from types import SimpleNamespace
from unittest.mock import Mock
from quest_teleop.controller import X5TeleopController
from quest_teleop.target_limits import CartesianTargetLimiter
import numpy as np
import meshcat.transformations as tf
from quest_teleop.base_controller import R_HEADSET_TO_WORLD
from quest_teleop.frames import operator_yaw_rotation
from xrobotoolkit_teleop.utils.geometry import apply_delta_pose


class OperatorFrameTests(unittest.TestCase):
    def controller(self, yaw, rotation_frame="base", tool_quat=None):
        obj = X5TeleopController.__new__(X5TeleopController)
        obj.rotation_frame = rotation_frame
        obj._cartesian_limits = {"right_arm": CartesianTargetLimiter(0, 0)}
        obj.dt = .02
        obj.R_headset_world = operator_yaw_rotation(yaw) @ R_HEADSET_TO_WORLD
        obj.scale_factor = 1.
        obj.ref_controller_xyz = {"right_arm": None}
        obj.ref_controller_quat = {"right_arm": None}
        obj.ref_ee_quat = {"right_arm": tool_quat}
        obj.manipulator_config = {"right_arm": {}}
        obj.reset_rotation_alignment()
        return obj

    def pose(self, quat):
        return [1, 2, 3, *quat[[1, 2, 3, 0]]]

    def test_horizontal_axes_and_vertical_for_quarter_turns(self):
        for yaw, expected in [(0, [1, 0, 0]), (90, [0, 1, 0]), (-90, [0, -1, 0]), (180, [-1, 0, 0])]:
            obj = self.controller(yaw)
            origin = [1, 2, 3, 0, 0, 0, 1]
            delta, rot = obj._process_xr_pose(origin, "right_arm")
            np.testing.assert_allclose(delta, 0, atol=1e-10)
            np.testing.assert_allclose(rot, 0, atol=1e-10)
            delta, _ = obj._process_xr_pose([1, 2, 2, 0, 0, 0, 1], "right_arm")
            np.testing.assert_allclose(delta, expected, atol=1e-10)
            delta, _ = obj._process_xr_pose([1, 3, 3, 0, 0, 0, 1], "right_arm")
            np.testing.assert_allclose(delta, [0, 0, 1], atol=1e-10)

    def test_rotations_use_same_world_frame_even_with_nonidentity_anchor(self):
        initial = tf.quaternion_from_euler(.3, -.4, .2)
        for yaw in (0, 90, -90, 180):
            for axis in np.eye(3):
                obj = self.controller(yaw)
                obj._process_xr_pose([0, 0, 0, *initial[[1, 2, 3, 0]]], "right_arm")
                current = tf.quaternion_multiply(tf.quaternion_about_axis(.15, axis), initial)
                _, rot = obj._process_xr_pose([0, 0, 0, *current[[1, 2, 3, 0]]], "right_arm")
                np.testing.assert_allclose(rot, obj.R_headset_world @ (axis * .15), atol=1e-8)

    def test_invalid_heading_rejected(self):
        for value in (float("nan"), float("inf"), -float("inf")):
            with self.assertRaises(ValueError):
                operator_yaw_rotation(value)

    def test_tool_twist_pitch_and_yaw_follow_tool_axes_at_any_heading(self):
        # Quest: forward -Z, right +X, up +Y. X5 link6: forward +X,
        # right -Y, up +Z. Neither the operator heading nor grip angle
        # may turn a forward-axis twist into pitch.
        axes = [([0, 0, -1], [1, 0, 0]), ([1, 0, 0], [0, -1, 0]),
                ([0, 1, 0], [0, 0, 1])]
        initial = tf.quaternion_from_euler(.4, -.7, .9)
        tool = tf.quaternion_from_euler(-.3, 1.2, -.6)
        for yaw in (0, 90, -90, 180):
            for controller_axis, tool_axis in axes:
                for angle in (-.2, .2):
                    with self.subTest(yaw=yaw, axis=controller_axis, angle=angle):
                        obj = self.controller(yaw, "tool", tool)
                        obj._process_xr_pose(self.pose(initial), "right_arm")
                        current = tf.quaternion_multiply(initial, tf.quaternion_about_axis(angle, controller_axis))
                        delta, rotation = obj._process_xr_pose(self.pose(current), "right_arm")
                        pos, quat = apply_delta_pose(np.zeros(3), tool, delta, rotation)
                        target = tf.quaternion_matrix(quat)[:3, :3]
                        # Inspect the actual tool-local target, not its rotvec representation.
                        expected = tf.rotation_matrix(angle, tool_axis)[:3, :3]
                        np.testing.assert_allclose(tf.quaternion_matrix(tool)[:3, :3].T @ target,
                                                   expected, atol=1e-8)
                        np.testing.assert_allclose(pos, 0, atol=1e-10)

    def test_aligned_reference_matches_all_three_selected_gestures(self):
        # User's chosen reference: controller horizontal forward, gripper down.
        # Forward twist -> side tilt, nod -> fore/aft tilt, yaw -> vertical spin.
        initial = tf.quaternion_from_euler(.4, -.7, .9)
        gestures = [([0, 0, -1], [1, 0, 0]), ([1, 0, 0], [0, -1, 0]),
                    ([0, 1, 0], [0, 0, 1])]
        for heading in (0., .6):
            heading_matrix = tf.rotation_matrix(heading, [0, 0, 1])
            grasp = heading_matrix @ tf.rotation_matrix(np.pi / 2, [0, 1, 0])
            tool = tf.quaternion_from_matrix(grasp)
            for yaw in (0, 90, -90, 180):
                for controller_axis, expected_axis in gestures:
                    for angle in (-.2, .2):
                        with self.subTest(heading=heading, yaw=yaw, axis=controller_axis, angle=angle):
                            obj = self.controller(yaw, "aligned", tool)
                            delta, rotation = obj._process_xr_pose(self.pose(initial), "right_arm")
                            np.testing.assert_allclose(rotation, 0, atol=1e-10)
                            current = tf.quaternion_multiply(initial, tf.quaternion_about_axis(angle, controller_axis))
                            delta, rotation = obj._process_xr_pose(self.pose(current), "right_arm")
                            _, quat = apply_delta_pose(np.zeros(3), tool, delta, rotation)
                            axis = heading_matrix[:3, :3] @ expected_axis
                            expected = tf.rotation_matrix(angle, axis) @ grasp
                            np.testing.assert_allclose(tf.quaternion_matrix(quat), expected, atol=1e-8)

    def test_aligned_axes_survive_regrip_after_yaw_and_tool_pose_change(self):
        grasp = tf.rotation_matrix(np.pi / 2, [0, 1, 0])
        initial = tf.quaternion_about_axis(.4, [0, 1, 0])
        obj = self.controller(90, "aligned", tf.quaternion_from_matrix(grasp))
        obj._process_xr_pose(self.pose(initial), "right_arm")
        basis = np.array(obj._rotation_alignment["right_arm"])
        position_basis = np.array(obj._position_alignment["right_arm"])
        np.testing.assert_allclose(basis.T @ basis, np.eye(3), atol=1e-10)
        self.assertAlmostEqual(np.linalg.det(basis), 1.)
        # After a large yaw, regrip with a different robot/controller reference.
        current = tf.quaternion_multiply(initial, tf.quaternion_about_axis(.8, [0, 1, 0]))
        obj._process_xr_pose(self.pose(current), "right_arm")
        obj.ref_controller_xyz["right_arm"] = None
        obj.ref_ee_quat["right_arm"] = tf.quaternion_from_euler(.7, -.5, .9)
        delta, rotation = obj._process_xr_pose(self.pose(current), "right_arm")
        np.testing.assert_allclose(delta, 0, atol=1e-10)
        np.testing.assert_allclose(rotation, 0, atol=1e-10)
        np.testing.assert_array_equal(obj._rotation_alignment["right_arm"], basis)
        np.testing.assert_array_equal(obj._position_alignment["right_arm"], position_basis)
        # A rotation about the originally aligned forward world axis stays a
        # side tilt, even though neither new Grip orientation equals the old one.
        forward = tf.quaternion_matrix(initial)[:3, :3] @ [0, 0, -1]
        current = tf.quaternion_multiply(tf.quaternion_about_axis(.15, forward), current)
        pose = self.pose(-current)  # q and -q must remain equivalent.
        pose[2] -= .1
        delta, rotation = obj._process_xr_pose(pose, "right_arm")
        np.testing.assert_allclose(rotation, [.15, 0, 0], atol=1e-8)
        np.testing.assert_allclose(delta, [.1 * np.cos(.4), -.1 * np.sin(.4), 0], atol=1e-10)
        obj.reset_rotation_alignment()
        self.assertIsNone(obj._rotation_alignment["right_arm"])
        self.assertIsNone(obj._position_alignment["right_arm"])

    def test_aligned_translation_matches_forward_right_and_up_after_yaw_recenter(self):
        axes = [([0, 0, -1], [1, 0, 0]), ([1, 0, 0], [0, -1, 0]),
                ([0, 1, 0], [0, 0, 1])]
        for quest_heading in (0, .7, np.pi / 2, -np.pi / 2):
            initial = tf.quaternion_about_axis(quest_heading, [0, 1, 0])
            for robot_heading in (0, .6):
                heading = tf.rotation_matrix(robot_heading, [0, 0, 1])
                grasp = heading @ tf.rotation_matrix(np.pi / 2, [0, 1, 0])
                # The old +90 fixed translation must not be added to calibration.
                obj = self.controller(90, "aligned", tf.quaternion_from_matrix(grasp))
                obj._process_xr_pose(self.pose(initial), "right_arm")
                for local_axis, expected_axis in axes:
                    for distance in (-.1, .1):
                        with self.subTest(quest=quest_heading, robot=robot_heading, axis=local_axis, distance=distance):
                            pose = self.pose(initial)
                            pose[:3] = np.array(pose[:3]) + tf.quaternion_matrix(initial)[:3, :3] @ (np.array(local_axis) * distance)
                            position, rotation = obj._process_xr_pose(pose, "right_arm")
                            np.testing.assert_allclose(position, heading[:3, :3] @ (np.array(expected_axis) * distance), atol=1e-8)
                            np.testing.assert_allclose(rotation, 0, atol=1e-10)

    def test_aligned_translation_keeps_vertical_despite_reference_tilt(self):
        initial = tf.quaternion_from_euler(.2, .5, -.1)
        tool = tf.quaternion_from_euler(.1, 1.4, .3)
        obj = self.controller(90, "aligned", tool)
        obj._process_xr_pose(self.pose(initial), "right_arm")
        basis = np.array(obj._position_alignment["right_arm"])
        np.testing.assert_allclose(basis.T @ basis, np.eye(3), atol=1e-10)
        self.assertAlmostEqual(np.linalg.det(basis), 1.)
        pose = self.pose(initial)
        pose[1] += .1
        position, _ = obj._process_xr_pose(pose, "right_arm")
        np.testing.assert_allclose(position, [0, 0, .1], atol=1e-10)
        pose = self.pose(initial)
        pose[0] += .1
        pose[2] -= .1
        position, _ = obj._process_xr_pose(pose, "right_arm")
        self.assertAlmostEqual(position[2], 0.)

    def test_tool_mode_preserves_translation_and_regrip_resets_rotation(self):
        obj = self.controller(90, "tool", tf.quaternion_from_euler(.2, .8, -.3))
        initial = tf.quaternion_from_euler(.5, .7, -.4)
        obj._process_xr_pose(self.pose(initial), "right_arm")
        current = tf.quaternion_multiply(initial, tf.quaternion_about_axis(.3, [0, 0, -1]))
        moved = self.pose(current)
        moved[2] -= .1
        delta, _ = obj._process_xr_pose(moved, "right_arm")
        np.testing.assert_allclose(delta, [0, .1, 0], atol=1e-10)
        obj.ref_controller_xyz["right_arm"] = None
        obj.ref_ee_quat["right_arm"] = tf.quaternion_from_euler(-.8, .1, .5)
        delta, rotation = obj._process_xr_pose(moved, "right_arm")
        np.testing.assert_allclose(delta, 0, atol=1e-10)
        np.testing.assert_allclose(rotation, 0, atol=1e-10)
        # Quaternion sign flips represent the same physical orientation.
        moved[3:] = [-value for value in moved[3:]]
        _, rotation = obj._process_xr_pose(moved, "right_arm")
        np.testing.assert_allclose(rotation, 0, atol=1e-10)

    def test_grasp_pose_twist_reaches_ik_and_reanchors_after_release(self):
        from robot_core.model import RobotModel
        from quest_teleop.presets import DEFAULT_GRASP_READY_JOINTS
        grasp = RobotModel().fk(DEFAULT_GRASP_READY_JOINTS)
        for mode, tool_axis in (("tool", [1, 0, 0]), ("aligned", [0, 0, 1])):
            obj = self.controller(90, mode)
            obj.manipulator_config = {"right_arm": {"control_trigger": "right_grip",
                                                   "pose_source": "right_controller", "link_name": "link6"}}
            obj.active = {}
            obj.ref_ee_xyz = {"right_arm": None}
            obj.effector_control_mode = {"right_arm": "pose"}
            obj.effector_task = {"right_arm": SimpleNamespace(T_world_frame=grasp.copy())}
            obj.placo_robot = Mock()
            obj.solver = Mock()
            obj._update_robot_state = Mock()
            obj._update_motion_tracker_tasks = Mock()
            obj._get_link_pose = Mock(return_value=(grasp[:3, 3].copy(), tf.quaternion_from_matrix(grasp)))
            obj.xr_client = Mock()
            obj.xr_client.get_key_value_by_name.return_value = 1.
            initial = tf.quaternion_from_euler(.2, -.3, .4)
            obj.xr_client.get_pose_by_name.return_value = self.pose(initial)
            obj._update_ik()
            np.testing.assert_allclose(obj.effector_task["right_arm"].T_world_frame, grasp, atol=1e-8)
            current = tf.quaternion_multiply(initial, tf.quaternion_about_axis(.2, [0, 0, -1]))
            obj.xr_client.get_pose_by_name.return_value = self.pose(current)
            obj._update_ik()
            expected = grasp @ tf.rotation_matrix(.2, tool_axis)
            np.testing.assert_allclose(obj.effector_task["right_arm"].T_world_frame, expected, atol=1e-8)
            obj.xr_client.get_key_value_by_name.return_value = 0.
            obj._update_ik()
            obj._get_link_pose.return_value = (expected[:3, 3].copy(), tf.quaternion_from_matrix(expected))
            obj.xr_client.get_key_value_by_name.return_value = 1.
            obj._update_ik()
            np.testing.assert_allclose(obj.effector_task["right_arm"].T_world_frame, expected, atol=1e-8)
