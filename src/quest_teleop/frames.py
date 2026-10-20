"""Operator heading and controller-to-X5-tool orientation mapping."""
import math
import numpy as np
import meshcat.transformations as tf

from quest_teleop.target_limits import rotvec


# Quest local forward -Z, right +X, up +Y -> X5 link6 forward +X,
# right -Y, up +Z. This is a tool-axis convention, independent of operator yaw.
R_CONTROLLER_TO_TOOL = np.array([[0., 0., -1.], [-1., 0., 0.], [0., 1., 0.]])

# Horizontal controller / downward X5 gripper reference: twist about -Z
# maps to tool +Z (side tilt), nod about +X to tool -Y (fore/aft tilt),
# and yaw about +Y to tool -X (vertical spin). This is Ry(-90deg) times
# the forward-controller / forward-tool convention above, with det = +1.
R_CONTROLLER_TO_DOWN_TOOL = np.array([[0., -1., 0.], [-1., 0., 0.], [0., 0., -1.]])


def operator_yaw_rotation(degrees):
    """Positive yaw rotates base +X toward +Y; Z is unchanged."""
    if not math.isfinite(degrees):
        raise ValueError("operator_yaw_deg must be finite")
    angle = math.radians(degrees % 360)
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, -s, 0.], [s, c, 0.], [0., 0., 1.]])


def tool_rotation_delta(controller_reference, controller_current, tool_reference):
    """Return a base-frame rotvec aligning local controller/tool axes at Grip.

    All quaternions are wxyz. With controller C, Grip reference C0, tool E0
    and axis map B, the target is E0 B (C0^-1 C) B^-1. The IK loop applies
    a base-frame delta on the left, so express this local delta through E0.
    """
    local_delta = tf.quaternion_multiply(tf.quaternion_inverse(controller_reference), controller_current)
    tool_basis = tf.quaternion_matrix(tool_reference)[:3, :3]
    return tool_basis @ R_CONTROLLER_TO_TOOL @ rotvec(local_delta)


def aligned_rotation_basis(controller_reference, tool_reference):
    """Latch a Quest-world -> base rotation at the chosen reference posture.

    Controller points forward and gripper down, with tool +Z toward the
    operator's forward direction (the nominal GRASP_READY orientation).
    Keep this basis across Grip releases so subsequent wrist/gripper motion
    cannot reassign the world axes. Inputs are wxyz quaternions.
    """
    controller_basis = tf.quaternion_matrix(controller_reference)[:3, :3]
    tool_basis = tf.quaternion_matrix(tool_reference)[:3, :3]
    return tool_basis @ R_CONTROLLER_TO_DOWN_TOOL @ controller_basis.T


def aligned_rotation_delta(controller_reference, controller_current, basis):
    """Map a world-frame delta through the latched alignment (wxyz inputs)."""
    delta = tf.quaternion_multiply(controller_current, tf.quaternion_inverse(controller_reference))
    return basis @ rotvec(delta)


def aligned_position_basis(rotation_basis):
    """Use the calibrated horizontal heading while keeping Quest +Y base +Z.

    Project onto the closest upright rotation so wrist/gripper tilt during
    calibration cannot turn horizontal translation into vertical motion.
    """
    relative = np.asarray(rotation_basis) @ R_CONTROLLER_TO_TOOL.T
    yaw = math.atan2(relative[1, 0] - relative[0, 1], relative[0, 0] + relative[1, 1])
    return operator_yaw_rotation(math.degrees(yaw)) @ R_CONTROLLER_TO_TOOL
