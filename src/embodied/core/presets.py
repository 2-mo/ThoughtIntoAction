"""Shared X5 starting poses, in URDF joint order and radians."""
DEFAULT_HOME_JOINTS = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
DEFAULT_GRASP_READY_JOINTS = (0.0, 1.50, 1.45, -1.52, 0.0, 0.0)


def pregrasp_joints(robot):
    """Resolve a preset from the active model, never from joint count alone."""
    import math
    values = robot.get('pregrasp_q')
    if values is None and robot.get('simulation_plugin') == 'arx.sim':
        values = DEFAULT_GRASP_READY_JOINTS
    if values is None:
        raise ValueError('当前机器人未配置预抓取位姿')
    if (len(values) != len(robot['park_q']) or any(not math.isfinite(v) or not lo <= v <= hi
            for v, lo, hi in zip(values, robot['joint_min'], robot['joint_max']))):
        raise ValueError('预抓取位姿与当前机器人限位不符')
    return list(values)
