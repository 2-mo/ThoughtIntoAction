"""MoveL：绝对 XYZ、相对偏移或完整位姿；位置用 m。"""
from embodied import Robot
from ..common import run_example


def run(robot: Robot, *, xyz=None, offset=None, pose=None, frame="base", tcp_speed=None, angular_speed=None, preview_only=False):
    """默认构造当前点上方 20 mm 的 XYZ 目标，保持朝向。"""
    if xyz is None and offset is None and pose is None:
        if frame == "tool":
            offset = [0., 0., .02]
        else:
            xyz = robot.xyz
            xyz[2] += .02
    if preview_only:
        return robot.plan_movel(pose, xyz=xyz, offset=offset, frame=frame, tcp_speed=tcp_speed, angular_speed=angular_speed)
    # TCP 与关节限制分别设置，规划同时满足两者。
    return robot.movel(pose, xyz=xyz, offset=offset, frame=frame, tcp_speed=tcp_speed, angular_speed=angular_speed)


def main(argv=None):
    return run_example("movel", run, argv)


if __name__ == "__main__":
    raise SystemExit(main())
