"""一条连续路径：中间转角圆滑过渡，最后才等待停稳。"""
from embodied import Robot
from ..common import run_example


def run(robot: Robot, *, tcp_speed=.1, angular_speed=.5, preview_only=False):
    xyz = robot.xyz
    waypoints = [[xyz[0], xyz[1], xyz[2] + .02],
                 [xyz[0], xyz[1] + .02, xyz[2] + .02]]
    options = dict(blend_radius=.005, tcp_speed=tcp_speed, angular_speed=angular_speed)
    if preview_only:
        return robot.plan_move_path(waypoints, **options)
    return robot.move_path(waypoints, **options)


def main(argv=None):
    return run_example("move_path", run, argv)


if __name__ == "__main__":
    raise SystemExit(main())
