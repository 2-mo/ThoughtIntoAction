"""读取状态与能力，不申请控制权。"""
from embodied import Robot
from ..common import run_example


def run(robot: Robot):
    """关节用 rad、rad/s；TCP 用 xyz 米 + xyzw 四元数；夹爪宽度用米。"""
    state = robot.state
    return {
        "feedback": {key: state[key] for key in ("q", "dq", "pose", "gripper_width")},
        "motion_limits": robot.describe()["motion_limits"],
    }


def main(argv=None):
    return run_example("read_state", run, argv)


if __name__ == "__main__":
    raise SystemExit(main())
