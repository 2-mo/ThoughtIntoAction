"""抓取、放置与二次观测；默认空场景演示失败停止，不假装抓到物体。

在自己的程序中传入 observe(robot) -> Observation 和
verify_grasp(robot) -> GraspCheck 即可接真实识别器及接触/视觉验证器。
观测输出为基座系 TCP 位姿；同一目标的 target_id 必须保持一致。
"""
from embodied import Robot
from embodied.applications.operations.tasks import GraspCheck, GraspUnconfirmed, Observation
from ..common import run_example


def run(robot: Robot, *, observe=None, verify_grasp=None):
    """默认仿真固定目标、空抓失败；注入真实观测和验证器后执行完整任务。"""
    empty_scene = verify_grasp is None
    if robot.backend != "sim" and (observe is None or verify_grasp is None):
        raise ValueError("真机需传入真实 observe 和 verify_grasp；示例固定目标仅用于仿真")
    fixture = Observation(robot.pose, "example-target", {"source": "fixed simulation example"})
    observe = observe or (lambda robot: fixture)
    verify_grasp = verify_grasp or (lambda robot: GraspCheck(False, "空场景没有物体接触证据"))
    destination = robot.xyz
    destination[0] += .01
    width = robot.describe()["robot"]["gripper_max_width"] / 2
    try:
        with robot.task("visual pick and place") as task:
            task.pick(observe=observe, width=width, verify_grasp=verify_grasp,
                      approach_offset=[0., 0., .02])
            task.place(xyz=destination, approach_offset=[0., 0., .02])
    except GraspUnconfirmed:
        if not empty_scene:
            raise
        # 预期的空抓演示：result 明确 failed，后面的 lift / place 均未执行。
    return task.result


def main(argv=None):
    return run_example("pick_place", run, argv)


if __name__ == "__main__":
    raise SystemExit(main())
