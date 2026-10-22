"""同一初始场景重复执行关节到位任务，使用真实仿真反馈判断成功。"""
from embodied import Robot
from ..common import run_example


def run(robot: Robot):
    """保存场景 → MoveJ → 判断成功 → 恢复场景；仅支持仿真。"""
    scene = robot.simulation
    initial = scene.snapshot()  # 真机在此处拒绝，不会执行后续动作。
    goal = list(initial['observation']['q'])
    goal[0] += .1
    task = scene.task('reach-joint-target', instruction='第一个关节增加 0.1 rad',
        goal='所有关节与目标偏差小于 0.01 rad',
        success=lambda obs: all(abs(q - target) < .01 for q, target in zip(obs['q'], goal)))
    outcomes = []
    for _ in range(2):
        task.reset()
        robot.movej(goal, speed=.5)
        outcomes.append(task.check_success())
    scene.restore(initial)
    return {'task': task.name, 'success': outcomes, 'restored': True}


def main(argv=None):
    return run_example('sim_task', run, argv)


if __name__ == '__main__':
    raise SystemExit(main())
