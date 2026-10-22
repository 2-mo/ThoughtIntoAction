# 从例子学习 Robot 接口

先运行可视化仿真控制，按终端提示逐步执行关节、XYZ 和夹爪动作：

```bash
./run.sh examples sim_control
```

它会打开 MuJoCo 窗口，并在每一步前等待按 Enter；无窗口、无需交互运行使用 `--headless`。
完整程序见 [simulation/control.py](simulation/control.py)。窗口负责观察，动作统一通过 `robot` 控制。
原生窗口需要桌面 `DISPLAY`；关闭窗口不会停止脚本，在终端按 Ctrl+C 或最后一步按 Enter 退出本次仿真。

常用操作直接写在 `robot` 上，默认等待动作完成。单项关节示例见 [movej.py](control/movej.py)：

```python
from embodied import Robot

with Robot.sim(viewer=True, speed=.5) as robot:
    q = robot.q
    q[0] += .1
    robot.movej(q)
    robot.gripper.open()
```

这段代码启动带窗口的仿真，让第一个关节增加 0.1 rad，再打开夹爪，结束后关闭本次仿真。
省略 `viewer=True` 则不打开窗口，适合批量验证。

## 仿真与真机：同一套动作，明确选择连接

| Python 入口 | 连接行为 | 退出 `with` 后 |
| --- | --- | --- |
| `Robot.sim(viewer=True)` | 新建 MuJoCo 仿真，并打开观察窗口 | 关闭本次仿真与窗口 |
| `Robot.sim()` | 新建无窗口 MuJoCo 仿真 | 关闭本次仿真 |
| `Robot.sim(config="configs/robots/reference7/robot.json")` | 新建七轴无夹爪教学仿真 | 关闭本次仿真 |
| `Robot.sim(run="outputs/runtime/control/<RUN>")` | 连接已有仿真并核对后端类型 | 保留外部内核 |
| `Robot.real("outputs/runtime/control/<RUN>")` | 连接已经启动的真机内核，核对后端类型 | 保留外部内核 |
| `Robot.connect("outputs/runtime/control/<RUN>")` | 连接已有仿真或真机内核，保留通用连接入口 | 保留外部内核 |

切换到真机只需把 `Robot.sim(...)` 换成 `Robot.real(run, ...)`，`movej`、`movel`、夹爪和采集写法保持相同。
`Robot.real()` 不负责启动或使能硬件；启动内核仍由平台工作台管理。
完整接口、坐标系和更多原语见 [Robot 接口与操作原语](../../docs/reference/robot-primitives.md)。

## 目录与阅读顺序

```text
examples/
├── simulation/              # 可视化、分步仿真控制
├── control/                 # 状态、MoveJ、MoveL、夹爪、记点返回
├── tasks/                   # 接近、退离、二次观测与带验证的抓放模板
├── data/                    # 数据采集、任务事件标注
├── planning/                # 离线轨迹与圆角路径
├── common.py                # 示例命令行参数与输出
├── __main__.py              # python -m examples 统一入口
└── README.md
```

控制示例里的 `run(robot, ...)` 只写动作步骤，便于复制到自己的程序。
真正的封装位于平台 [operations/robot.py](../embodied/applications/operations/robot.py)，不依赖示例目录。

| 文件 | 运行参数 | 学会什么 |
| --- | --- | --- |
| [control.py](simulation/control.py) | `sim_control` / `sim_control --headless` | 在仿真中逐步执行关节、XYZ 和夹爪动作 |
| [task_reset.py](simulation/task_reset.py) | `sim_task` | 保存/恢复物理状态，重复执行任务并检查成功条件 |
| [reference7.py](simulation/reference7.py) | `reference7` | 通过同一 Robot 接口控制独立的七轴无夹爪模型 |
| [read_state.py](control/read_state.py) | `read_state` | 读取 `state`、关节、末端和能力说明 |
| [movej.py](control/movej.py) | `movej --joint 1 --degrees 10` | 从当前关节位置构造目标，再调用 `movej` |
| [movel.py](control/movel.py) | `movel` / `movel --xyz X Y Z` / `movel --offset 0 0 0.02` | 绝对 XYZ、相对平移与完整位姿控制 |
| [move_path.py](control/move_path.py) | `move_path` | 一次执行带圆角的多段路径，使用显式 TCP 速度 |
| [benchmark.py](control/benchmark.py) | `benchmark --degrees 10 --speed 0` | 保存往返动作的全量反馈、速度与跟踪报告；可用 `--baseline` 比较 |
| [gripper.py](control/gripper.py) | `gripper --width 0.04` | 用夹爪对象控制 40 mm 开口 |
| [grasp.py](control/grasp.py) | `grasp --effort <幅值>` | 按设备声明的 effort 单位判断真实接触，空闭合不算抓住 |
| [teach_return.py](control/teach_return.py) | `teach_return` | 记住当前点位，移动，再返回 |
| [approach_retreat.py](tasks/approach_retreat.py) | `approach_retreat` | 使用同一 Robot 接口组合接近/退离，记录阶段结果 |
| [pick_place.py](tasks/pick_place.py) | `pick_place` | 注入目标观测和抓取验证；默认仿真空抓失败后停止 |
| [record.py](data/record.py) | `record --duration 2` | 采集一段数据，查看记录结果 |
| [record_task.py](data/record_task.py) | `record_task --help` | 同步任务与相机录制，保留 scripted 来源，生成待复核标注和导出预检 |
| [annotate.py](data/annotate.py) | `annotate <EPISODE> <EVENTS> --annotator operator --confidence 0.8` | 将同一运行的任务事件映射到已录制相机帧，保存待复核技能区间 |
| [offline_trajectory.py](planning/offline_trajectory.py) | `offline_trajectory` | 离线生成 Ruckig 轨迹，读取 q / dq / ddq |
| [cartesian_blend.py](planning/cartesian_blend.py) | `cartesian_blend --radius 0.025` | 离线生成直线与圆角几何路径 |

## 运行示例

在仓库根目录使用平台 `.venv`；仿真还需要内核 `outputs/envs/control`。
环境准备见[安装说明](../../docs/guides/setup.md)。

```bash
# 列出示例
./run.sh examples

# 自动启动仿真并执行 MoveJ
./run.sh examples movej

# 带窗口执行；动作完成后按 Enter 关闭本次仿真
./run.sh examples movej --backend sim --viewer --keep-open

# 带窗口查看 XYZ 运动
./run.sh examples movel --viewer --keep-open

# 只看计划；speed=0 使用后端每轴速度上限
./run.sh examples movej --preview-only --speed 0

# 查看参数；也能用 python -m examples.control.movej 直接运行模块
./run.sh examples movej --help
```

控制和采集示例默认 `--backend sim`，省略 `--run` 时自动新建仿真。
`--viewer` 仅用于新建仿真；`--keep-open` 在交互终端中执行后等待 Enter，便于查看结果。
两个离线规划示例不启动内核。
`sim_task` 仅支持仿真，演示场景快照、重置和观测成功判据；它不把动作结束直接当作任务成功。
`pick_place` 默认演示没有物体时的失败分支，结果为 `failed`，不会继续抬升或放置。
抓放需要明确的闭合 `effort` 幅值；实际接触成立后才继续，`verify_grasp` 可追加视觉验证。ARX 的电机力矩与仿真指端力单位不同，必须先读取设备能力；不把电机力矩直接当作指端力。
示例数据保存在 `outputs/examples/`，支持输出路径的例子可用 `--output` 指定位置；仿真日志保存在 `outputs/robot/runs/`。

要使用已经运行的内核，显式选择后端和运行目录：

```bash
# 连接已有仿真
./run.sh examples movej --backend sim --run outputs/runtime/control/<SIM_RUN>

# 连接已有真机：先读取状态，再按需要执行同一份动作示例
./run.sh examples read_state --backend real --run outputs/runtime/control/<REAL_RUN>
./run.sh examples movej --backend real --run outputs/runtime/control/<REAL_RUN> --degrees 10
```

`--backend real` 必须同时传 `--run`；目录中的后端必须与 `--backend` 一致，否则报错。
因此，仅加 `--run` 不会把默认仿真命令切换到真机。连接已有内核的示例退出时不关闭该内核。

## 常用动作怎么写

```python
with Robot.sim(speed=.5) as robot:
    start = robot.teach()
    robot.jog(joint=0, delta=.1)
    robot.return_to(start)

    x, y, z = robot.xyz
    robot.movel(xyz=[x, y, z + .02])       # 绝对基座位置，保持朝向
    robot.movel(offset=[0, 0, .02], frame="tool")
    robot.gripper.move(.04)
```

`teach()` 记录当前位置，不是自由拖动模式。`movej`、`movel` 默认等待完成，失败会报告错误。
需要先检查再执行时，使用 `plan = robot.plan_movej(q)` 和 `robot.execute(plan)`；
需要非阻塞调用时，使用 `motion = robot.movej(q, wait=False)`，然后 `motion.wait()`。

`record.py` 只采集，不触发运动；默认记录无相机诊断数据。
视觉训练数据需要另行配置相机；支持只有外部相机的录制，不要求双相机。
`record_task` 连接显式 `--run`，执行任务并录制指定相机；`--camera-roles` 把设备 ID 映射到策略图像角色。
例如在仓库根目录，对已经运行的仿真内核和已配置的 `front_rgb` 相机执行：

```bash
./run.sh examples record_task --run outputs/runtime/control/<SIM_RUN> \
  --cameras front_rgb --camera-roles '{"base_0_rgb":"front_rgb"}' \
  --annotator operator --confidence .8
```

录制目录含 `task/events.jsonl`、`task/result.json`、相机数据、`capture-status.json` 和技能标注。
程序示范与人工示范通过 provenance 区分；未给出场景成功判据时，任务执行完成后的 outcome 仍为 unknown。
`annotate` 只处理已结束、带相机的录制，需要同一次内核运行的 `tasks/<TASK_ID>/events.jsonl`；
不会从无相机的 `record` 诊断数据生成帧标签。完整工作流见[采集说明](../../docs/guides/recording.md)。

## 名称、单位与当前边界

| 名称 | 含义 |
| --- | --- |
| MoveJ | 输入关节目标，在关节空间规划；末端路径不保证直线 |
| MoveL | 输入绝对 XYZ、完整位姿或平移偏移，规划直线路径 |
| `move_path` | 多点与圆角组成一条连续轨迹；不承诺完整 UR MoveP 的恒定工艺速度语义 |

`q` 用 rad，长度与机器人关节数一致；X5 为六轴，Reference7 为七轴且不带夹爪。
`robot.xyz` 为基座系位置（m），`robot.pose` 始终返回 `[x, y, z, qx, qy, qz, qw]`。
`movel(pose)` 同时接受 UR 风格六值 `[x, y, z, rx, ry, rz]`（旋转向量 rad，**不是 RPY**）和七值 XYZ + **XYZW**；两种输入共用同一个 MoveL 实现。
`--xyz`、`--offset`、`--pose` 三选一；绝对 XYZ / 位姿仅使用 `frame="base"`。
Python 关节编号从 0 开始；示例命令行的 `--joint 1` 表示第一个关节。
`offset` 用 m，`frame="base"` 沿基座轴移动，`frame="tool"` 沿当前工具轴移动。

`speed` / `acceleration` / `jerk` 是关节约束，单位为 rad/s、rad/s²、rad/s³；
**MoveL 的 speed 当前也不是末端 m/s**。运动示例默认使用 `.5 / 2 / 10`。
`speed=0` 使用后端每轴上限；离线轨迹没有后端，速度必须为正数。

MoveL 和 `move_path` 的末端约束使用 `tcp_speed/tcp_acceleration/tcp_jerk`（m/s、m/s²、m/s³）；
旋转约束使用 `angular_speed/angular_acceleration/angular_jerk`（rad/s、rad/s²、rad/s³）。
这些约束和设备关节限值同时生效，计划指标与限制原因可直接查看。

MoveL 小步运动仍有已知局限：此前离线检查中，仿真零姿态的 +Z 20 mm 约 0.888 s，
而 +Z 5 mm 约 15.786 s；IK 离散误差可能放大 jerk 估计。示例保留真实规划指标，
这些时长不是实机测量。逐段 MoveJ / MoveL 也不等于连续圆角 MoveP。

更多原语、任务技能边界与新接口参考见 [Robot 接口与操作原语](../../docs/reference/robot-primitives.md)。

## 开源参考

示例组织还参考了以下官方材料；本目录调用本项目接口，没有引入这些机器人的 SDK。

- [UR MoveJ / MoveL / MoveP](https://www.universal-robots.com/manuals/EN/HTML/SW5_19/Content/prod-usr-man/complianceUR5e/SW_sections/first_program/move.htm)：运动空间与路径语义。
- [ur_rtde Quick Start](https://sdurobotics.gitlab.io/ur_rtde/pages/getting_started/quick_start.html)：读状态、移动、返回的完整短例。
- [xArm-Python-SDK examples](https://github.com/xArm-Developer/xArm-Python-SDK/tree/master/example/wrapper)：按功能分文件。
- [Ruckig v0.12.2 离线示例](https://github.com/pantor/ruckig/blob/v0.12.2/examples/02_position_offline.py)：轨迹约束、时间与采样。
- [MoveIt2 Python 教程](https://moveit.picknik.ai/main/doc/examples/motion_planning_python_api/motion_planning_python_api_tutorial.html)：规划与执行分开。
- [ur_rtde Record Data](https://sdurobotics.gitlab.io/ur_rtde/pages/examples/basic_motion/record_data_example.html)：采集单独成例。
