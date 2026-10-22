# Robot 接口与操作原语

示例应该写“让机器人做什么”，连接、规划、执行和等待由统一接口处理。
Python 入口为 `from embodied import Robot`，复用项目现有控制内核。

```python
from embodied import Robot

with Robot.sim() as robot:
    q = robot.q.copy()
    q[0] += 0.1
    robot.movej(q)
    robot.gripper.open()
```

`Robot.sim()` 管理本次示例的仿真生命周期；`Robot.real(run)` 连接已启动的真机内核，
其中 `run` 是内核运行目录。仿真与真机使用同一套动作写法，连接方式明确区分。
可运行的完整例子见 [src/examples](../../src/examples/README.md)。

## 选择仿真或真机

| 入口 | 后端与生命周期 |
| --- | --- |
| `Robot.sim()` | 创建无窗口 MuJoCo 仿真；退出时关闭自己创建的内核 |
| `Robot.sim(viewer=True)` | 创建带观察窗口的 MuJoCo 仿真；退出时关闭内核与窗口 |
| `Robot.sim(run=run)` | 连接已有仿真并检查后端类型；退出时保留外部内核 |
| `Robot.real(run)` | 连接已有真机内核并检查后端类型；不启动或使能硬件，退出时保留内核 |
| `Robot.connect(run)` | 通用连接，兼容已有仿真或真机内核；退出时保留内核 |

把运行环境选择放在连接处，动作程序不需要判断后端：

```python
def control(robot):
    robot.jog(joint=0, delta=0.1)
    x, y, z = robot.xyz
    robot.movel(xyz=[x, y, z + 0.02])
    robot.gripper.open()

with Robot.sim(viewer=True) as robot:
    control(robot)

# 使用同一份 control：将连接替换为 with Robot.real(run) as robot:
```

可视化入门运行 `./run.sh examples sim_control`；无窗口批量运行加 `--headless`。
单项示例使用 `./run.sh examples movej --viewer --keep-open`，执行后按 Enter 结束。
窗口用于观察仿真，`movej` / `movel` 等命令仍走统一规划与执行链路；不另设一套仿真动作接口。
原生窗口需要桌面 `DISPLAY`。关闭窗口只关闭显示，脚本继续运行；在终端结束脚本或退出 `with` 才关闭它创建的仿真。

命令行默认 `--backend sim`；连接已有内核时检查 `--backend` 与实际后端一致。
真机示例必须显式给出 `--backend real --run outputs/runtime/control/<RUN>`，
`--viewer` 仅适用于自动创建的仿真。当前仿真与真机后端均使用 X5 模型，其他机械臂仍需适配。

## 可直接使用的接口

下面是易用接口覆盖的范围，不代表项目具备表格以外的任务能力。

| 类别 | 写法 | 含义 |
| --- | --- | --- |
| 观察 | `robot.state` | 当前反馈与运行状态 |
| 后端 | `robot.backend` | 已核对的运行类型：`sim` 或 `real` |
| 关节 | `robot.q` | 当前关节位置，单位 rad |
| 末端 | `robot.pose` | 当前末端位姿 `[x, y, z, qx, qy, qz, qw]` |
| 位置 | `robot.xyz` | 当前基座坐标系 XYZ，单位 m |
| 能力 | `robot.describe()` | 模型、工作空间和运动限值 |
| 关节运动 | `robot.movej(q)` | 规划到目标关节位置并执行 |
| 直线运动 | `robot.movel(pose)` | 规划末端直线路径并执行 |
| XYZ 控制 | `robot.movel(xyz=[x, y, z])` | 到绝对 XYZ 位置，保持当前朝向 |
| 相对移动 | `robot.movel(offset=[x, y, z], frame="base")` | 相对当前位置平移，保持末端朝向 |
| 关节点动 | `robot.jog(joint=0, delta=0.1)` | 第一个关节相对移动 0.1 rad |
| 回零 | `robot.home()` | 执行内核定义的回零动作 |
| 夹爪 | `robot.gripper.move(width)` | 设置夹爪开口，单位 m |
| 开合 | `robot.gripper.open()` / `.close()` | 打开或闭合夹爪 |
| 记点 | `point = robot.teach()` | 保存当前点位 |
| 返回 | `robot.return_to(point)` | 运动到已保存点位 |
| 保持 | `robot.hold()` | 请求内核保持 |
| 取消 | `robot.cancel()` | 取消当前执行的动作 |
| 采集 | `robot.record(duration=3, output=path, task="观察")` | 按给定时长采集，并附任务说明 |

关节编号从 0 开始；`q` 的长度与连接后的关节反馈一致，单位 rad。当前内核为六轴 X5；公共接口去掉固定轴数并不表示其他型号已完成适配。
位置、偏移和夹爪宽度用 m；`robot.pose` 的反馈始终为 `xyz + xyzw` 七值，避免反馈格式随调用改变。
`teach()` 是记录点位，不是打开拖动示教模式；此接口暂不提供 `free_drive()`。
夹爪动作完成表示开合指令完成，不表示已经抓住物体。

## 默认直接执行，也能先看计划

`movej`、`movel` 等动作默认等待完成，因此顺序代码就是顺序动作。
等待期间检查内核返回的执行状态；规划失败、执行失败或超时会报告错误。

```python
with Robot.sim() as robot:
    start = robot.teach()
    robot.jog(joint=0, delta=0.1)
    robot.return_to(start)
```

需要检查计划时，使用 `plan_movej()` 或 `plan_movel()`，然后显式执行：

```python
with Robot.sim() as robot:
    target = robot.q.copy()
    target[0] += 0.1
    plan = robot.plan_movej(target)
    print(plan["metrics"])
    robot.execute(plan)
```

需要同时处理界面或其他工作时，保留动作对象，再等待其完成：

```python
with Robot.sim() as robot:
    target = robot.q.copy()
    target[0] += 0.1
    motion = robot.movej(target, wait=False)
    print(robot.state)
    motion.wait()
```

`wait=False` 仅改变调用方是否阻塞，不表示可以重叠提交任意多个动作。
MoveJ 和 MoveL 的 `speed`、`acceleration`、`jerk` 当前均约束关节轨迹，
单位分别为 rad/s、rad/s²、rad/s³；不能把 MoveL 的 `speed` 当成末端 m/s。
`speed=0` 使用后端报告的每轴速度上限，仍由轨迹规划器生成连续运动。

## 关节、XYZ 与 UR 风格位姿

借鉴 UR 的 MoveJ / MoveL 分工：关节目标写 `movej(q)`，空间直线目标写 `movel(...)`。
常见的 XYZ 控制无需自己拼四元数：

```python
with Robot.sim() as robot:
    q = robot.q
    q[0] += 0.1
    robot.movej(q)

    x, y, z = robot.xyz
    robot.movel(xyz=[x, y, z + 0.02])  # 绝对 XYZ，保持朝向
```

需要指定朝向时，`movel(pose)` 接受两种明确长度的格式，统一转换为内核的 XYZW 表示：

```python
robot.movel([0.3, 0.1, 0.25, 0, 0, 1.57])       # xyz + 旋转向量，m / rad
robot.movel([0.3, 0.1, 0.25, 0, 0, 0, 1])       # xyz + XYZW 四元数
```

前者沿用 [UR 轴角旋转向量](https://www.universal-robots.com/articles/ur/application-installation/explanation-on-robot-orientation/)，不是 RPY 欧拉角；目标必须在所接机器人工作空间内。
`pose`、`xyz`、`offset` 三选一。这里统一目标表达，不宣称兼容全部 URScript：
[UR MoveL 的速度为 m/s](https://www.universal-robots.com/manuals/EN/HTML/SW5_25/Content/prod-scriptmanual/all_scripts/movel_posea12v025t.htm)，
本项目现有 `speed` 仍为关节限值，不增加含义不一致的 `v` / `a` 别名。

## 相对移动与坐标系

相同的偏移量，在不同坐标系下具有不同方向：

```python
robot.movel(offset=[0, 0, 0.02], frame="base")  # 沿基座 Z 轴移动 2 cm
robot.movel(offset=[0, 0, 0.02], frame="tool")  # 沿当前工具 Z 轴移动 2 cm
```

这里的 `offset` 只表示三维平移，保持原有朝向。
绝对位姿 `robot.movel(pose)` 和 `robot.movel(xyz=...)` 使用基座坐标系，不支持 `frame="tool"`。
`approach`、`retreat`、`lift` 可以先用明确坐标系的相对移动表达，
不必为同一个动作增加许多含义相近的接口。

## 运动原语与任务技能的区别

MoveJ、MoveL、夹爪开合回答“运动到哪里”；抓取、放置回答“任务是否完成”。
后者还需要物体位姿、工具与对象的关系，以及可验证的结果。

| 层次 | 例子 | 本项目当前边界 |
| --- | --- | --- |
| 可直接调用的原语 | MoveJ、MoveL、点动、开合、记点返回 | 本轮统一封装已有内核能力 |
| 原语组合 | 接近、抬升、后退 | 用 `movel(offset=..., frame=...)` 编写几何动作 |
| 任务技能 | grasp、pick、place、push、insert、handover | 尚未提供通用实现，需场景与反馈条件 |
| 连续路径 | MoveP、圆角衔接 | 有离线路径示例，尚未接入内核执行 |

抓取任务可以拆成：接近 → 闭爪 → 验证抓持 → 抬升。
没有抓持验证时，不能把“闭爪完成”自动转换成“抓取成功”。
放置也需要确认对象被释放并处于目标位置，不能仅凭开爪结束作判断。
这些是后续技能层的完成条件，本轮不添加空壳 `pick()` / `place()`。

## 参考了哪些优秀设计

下表链接均为项目官方文档或官方仓库；借鉴的是接口与行为设计，
不意味着已经引入这些框架，也不意味着它们的能力已迁移到本项目。

| 项目与一手参考 | 借鉴部分 | 本项目的取舍 |
| --- | --- | --- |
| [UR：MoveJ](https://www.universal-robots.com/manuals/EN/HTML/SW10_10/Content/prod-scriptmanual/all_scripts/movej_qa14v105t0r.htm) / [MoveL](https://www.universal-robots.com/manuals/EN/HTML/SW5_25/Content/prod-scriptmanual/all_scripts/movel_posea12v025t.htm) | 区分关节运动、工具直线运动及明确单位 | 常用名称与位姿输入对齐，底层轨迹由自身内核实现，不混用速度单位 |
| [Genesis：Robot Control](https://genesis-world.readthedocs.io/en/v1.4.0/user_guide/robot_control/index.html) | 围绕机器人对象表达 IK、规划和控制 | 示例围绕 `robot` 写动作，常用执行步骤封装起来 |
| [ManiSkill：Controllers / Action Spaces](https://maniskill.readthedocs.io/en/latest/user_guide/concepts/controllers.html) | 区分绝对目标、增量目标及参考坐标系 | `pose` 与 `offset` 分开，`base` / `tool` 显式指定 |
| [robosuite：Controllers](https://robosuite.ai/docs/modules/controllers.html) | 控制器定义动作含义，机器人按部件组合 | 机械臂运动与 `robot.gripper` 分开，不混淆位置、速度和力矩 |
| [PyRoboSim：Robot Actions](https://pyrobosim.readthedocs.io/en/latest/usage/robot_actions.html) | 动作、计划、执行结果和取消有统一语义 | 简单动作默认等待；复杂调用可持有 `Motion`，失败不会当作完成 |
| [MoveIt Task Constructor：Pick and Place](https://moveit.picknik.ai/main/doc/tutorials/pick_and_place_with_moveit_task_constructor/pick_and_place_with_moveit_task_constructor.html) | 抓取放置分成可检查、可组合的阶段 | 先保留动作原语；任务技能补齐感知与结果验证后再封装 |
| [LeRobot：硬件接口](https://huggingface.co/docs/lerobot/main/integrate_hardware) / [数据集](https://huggingface.co/docs/lerobot/main/lerobot-dataset-v3) | 观察、动作、硬件接口和 episode 数据解耦 | 控制与采集分别调用，保留状态、动作、时间和任务语义 |
| [libfranka：Gripper](https://frankarobotics.github.io/libfranka/0.15.0/classfranka_1_1Gripper.html) | `move` 与带反馈判据的 `grasp` 分开 | 当前只承诺夹爪开合，不伪造抓持检测 |

MTC 的 `attachObject` 修改的是规划场景，不能作为真实抓持的传感反馈。
libfranka 的抓取判据依赖其夹爪行为，也不能未经标定直接套用到 ARX。
后续增加技能时，应让每个阶段有清楚的输入、完成条件和失败结果，
同时保持最外层示例短小、直观。
