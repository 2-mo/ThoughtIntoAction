# RM65-BI 与 EG2-4C2 接口和夹爪排查

调研日期 2026-11-26。适用于当前 RM65-BI 双臂及因时 EG2-4C2。单臂配置也可按同样方法检查。

## 当前判断

- **松手后张开慢已有软件侧证据。** 右臂遥测有 7 段在新张开目标已经到达时仍等待未完成的闭合段，其中 6 段约 0.4–1.1 秒。驱动约每 80 ms 查询夹爪，运动中保留已接受的目标，待其完成后才采用最新目标。机械臂进入保持也可能通过 `set_arm_stop` 打断夹爪段，需检查停止请求和确认时间。参见 [驱动](../../../src/embodied/devices/robots/realman/hardware.py) 和现场记录 `outputs/runtime/control/20261126-143924-robot-abe9aed02a77/tuning.jsonl`。
- **小物体夹不牢需要区分力与间隙。** 软件发送范围为 1–1000，目标附近容许 5 个原始单位，按产品名义总行程 70 mm 折算约 0.35 mm。右臂在全闭目标下已多次报告 `actpos=0/1`。应测量现有指尖空夹的实际间隙及小物体厚度。因时给出的 0–20 N 为产品规格，现场夹持力仍需实测。[因时规格](https://www.inspire-robots.com/dexterous%20hands/eg2-4c-series/) · [项目机型记录](../../realman/robot18.md)
- **位置闭合有控制器力阈值。** 手册写明达到目标位置或超过力阈值后停止。位置命令没有力字段，手册未公布该模式默认阈值或专门的读回字段。维护工具的单次力控夹取默认参数为 50，属协议范围 50–1000，量纲未定义。手册将状态中的 `current_force` 标作当前压力，单位 g。[EG2-4C2 专用协议 PDF 对应版本](https://api.github.com/repos/RealManRobot/Dev_Center/git/blobs/cf1910b79a039523d3a79da46cd663f3f063f52f)

## 底层接口

机械臂控制器使用 TCP JSON，每条消息以 CRLF 结束。项目的现场连接记录使用控制器端口 8080。EG2 专用命令通过 RM65 控制器转发，睿尔曼通用末端页面明确要求该型号使用旧版适配资料。通过官方示教器或现有维护入口分时操作同一控制器，避免和遥操同时写入。[JSON 快速开始](https://develop.realman-robotics.com/robot/json/getStartedJson/) · [型号适配入口](https://develop.realman-robotics.com/robot/json/endeffector/)

| 层 | 命令 | 诊断用途与当前接入 |
| --- | --- | --- |
| RM65 状态 | `get_arm_software_info`、`get_current_arm_state` | 记录软件版本、实测关节和 TCP、故障。左右臂分别留档。 |
| RM65 几何和速度 | `get_DH_data`、`get_joint_min_pos`、`get_joint_max_pos`、`get_joint_max_speed`、`get_current_tool_frame`、`get_current_work_frame` | 检查左右配置、工作系、工具系或限位差异。项目已有查询入口。 |
| RM65 运动 | `movej_canfd`、`movep_canfd`、`set_arm_stop` | 关节透传、末端透传、停止运动。当前 RM65 内核用本地逆解配合关节透传。排查夹爪延迟时记录停止命令及其后反馈。 |
| EG2 状态 | `get_gripper_state` | 读取 `enable/status/error/mode/current_force/temperature/actpos`。模式 4/5 为运动，6 对应力控停止。 |
| EG2 位置 | `set_gripper_position` | 参数 `position` 为原始值 1–1000，`block` 选择阻塞方式。当前遥操常规接口。 |
| EG2 张开 | `set_gripper_release` | 参数 `speed` 为 1–1000，项目维护命令可测试整程张开。 |
| EG2 单次力控 | `set_gripper_pick` | 参数 `speed/force/block`。适合对比位置闭合与显式力阈值。项目已有入口，默认力参数为 50，可通过 `--gripper-force` 显式指定。 |
| EG2 持续力控 | `set_gripper_pick_on` | 压力回落时继续夹取。厂家文档提供，当前项目驱动未接入。采用前需在本机固件上验证释放和停止行为。 |
| EG2 行程配置 | `set_gripper_route` | 写入行程端点。现有驱动未接入，诊断先核实指尖间隙和示教器中的行程设置。 |

运动参数和应答以对应固件版本为准。非阻塞应答意味着控制器接受命令，随后通过位置、模式和物体是否滑动判断结果。[RM65 状态](https://develop.realman-robotics.com/robot/json/armState/) · [RM65 运动协议](https://develop.realman-robotics.com/robot/json/motionConfig/) · [EG2 专用协议目录](https://github.com/RealManRobot/Dev_Center/tree/main/RobotGen3/legacyversions/zh/json) · [项目协议白名单](../../../src/embodied/devices/robots/realman/protocol.py)

## 现场复现与判定

在仓库根目录运行以下只读命令。默认连接配置对应右臂 .18。要测左臂，用 `--config` 指定对应 .19 的连接配置，先确认文件内的地址。记录固件、模式、原始开口和压力反馈。

```bash
cd /home/guest/workspace/ThoughtIntoAction
./run.sh realman status
./run.sh realman gripper-status
./run.sh realman check-kinematics
```

单独做维护开合时，先结束采集和遥操，让该控制器只接受维护入口的命令。下列命令会发出真实开合动作。先在固定机械臂姿态和足够开合空间内做空夹，再在物体有支撑时逐条执行。每次查状态确认动作完成后再继续。

```bash
./run.sh realman gripper-open --execute
./run.sh realman gripper-status
./run.sh realman gripper-close --execute --gripper-force 50
./run.sh realman gripper-status
```

1. **检查实体间隙。** 对左右夹爪分别记录空夹时的最小原始开口和指尖实际间隙。夹同一小物体并记录厚度、接触位置、夹垫形状和滑落前开口。若空夹、夹物都到 0/1 且指尖间隙仍大于物体厚度，先处理夹指几何或行程设置。
2. **做同物体对照。** 在同一臂、同一姿态、同一夹垫上，比较 Quest 全扣的位置闭合与维护工具的单次力控夹取。记录模式、开口、压力随时间变化和是否持住。若维护命令能持住而遥操会滑，优先验证在遥操全扣时接入显式力控。若两者表现一致，结合指尖间隙、接触位置和摩擦再判断本体。
3. **量化松手延迟。** 记录 Trigger 松开或张开目标出现、上一段闭合结束、张开命令接受、开口首次变化四个时点。优先用既有 `command_gripper_target`、`gripper_pending`、`gripper_raw` 和 `gripper_stop_requested_at` 核对。若主要时间耗在旧闭合段，优先验证该固件能否安全接受反向张开指令，再改调度。若主要时间在张开命令接受之后，查设备速度设置、负载和控制器端响应。用多轮的中位值及最大值比较修复前后。

## 外部案例

- ROS 2 的并联夹爪控制器有一个已合并修复，将停滞计时状态放入线程安全容器。它展示了把「运动停滞」追踪到反馈时间与控制器内部状态的方法。这里对应的检查是把 `gripper_pending` 的接受时间和新反馈对齐。[ros2_controllers PR #1855](https://github.com/ros-controls/ros2_controllers/pull/1855)
- MoveIt 用户沿着夹爪目标、关节状态和电机位置逐层检查，发现仿真能动，实机却没有得到预期电机位移。该报告提出检查传动换算和硬件接口，公开内容不足以确认最终修复。这里可用相同方法检查 Trigger、目标原始值、`actpos` 和实际指尖间隙。[MoveIt 2 issue #3518](https://github.com/moveit/moveit2/issues/3518)
- 以 EG2-4C2 型号和协议命令检索公开 GitHub 问题，没有找到能确认与本机一致的真机夹持力或反向延迟故障。现场结论以同一物体的对照结果为准。[型号问题检索](https://github.com/search?q=%22EG2-4C2%22&type=issues) · [命令问题检索](https://github.com/search?q=%22set_gripper_pick%22&type=issues)
