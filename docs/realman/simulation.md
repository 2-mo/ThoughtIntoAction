# RM65 MuJoCo 仿真

已接入统一平台 `realman.sim`，可通过 `Robot.sim(config=...)` 使用 MoveJ、MoveL、简化夹爪以及仿真快照/恢复/复位。入口创建并管理自己的仿真内核，退出时释放控制权并关闭该内核。仿真配置没有硬件插件，代码不连接 `.18` / `.19`。

## 先运行无 VR 演示

工作目录 `/home/guest/下载/ThoughtIntoAction-local`；使用现有平台 `.venv` 和 `outputs/envs/control`，不需要 ROS、机械臂或头显。

```bash
# 本机桌面：打开 MuJoCo 窗口，在终端按回车逐步执行
./run.sh examples realman

# SSH / 无窗口：连续执行后退出
./run.sh examples realman --headless
```

演示依次执行：关节 1 转动 0.1 rad → TCP 沿基座 X 轴移动 10 mm → 简化夹爪从 40 mm 闭合到 20 mm，再张开 → 恢复开始快照 → 复位。末尾打印 `moved_q`、`restored_q` 和 `restored_gripper_width_m`。选择 X 方向是因为演示起始姿态接近向上伸展边界，保持姿态向上移动 10 mm 的路径不可达；程序保留不可达检查。

窗口只显示反馈，动作从终端执行。运行目录和日志打印在终端，位于 `outputs/runtime/control/<RUN>/`；脚本内核启动日志位于 `outputs/robot/runs/sim-*/runtime.log`。

自己的脚本沿用公共接口，使用前先让新启动内核积累约 0.5 秒静止反馈：

```python
import time
from embodied import Robot

with Robot.sim(config="configs/robots/realman/sim.json", viewer=True) as robot:
    time.sleep(.5)
    initial = robot.simulation.snapshot()
    robot.movel(offset=[.005, 0, 0], speed=.2)
    robot.gripper.move(.02, timeout=5)
    robot.hold()  # 结束夹爪流指令，才能恢复场景
    robot.simulation.restore(initial)
```

如需持续运行内核、另开终端使用现有监看与 motion 命令：

```bash
./run.sh kernel start --backend sim --input idle --managed --headless --config configs/robots/realman/sim.json
# 另一终端；RUN 使用上条命令打印的运行目录
./run.sh motion run --run <RUN> status
./run.sh web monitor --kernel --no-camera --no-connections
```

统一网页通过运行时 URDF 显示六轴机械臂和工具掌部；动态夹指显示在 MuJoCo 桌面窗口。网页和数据契约中的夹爪开口数值来自两个仿真夹指的位置之和。

## 官方模型与验证依据

模型来自 RealMan 官方 [ros2_rm_robot](https://github.com/RealManRobot/ros2_rm_robot/tree/711abedd48c6614f26fb4f7661db92f306ff05bd)，固定提交 `711abedd48c6614f26fb4f7661db92f306ff05bd` 的 `rm_description/urdf/rm_65.urdf` 和七个 STL。原始文件、Apache-2.0 许可证、来源及校验值保存在 `assets/robots/realman/`。构建脚本保留官方关节变换、轴向、质量、惯性和角度范围，生成可移植的 MJCF 与六轴显示 URDF。

部署参数集中在 `configs/robots/realman/sim.json`：

- TCP 是现场 `Lq` 配置的法兰 Z +140 mm；这是采用的工具配置，不是实测标定。
- 起始关节角取自先前记录的现场姿态，启动时不读取机械臂。
- 100 Hz 仿真控制、2 ms 物理步长；PD 加模型偏置补偿，0.5 rad/s 关节目标速度上限。增益、阻尼、armature、速度限制是工程仿真设置，不能代表真实控制器的透传、延迟或跟踪性能。
- 使用名义 URDF，未导入实际机械臂的 DH 校准。官方 URDF 自身的 RPY 数值有有限精度。

可与已有锁版本 `realman-api` 的官方 `Algo(RM65, B)` 做离线 FK 对照：

```bash
./run.sh examples realman --check-kinematics
```

该检查不启动仿真服务、不初始化 XR、不连接机械臂。若该可选依赖未准备好，按现有依赖入口安装：`./run.sh fetch-dependencies --name realman-api`。报告保存为 `outputs/realman/sim-kinematics.json`，记录模型标识、算法版本、随机种子、逐样本关节角、双方位姿和误差。开发验证使用零位、起始姿态及种子 42 的 32 组随机角，共 34 组；最大位置差约 5.3 微米，最大姿态差约 0.000014 rad。这证明两种名义运动学模型对齐，不代表实机标定或动力学验证。

源文件或工具长度/假定夹爪范围变化后，在仓库根目录重新生成：

```bash
outputs/envs/control/bin/python scripts/realman/build_model.py
```

运行时校验模型、来源记录、网格及工具参数；模型标识和有效配置写入内核运行记录。快照同时绑定模型、MuJoCo 版本和配置，错误身份、非有限状态、越界状态会被拒绝，拒绝时不改写当前场景。

## 简化夹爪边界

桌面窗口中的双指是为控制流程自建的平行滑块，**不是 EG2-4C2 官方几何或标定模型**。假定开口 0–40 mm、总质量 0.5 kg（掌部 0.46 kg + 双指各 0.02 kg）、开口目标速度 20 mm/s。原始控制器 `actpos` 没有被换算为毫米。

当前支持位置开合，`grasp=false`，不输出伪造的力或接触反馈。机器人接触关闭，尚不支持碰撞避让、物体抓取或真实夹持力判断。TCP 平面限制仍由平台 gateway 执行，但不能替代碰撞检测。

## 头显充电后接入 Quest

```bash
# 打开自己的 RM65 仿真窗口，并接入 Quest；默认运行 30 秒
./run.sh examples realman --quest
# 无窗口运行，直到 B / Ctrl+C
./run.sh examples realman --quest --headless --duration 0
```

XRoboToolkit 的主机服务、USB 转发和头显客户端仍沿用现有准备流程。先松开 Grip/A/Trigger；Grip 按住跟随，松开 Grip 后按住 A 张开、Trigger 闭合，B 退出。重用真机入口的相对位姿映射、松手解锁和 XR 新鲜度规则；从 `.19` 配置仅读取映射字段，仿真输入速度显式使用 `sim.json` 的 `teleop_linear_speed_m_s=0.125`、`teleop_angular_speed_rad_s=0.30`，不查询或连接真实机器人。

Quest 输入持有统一 gateway 的显式控制租约，数据过期或异常会进入保持并释放租约。这里验证的是映射和平台输入链路；没有复刻真机的 `movep_canfd`、控制器 IK、网络延迟或真实 EG2 分段执行。

当前已实际完成无窗口 MuJoCo 演示、离线 FK 对照和模型截图；Quest 松手解锁、方向/速度、夹爪和数据停滞退出使用合成 SDK 输入回归，真实头显操作待充电后现场验证。
