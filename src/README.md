# 源码结构

平台代码都在 `embodied` 包中；公共控制内核位于 `embodied/control/`，由独立的 Python 3.10 环境运行。按“公共接口、独立能力、设备适配”组织，目标是更换机械臂、相机或控制来源时，继续复用任务程序与数据工作流。当前统一内核实现的是 ARX X5 六轴真机与 MuJoCo，其他硬件仍需具体适配。

## 先找入口

| 要做什么 | 使用入口 | 找实现 |
| --- | --- | --- |
| 启动工作台、查看服务 | `./run.sh`、`./run.sh help` | [applications/workstation](embodied/applications/workstation/) |
| 写 MoveJ / MoveL、夹爪、记点返回 | `from embodied import Robot` · [控制示例](examples/README.md) | [operations/robot.py](embodied/applications/operations/robot.py) |
| 改运动规划和时间参数化 | [离线规划示例](examples/planning/) | [motion](embodied/motion/)；内核接入在 [control/commands.py](embodied/control/commands.py) |
| 接机械臂、相机或新控制来源 | [现有接口](embodied/contracts/interfaces.py) · [接入边界](../docs/architecture/overview.md#替换组件时改哪里) | [devices](embodied/devices/)、[ARX 适配](embodied/devices/robots/arx5/) |
| 采集、回放、导出数据 | [采集示例](examples/data/record.py)、`./run.sh help data` | [data](embodied/data/) |
| 接入策略或标定流程 | `./run.sh help policy`、`./run.sh help calib` | [policy](embodied/policy/)、[applications/calibration](embodied/applications/calibration/) |

## 代码职责

```text
src/
├── examples/           可运行教学示例：状态、MoveJ / MoveL、夹爪、示教返回、采集、离线轨迹
├── embodied/
│   ├── applications/   面向操作者的应用、工作流与进程装配
│   │   ├── console/    apps/console 的后端
│   │   ├── monitor/    18081 监看页与 MuJoCo 工作台
│   │   ├── workstation/ 工作台装配、内核启动、组件注册与命令
│   │   ├── operations/ Robot 编程接口、MoveJ / MoveL、点动、夹爪、示教点与 CLI
│   │   └── calibration/ 手眼标定：标定板、采样计划、采样与求解
│   ├── control/        公共控制运行时：网关、命令调度、worker、IPC 与 observer
│   ├── devices/        设备接入、输入协议与适配
│   │   ├── inputs/     Quest 原生输入/路由、手机、idle、手柄手势
│   │   ├── phone/      手机 UDP 协议、相对位姿映射与诊断工具
│   │   ├── sensors/    相机服务（唯一设备所有者）、客户端、传输、合成相机
│   │   └── robots/     arx5：profile、真机/仿真、运动学、SDK；rebot/B601：实验适配
│   ├── contracts/      共享契约：组件接口、注册表、RPC、内核客户端、设备锁、机器人配置
│   ├── core/           项目配置与路径、坐标变换、遥测、预设位姿
│   ├── motion/         独立运动算法：轨迹、时间参数化、碰撞检查，不拥有设备
│   ├── data/
│   │   ├── episode/    MCAP / 视频 / 元数据读写与质量检查
│   │   ├── capture/    从内核 tick 采集
│   │   ├── alignment/  图像与状态的因果对齐
│   │   ├── export/     LeRobot 导出（schema 1 / 2 适配器、视频、解码）
│   │   ├── groot/      GR00T 双相机采集与导出
│   │   └── datasets.py、replay.py、task_package.py
│   └── policy/         pi05_zeroshot、pi05_sft（OpenPI）、groot；模型代码在各自环境运行
└── legacy/             robot_control（旧机械臂服务）、quest_teleop（旧遥操）、
                        arx5_control（直连 SDK 控制器）、arx_tools、hand_eye（旧 D405/D455 采集器）、
                        cube（方块演示，仅旧控制台）
```

`applications` 和 `devices` 是职责分组，不是额外的运行进程。找用户入口先看 `applications`，找设备协议和适配先看 `devices`；运动算法、数据处理和策略仍各自独立。

## 依赖与替换边界

用户程序调用 `Robot`；`Robot` 通过 `contracts.runtime` 向内核提交动作。设备驱动提供 preflight、资源标识、能力、关闭策略和模型快照元数据；诊断、viewer 与输入路由由注册表选择，输入配置仅在使用该输入时读取。规划算法接收状态、目标和约束，返回轨迹；设备后端负责 SDK、读反馈和下发命令。优化轨迹算法时不需要把 SDK 搬进 `motion`，增加相机也不应让采集与导出直接打开设备。

控制内核通过 `contracts` 的契约与注册表装配设备，并调用共享规划入口 `embodied.motion.offline`；设备 SDK、型号检查与运动学实现归设备模块。内核不反向导入平台应用、采集、策略或 `legacy`。跨 Python 环境的路径常量在 `contracts/paths.py`；平台配置在 `core`。`tests/test_convergence.py` 检查内核导入范围，`tests/test_package_boundaries.py` 检查相关模块的设备隔离。

单一组件目录 [configs/plugins.json](../configs/plugins.json) 按需加载工厂，接口见 [contracts/interfaces.py](embodied/contracts/interfaces.py)。相机服务已通过 `camera_device` 工厂选择 RealSense 或 synthetic，驱动独立于共享帧的消费端。通用命令按设备描述的关节数校验；RobotTick V3 使用最多 15 轴的固定容量和显式关节数，存储只输出实际轴数。现有完整机械臂后端仍是 X5 六轴；多轴协议通过不等于新硬件已适配。Quest 原生链路要求匹配的输入 ABI，rebot/B601 尚未接入统一内核；当前 LeRobot X5 导出器仍明确限定六轴。新增硬件需实现对应契约并验证，具体边界见[架构概览](../docs/architecture/overview.md)。`legacy/` 仍保留日常真机兼容链路，按验收计划逐步退役，新增代码不应继续依赖它。

环境定义在 `configs/environments/control/`，控制解释器安装到 `outputs/envs/control/bin/python`，每次运行写入 `outputs/runtime/control/`。设备模型放 `assets/robots/`，设备参数放 `configs/robots/`，输入参数放 `configs/inputs/`；源码目录不存虚拟环境或运行记录。

`third_party/` 仅作只读参考。正式第三方依赖按锁版本安装到项目环境，构建或安装工作区在 `outputs/dependencies/`；保留的 vendor 源码随模块保存来源与许可证，不通过参考 checkout 注入 `PYTHONPATH`。

日常仍使用 `./run.sh`。直接导入或 `python -m` 时使用完整新路径，例如 `embodied.applications.workstation.bench`、`embodied.devices.sensors.service`；旧位置不保留转发包。

想自己写控制程序，从 [examples/README.md](examples/README.md) 开始：通过 `from embodied import Robot` 调用动作，默认示例使用仿真；[Robot 接口与操作原语](../docs/reference/robot-primitives.md) 说明功能、坐标系与开源参考。

入口与命令见 [命令说明](../docs/reference/commands.md)，迁移计划见 [docs/plans/](../docs/plans/README.md)。
