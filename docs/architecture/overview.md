# 架构概览

项目按模块化具身平台组织：用户通过统一入口编排动作，运动、数据和策略能力独立维护，硬件差异放在适配端。机械臂插件包括 ARX X5 六轴真机与 MuJoCo，以及独立的七轴无夹爪 Reference7 仿真。后者用来验证关节数、模型与夹爪可选性，没有对应真机。“具身操作系统”是建设方向，不代表已有任意机器人、相机和 VR 的通用驱动。

## 从使用入口到实现

| 用户入口 | 平台职责 | 实现位置 |
| --- | --- | --- |
| `./run.sh` | 工作台、组件选择、服务生命周期 | [applications/workstation](../../src/embodied/applications/workstation/) |
| `from embodied import Robot` | 简短的动作程序、规划与执行、等待结果 | [operations/robot.py](../../src/embodied/applications/operations/robot.py) · [示例](../../src/examples/README.md) |
| 网页与采集面板 | 查看状态、组织操作与数据 | [applications/monitor](../../src/embodied/applications/monitor/) · [data](../../src/embodied/data/) |
| 设备与算法扩展 | 按现有协议实现工厂，由装配层选择 | [contracts/interfaces.py](../../src/embodied/contracts/interfaces.py) · [组件工作台](../guides/plugin-workbench.md) |

## 职责与依赖

```text
用户入口        ./run.sh · Python Robot · CLI · 网页 · 示例
                       │
平台应用        applications · devices 输入端 · data · policy
                       │ contracts.runtime：命令与观测
                       ▼
控制运行时      embodied/control：gateway → worker → 设备插件
                                              devices/robots/{arx5,reference7}
                       │                    │
                       ▼                    ▼
共享基础        contracts：接口与协议      motion.offline：独立轨迹算法
```

- 共享契约和运动算法不依赖应用流程。内核可直接导入 `embodied.contracts` 和 `embodied.motion.offline`；设备实现通过注册表选择，不导入 `applications`、`data`、`policy` 或 `legacy`。导入范围由 `tests/test_convergence.py` 检查。
- `motion.offline` 从输入向量推导自由度，用状态、目标与逐轴约束计算轨迹；不创建机械臂或相机连接。X5 的模型、IK、目标控制器和硬件后端位于 `devices/robots/arx5/`，公共规划器为 `planner.motion.default`。
- 统一内核链路只有内核拥有机械臂 SDK；相机由 `embodied.devices.sensors` 的相机服务拥有，采集与监看消费共享帧。仍保留的旧真机服务是另一条兼容链路，不能与内核同时占用同一设备。
- 关节限位、桌面平面、反馈检查与停止在内核处理。公共 API 负责参数含义和请求，不复制控制循环或独立限速器。
- 操控行为以上游 arx-televr 为准；平台需要的改动依次选择：上游原样 → `src/embodied/devices/robots/arx5/` 外围适配 → 登记在 `docs/provenance/arx-televr/LOCAL_PATCHES.json` 的局部补丁。

## 替换组件时改哪里

已有 [Registry](../../src/embodied/contracts/registry.py) 负责按选择懒加载工厂，`contracts.interfaces` 描述结构化接口。协议和注册表提供接入位置；接入是否完整还取决于模型、状态字段和运行时行为是否匹配。

| 模块 | 已有边界与实现 | 更换时仍需完成的工作 |
| --- | --- | --- |
| 机械臂 | `RobotBackend`；`devices/robots/arx5/plugin.py` 接 ARX 真机与 MuJoCo，`reference7/plugin.py` 接独立七轴 MuJoCo | worker 按 profile 的关节数与周期工作；preflight、夹爪能力、模型快照、诊断/viewer 与关闭策略归设备工厂。网页读取运行描述中的 URDF / 关节顺序，不固定 X5。新型号仍需完整实现与验收；B601 是独立实验实现 |
| 相机 | `CameraService` 通过 `camera_device` 注册工厂选择 RealSense / synthetic，沿用 `Camera` 生命周期及共享帧 | 配置 `cameras.<id>.driver`；新硬件实现 [设备工厂](../../src/embodied/devices/sensors/factories.py) 与图像、内参、时间戳契约。`camera` 注册项是消费端，`camera_device` 才拥有设备；采集/导出无需改为直连驱动 |
| VR / 控制来源 | `NativeDevice`、`NativeRouting`、`CommandIngress`；Quest 原生链路、手机 RPC 来源 | 新 VR 可适配原生输入 ABI，或作为独立来源提交统一目标；需处理坐标系、追踪质量、按键与重定位。当前没有任意 VR 的通用驱动 |
| 运动规划 | `MotionPlanner`；MoveJ 复用 Ruckig，MoveL / `move_path` 使用模型插件的 IK 与共享时间参数化 | 关节限值和显式 TCP / 角速度约束进入同一计划；圆角多段路径为一次 operation。X5 与 Reference7 各自实现运动学，算法不依赖任务或网页 |
| 数据与策略 | `RuntimeReader`、episode、导出器、共享 OpenPI 契约适配与独立策略环境 | `data_contract` 声明关节顺序、夹爪可选性，`camera_roles` 声明模型图像角色；训练与推理复用同一向量转换。GR00T 等专用格式仍有自己的设备边界；策略不能绕过内核直接下发 SDK |

添加能力时优先扩展这些现有边界；没有实际后端或算法时，不登记空实现。终端、网页和示例使用相同能力名称，具体运行是否支持以 `describe` 与对应实现为准。

## 多个版本如何收敛

同一动作保留一个公共语义与共享实现；不同硬件保留各自驱动。下面区分已统一的部分与待迁移的调用方，避免为每个界面再造一套控制版本。

| 范围 | 当前调用关系 | 收敛方向 |
| --- | --- | --- |
| Python、示例与动作 CLI | `examples / operations.cli → Robot → contracts.runtime → 内核`；共同动作已共享规划、执行等待、夹爪和记点返回实现 | `Robot` 作为用户编程入口；CLI 保留参数、输出和文件管理，`stop` / `free-drive` 保留独立内核管理语义 |
| 网页控制台 | `console/kernel/arx.py` 仍调用 `ControlClient/ControlSession`；旧控制台仍有直连 SDK 实现 | 将公共动作语义继续收敛到 operations，网页保留呈现与交互，不复制运动算法 |
| 运动算法 | 新内核与旧控制器已复用 `motion.offline`，仍各自保留部分轨迹装配与控制流程 | `motion` 作为可复用算法的唯一维护位置；从旧版提取有效算法并验证后复用，内核保留模型适配与实时执行 |
| 设备驱动 | 机械臂、相机、控制来源分别有协议及具体实现 | 统一能力、单位、状态与生命周期契约；各驱动独立实现硬件行为，不为换设备复制采集、策略或界面 |
| 旧链路 | `legacy` 与旧控制台承担尚未迁完的真机和专用流程 | 逐项迁移、对照验证后切换入口；只有功能与数据语义已有替代且完成相应验收，才删除旧实现 |

## 内核的进程与接口

```text
                    ┌───────────────────── embodied/control（一次运行 = outputs/runtime/control/<RUN>/）─────────────────────┐
 Quest ─▶ rollio-device-xrobot ─▶ rollio-teleop-router ─▶ native ingress ┐                                 │
                                                                          ├─▶ worker（profile Hz）─▶ real / sim   │
 手机 / CLI / 动作 / 标定 / 策略 ─▶ commands.sock ─▶ gateway ─────────────┘        │                       │
                                                                                    ▼                       │
                                                       observer ◀── RobotTick（共享内存）                   │
                    └───────────────────────────────────┬────────────────────────────────────────────────┘
                                                        ▼
                                       observer.sock：采集、监看、诊断（只读）
```

| 接口 | 用途 |
| --- | --- |
| `commands.sock` | 长度前缀 JSON。`describe`（限位、频率、模型、工作空间、夹爪能力）、`status`、`acquire` / `heartbeat` / `release`（租约）、流式 `joint` / `cartesian` / `gripper` / `free-drive`、`preview` / `execute` / `cancel`（MoveJ / MoveL / Path / Home）、反馈驱动的 `grasp`、`hold` / `stop`。完整字段与单位见 [内核命令协议](../reference/kernel-protocol.md) |
| `observer.sock` | RobotTickV3 流：显式关节数（1..15）、反馈、命令、来源、owner epoch、操作 ID；`latest` 或 `full` 模式 |
| `embodied.contracts.runtime` | `ControlClient`、`ControlSession`、`RuntimeReader`，平台侧唯一的内核客户端 |

右手柄由原生 coordinator 处理，语义与上游相同；工作台只加一层“允许 / 暂停”，并要求先松开全部按键。其他来源经 gateway 进入，统一检查限位、桌面平面与步长。

## 数据流

| 阶段 | 位置 | 产物 |
| --- | --- | --- |
| 采集 | `embodied.data.capture`（内核 tick）、`operations.recording`（任务录制）、旧链路会话 | MCAP + 视频 / PNG + `episode.json`；任务录制附事件、结果、场景快照与显式 scripted 来源；`capture-status.json` 提供实时心跳、计数与错误 |
| 查看与回放 | `embodied.data.datasets`、`embodied.data.replay` | 质量报告、标注 |
| 导出 | `embodied.data.export` | LeRobot v2.1 + `robot/*.npz` 原始样本 + `meta/quest_export.json`（action 定义、控制栈、相机、缺口） |
| GR00T | `embodied.data.groot` | 双相机 Rot6D 16 维数据集 |
| 训练 | `scripts/training/openpi`（训练服务器） | checkpoint |
| 推理 | `embodied.policy.pi05_sft`、`pi05_zeroshot`、`groot` | 机械臂动作 |

原始数据从不改写；格式变化只增加新的读取适配。RobotTick V3 是遥测协议版本，episode schema 2 表示存储及动作语义。LeRobot 关节向量导出读取运行声明的轴数、夹爪与相机，不把第七维固定解释为夹爪。OpenPI 使用同一契约适配训练样本和推理观测，保留 X5 预设；模型动作维度填充必须显式配置。这些接口验证不代表任意新硬件已有可用策略权重。

人工遥操与程序任务在 provenance 中分开。只有明确声明的 scripted 来源才允许纳入 robot-api 的 managed phase=4；未提交指令、owner/epoch 不符、错误和失败样本仍受过滤。任务事件转换成技能标注后先处于待复核状态，不把脚本执行完成直接当作高质量训练标签。

监看页按运行标识与进程身份检查采集心跳，区分录制、收尾、过期、中断和完成；未知计数显示为空。运动性能报告从全量 tick 计算，保留原始样本、配置和源码标识，避免用页面刷新频率推断控制周期。

## 配置

| 文件 | 内容 |
| --- | --- |
| `configs/robots/arx5/robot.json`、`tool.json` | ARX profile、逐轴参数与 TCP；设备适配器解析型号约束 |
| `configs/robots/reference7/robot.json` | 七轴无夹爪教学模型、独立 profile / 运动学 / 仿真 / viewer；仅支持仿真 |
| `configs/control/motion.json`、`workspace.json` | 公共运动策略与工作区约束 |
| `configs/inputs/quest.json` | Quest 输入配置，选用时读取 |
| `assets/robots/x5/runtime/` | 由 `scripts/control/build_model.py` 生成并校验的 URDF / MJCF |
| `configs/project.toml` + `configs/local.toml` | 平台：路径、端口、相机、旧服务参数；本机覆盖不提交 |
| `configs/plugins.json` | 单一插件目录：工厂、环境、能力与硬件属性 |
| `configs/components/commands.json` | 启动命令与环境选择 |
| `configs/environments/control/` | Python 3.10 声明与锁文件；安装到 `outputs/envs/control/` |
| `configs/workbench/*.json` | 内核工作台组合 profile |

`third_party/` 是 Git 忽略的本地只读参考目录；来源记录在 `docs/provenance/`；运行依赖通过锁版本安装，受管构建工作区在 `outputs/dependencies/`。维护的 vendor 代码与原生资源随设备模块保留来源、许可证及补丁记录。参考决策见 [PhyAgentOS 取舍](phyagent-reference.md)。

## 现状与计划

当前日常真机兼容命令仍走旧机械臂服务。统一内核已进行保持与小幅运动现场测试；后续接回 Ruckig、逐轴限值与终点保持的版本已做离线/仿真验证，尚未完成新版真机验收。不能将已完成的单项测试等同于整个平台或任意硬件已验收。

验收范围见 [真机验收](../guides/real-arm-acceptance.md)。客户端迁移、重复实现合并和旧链路删除的计划见 [docs/plans/](../plans/README.md)。目录与具体实现见 [源码导航](../../src/README.md)。
