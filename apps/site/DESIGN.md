# 展示站设计与内容约定

## 目标

让第一次进入的人能回答三个问题：平台整合了什么、模块如何协作、如何用同一套代码开始仿真与真机操作。
定位为模块化具身智能开发平台，向具身操作系统演进。当前接入能力与扩展边界必须同时可见。
不再把遥操、数据、代码、架构拆成相似的长章节；用一条从全貌到实际使用的阅读路径组织内容。

## 页面叙事

| 段落 | 用户得到什么 | 主视觉与交互 |
| --- | --- | --- |
| 首屏 | 输入、控制、感知、数据与策略可以在一个平台组合 | Three.js 模块拓扑；切换数据流高亮连接，选中模块查看说明；直接进入框架或仿真示例 |
| 完整架构 | 看清每个模块的职责、接口、代码与现状 | 可点击的架构 explorer；控制、采集、策略三条路径；详情面板与源码链接 |
| 一套动作接口 | 仿真与真机只替换连接，动作内容保持一致 | SIM / REAL 连接切换；关节、XYZ、夹爪、记点、采集的短代码；完整代码可复制 |
| 工作台实景 | 知道运行后实际看到什么、如何定位设备和数据状态 | 仓库已有工作台、姿态、遥测截图；手动切换；清晰的截图说明 |
| 示教到策略 | 了解采集、检查、导出、训练和执行怎样衔接 | 四阶段固定流程；双时间线、episode 产物与指南；流动示意 |
| 当前状态与开始 | 分清可直接使用、待验收、按模块扩展的部分 | 三种状态；启动工作台、可视化仿真、示例目录与安装文档 |

每段保留一个主要论点和一个主要操作。主要标题说明平台能力，细节放在用户主动选择的模块详情中。
首屏不能只摆一只 ARX 机械臂；拓扑优先表达平台组合，当前设备作为具体实现出现。

## 模块模型

节点 ID 稳定供 Three.js、架构按钮、调用路径和详情面板共用。前三列是页面信息，最后一列用于维护核对。

| ID / 标题 | 职责与接口 | 当前状态与边界 | 事实来源（仓库相对路径） |
| --- | --- | --- | --- |
| `inputs` / 输入与遥操 | 将控制来源映射为目标；`NativeDevice`、`NativeRouting`、`CommandIngress`，或 RPC 来源 | Quest 走原生输入链路；iPhone 映射后向网关提交；新 VR 需坐标、追踪质量、按键与 ABI 适配 | `src/embodied/devices/inputs/phone.py`；`src/embodied/devices/inputs/quest/composition.py`；`src/embodied/devices/inputs/quest/ingress.py` |
| `control` / 控制运行时 | 网关、动作调度、周期执行与观测；设备后端由工厂装配；`commands.sock`、`observer.sock`、`RobotBackend` | 现有完整插件为 ARX X5 / MuJoCo；X5 profile 为 250 Hz，新版真机验收尚未完成 | `src/embodied/control/gateway.py`；`src/embodied/control/commands.py`；`src/embodied/control/worker.py`；`src/embodied/devices/robots/arx5/plugin.py` |
| `motion` / 运动算法 | 根据状态、目标、约束计算轨迹；不拥有设备；`MotionPlanner`、独立 `motion.offline` | MoveJ 使用共享 Ruckig；MoveL 仍需 X5 IK；MoveP 当前仅离线几何示例 | `src/embodied/motion/offline.py`；`src/embodied/motion/contracts.py`；`src/embodied/control/commands.py` |
| `sensors` / 感知与相机 | 独立服务拥有相机，消费端共享帧、时间戳、内参；`Camera`、`camera_device` 工厂 | RealSense / synthetic 已接入；新相机实现设备工厂；标定属于应用层 | `src/embodied/devices/sensors/service.py`；`src/embodied/devices/sensors/factories.py`；`src/embodied/devices/sensors/client.py` |
| `data` / 数据工作流 | 从反馈和相机汇集 episode，再检查、回放、导出；`RuntimeReader`、`capture`、`EpisodeExporter` | MCAP、视频 / PNG、episode.json；LeRobot v2.1 与 GR00T 导出；原始记录保留 | `src/embodied/data/capture/capture.py`；`src/embodied/data/episode/runtime_episode.py`；`src/embodied/data/export/lerobot.py`；`src/embodied/data/groot/` |
| `policy` / 模型与策略 | 组织观察、在独立模型环境推理、映射动作并提交控制入口 | OpenPI / π0.5、GR00T 各有适配；动作与相机配置仍需匹配；不能宣称任意模型即插即用 | `src/embodied/policy/kernel_runner.py`；`src/embodied/policy/pi05_zeroshot/worker_client.py`；`src/embodied/policy/groot/kernel_runtime.py` |
| `applications` / 应用与工作台 | 服务装配、Robot API、CLI、监看、控制台与标定 | Python / 示例 / 动作 CLI 已共用 Robot；网页动作仍调用 ControlClient / ControlSession，尚未全部收敛 | `src/embodied/applications/workstation/bench.py`；`src/embodied/applications/operations/robot.py`；`src/embodied/applications/operations/cli.py`；`src/embodied/applications/monitor/` |
| `contracts` / 共享契约 | 控制与观测客户端、组件协议、注册工厂、模型配置读取 | 统一单位与生命周期边界；协议存在不等于新增硬件已经适配 | `src/embodied/contracts/interfaces.py`；`src/embodied/contracts/registry.py`；`src/embodied/contracts/runtime/`；`src/embodied/contracts/robot.py` |
| `core` / 平台基础 | 项目配置与路径、坐标变换与遥测等平台工具 | X5 / Pinocchio 模型已经归设备适配器 | `src/embodied/core/project.py`；`src/embodied/core/transforms.py`；`src/embodied/devices/robots/arx5/pinocchio_model.py` |

首屏以 `inputs / control / sensors / data / policy` 为主节点，运动、应用、契约、基础在完整架构展开。
模块边界按职责呈现，不把九个节点误画成九个常驻进程。实际源码是 `src/embodied/` 的公共运行时、设备适配与领域模块，以及 `apps/` 界面。
`motion` 是被内核调用的独立算法，`contracts` 是共享接口，均不是与控制内核串联的网络服务。

## 三条真实调用路径

### 控制

`Robot.movej(q) → ControlSession / commands.sock → MotionPlanner → motion.offline → worker → ARX / MuJoCo → 反馈到位`

- `Robot` 负责组织规划、执行和等待；`src/embodied/control/commands.py` 通过模型插件规划，并根据反馈判断动作完成。
- `motion.offline` 负责轨迹计算，内核按周期消费 q/dq；算法优化不要求修改任务代码或直接调用 SDK。
- Quest 原生输入与策略流式目标有自己的入口；不能把每个来源画成经过 `Robot.movej()`。

### 采集

`observer.sock → RuntimeReader` 与 `CameraService → SharedCamera` 并行汇入 `capture → RuntimeEpisodeWriter → 检查 / 回放 / 导出`。

- 相机图像不经过机械臂 worker，数据导出和训练不需要占用机械臂。
- `Robot.record()` 默认只记录命令和反馈；多相机示教在采集入口配置相机。
- 原始 episode 与训练格式分别存放；不得把 LeRobot v2.1 写成当前第三方生态的最新版本。

### 策略

`ControlSession.status() + SharedCamera → PolicyWorker → 观察 / 动作映射检查 → ControlSession.command("joint", ...) → gateway / worker → 下一轮观察`。

- 这条链路以 `src/embodied/policy/kernel_runner.py` 的 OpenPI 实现为依据；GR00T 有单独适配。
- 流式 joint 目标与 MoveJ 的整段规划执行语义不同，图中保留区别。
- 模型推理在独立环境，SDK 仍归设备后端；不宣称训练、推理与全部硬件已完成端到端验收。

## API 展示规则

代码对应控制示例；网页切换同一段动作外层的 `Robot.sim(viewer=True)` / `Robot.real(run)` 连接。
下面的任务函数展示相同边界，对应 `src/examples/simulation/control.py` 的组合方式：

```python
from embodied import Robot

def task(robot):
    q = robot.q
    q[0] += 0.2
    robot.movej(q)
    x, y, z = robot.xyz
    robot.movel(xyz=[x, y, z + 0.02])
    robot.gripper.open()

with Robot.sim(viewer=True) as robot:
    task(robot)
```

真机切换只替换连接行为为 `Robot.real(run)`；`run` 必须由用户指向已启动的真机内核目录，不隐式猜测目录。
可显示 `run = "outputs/runtime/control/<RUN>"` 并注明需要替换；浏览器只展示和复制代码，不启动硬件。

- `Robot.sim(viewer=True)` 创建并管理本次仿真和窗口；`Robot.sim()` 为无窗口模式。
- `Robot.sim(run=run)` 与 `Robot.real(run)` 检查已有运行的后端类型，退出时保留外部内核。
- `Robot.real(run)` 不启动或使能硬件；同接口不代表仿真动力学、误差和现实任务结果一致。
- `movej(q)` 用 rad；XYZ / offset / 夹爪宽度用 m；仅 XYZ 时保持当前朝向。
- 六值姿态是 XYZ + 旋转向量，七值姿态是 XYZ + XYZW；不把旋转向量称作 RPY。
- `speed / acceleration / jerk` 仍约束关节轨迹，不使用暗示末端 m/s 的 UR `v` 参数。
- `teach()` 是记点；`gripper.close()` 是开合；不能写成拖动示教或已验证抓取。

## 视觉与动态

沿用项目深色空间、靛蓝与薄荷色的识别，增加明确的排版层级、章节留白和有内容的空间结构。
参考 RIPO 的任务导向与可追溯证据；采用本项目自身的架构与素材，不复制它的科研内容。

- Three.js 呈现可选中的模块与连接；路径光点表达数据流，支持拖动旋转、键盘调整与轻微指针视差，不自动漂浮或切换视角。
- 场景注明“架构示意，非实时数据”；不生成假 FPS、连接状态、成功率或测量曲线。
- 用户切换控制 / 采集 / 策略时，连接与说明一起变化；不靠颜色单独表达选中状态。
- 滚动进入的淡入与小位移只辅助节奏；不能让动画结束成为阅读正文的前提。
- 截图使用现有 `assets/` 内容并标注用途；不得将静态截图包装成实时控制台。
- 主动操作立即反馈，不做自动翻页、强制滚动或自动切换代码。动效暂停入口始终可找到。

## 可访问性、性能与静态托管

- HTML 提供全部主要内容与入口；WebGL 不可用时保留静态拓扑和模块按钮，文案不消失。
- 每个 3D 节点有等价 DOM 操作；架构选择、标签、复制、暂停支持键盘与可见焦点。
- `prefers-reduced-motion` 下关闭持续流动与指针视差，切换和手动视角操作仍可使用；暂停只影响动效。
- Canvas 离屏、标签页隐藏时停止持续渲染；限制像素比、几何数量和粒子数量。
- 手机架构纵向排布，代码区域内部滚动，正文不横向溢出；触屏操作不依赖 hover。
- Three.js 0.180.0 沿用仓库内已有版本，放在 `assets/vendor/three/` 并保留 MIT 许可证；无远程字体、在线 CDN 或构建步骤。
- 全部文档 / 源码链接指向仓库页面；保持 `./run.sh web site` 与独立静态服务器可预览。

## 内容验收

维护时先对照 [架构概览](../../docs/architecture/overview.md)、[接口说明](../../docs/reference/robot-primitives.md)、[源码导航](../../src/README.md) 与真实实现。
仿真已贯通、legacy 真机日常兼容、新版内核真机待验收三种状态必须能被找到。
250 Hz 只能标为 worker 配置频率；截图、动态拓扑和仿真结果均不代表实机性能结论。
当前 X5 六轴、RealSense / synthetic、Quest / iPhone 是已有实现；新增设备仍需适配与验证。
入口至少包含 `./run.sh`、`./run.sh examples sim_control`、`./run.sh examples` 与安装指南。
实现后只检查本页相关的桌面 / 手机排版、交互、降级与控制台错误，不因展示站改动启动真机或运行全仓测试。
