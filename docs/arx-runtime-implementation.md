# arx 架构迁移实施记录

> 合并更新：三个原目录已退役，源码/资源与独立操作已迁入本仓库，原生 arx 默认链路保留。新增范围和精确集成边界见 [合并后的组件说明](consolidated-components.md)。以下为先前原生链路实施记录。

日期：2026-09-29。分支：`refactor/arx-runtime`。本次交付插件框架和可用的基本功能，并完成离线/仿真验收。真实设备验收待用户连接设备后进行。使用命令和接口见 [插件工作台](plugin-workbench.md)；[主方案](arx-runtime-redesign.md)、[部件边界](component-boundaries.md)、[输入扩展](control-inputs-and-motion.md) 保留设计依据与后续扩展目录。

范围复核：三个参考仓库尚未全部迁完。[插件迁入清单](plugin-migration-inventory.md) 按“有用功能迁入，已有完整替代则合并”列出 41 项功能及来源、现状、插件边界和验收要求，补齐 B601、双相机标定、视觉任务、GR00T 与诊断等原先未纳入的范围。该清单是后续方案，不增加本文的已实现项。

最终交付须达到三个原目录全部可退役：运行、构建、测试和维护不再需要 `arx5_test`、`reBotArm_control_py`、`robot-teleop`。迁入清单新增 P5 独立环境验收及逐文件处置要求；现阶段尚未达到完整退役条件。

## 当前已实现

| 功能部件 | 实现与边界 |
| --- | --- |
| arx 控制内核 | `control/` 完整导入 arx-televr 59 个受版本管理文件；保留原 Quest 原生输入/路由/映射/IK、模型、SDK、MuJoCo。新增 managed 模式门控命令，原 unmanaged 基线可单独运行。 |
| 插件契约/装配 | `component_api` 负责显式注册、延迟加载、API/运行环境检查；`workstation` 负责 profile 和进程生命周期。机械臂、VR 设备、原生路由装配、命令入口、目标控制器、控制源、映射、规划、相机、标定和导出均通过工厂选择。 |
| 控制源 | `source.cli/quest/phone.arkit`。手机复用 robot-teleop 的 iOS App、UDP 二进制协议、session/sequence 和相对位姿策略；不带入第二个机械臂 SDK owner。 |
| 控制权/动作 | `control/adapters/commands.py` 与 gateway：单 owner、epoch/token、独立目标 TTL；预览/执行/查询/取消、MoveJ/MoveL、关节点动、夹爪、Home、FreeDrive、保存/返回示教点。规划与网络不运行在机器人周期。 |
| 遥测/客户端 | 固定布局 RobotTickV2 增加 source/epoch/输入序号/operation。只读 observer 与命令 socket 分开；标准库客户端保持 v1 档案兼容。 |
| 相机/采集 | 复用独立 SharedCamera 服务，增加 synthetic 插件；`collection` 独立消费完整 tick 与图像；检测序号、stream ID、掉帧、EOF 和 owner 变化。 |
| 文件/对齐 | `episode_io` 保存 MCAP、PNG、元数据和完成边界；`observations` 分别选择因果反馈与命令，原始时间不被改写。旧存储模块保留兼容入口。 |
| LeRobot | `dataset_tools.exporters` 注册 schema 1/2 适配器，复用原视频转换流水线；schema 2 导出 `aligned30_raw250_v2`，保留完整原始 sidecar。OpenPI 数据适配识别新布局和 action 定义。任务 ZIP 可带两种 schema 的独立转换工具，禁止混装不同 action 定义。 |
| 自动手眼标定 | `handeye --runtime <RUN>` 使用 arx 模型及控制适配器；原 marker/manual/plan/auto/solve、工作区检查、静止采样、验证集求解保留。旧标定入口仍可使用。 |

基本功能的使用不再以“后续迁移”为前提。仍保留旧默认入口，是因为旧采集按键、任务页面、回放和策略执行尚未接新 runtime；不会把原页面或旧策略自动转成新动作定义。MoveC/混合路径、浏览器手势、限扭矩夹爪和负载实验属于扩展能力，本次未实现。

## 原生组件拆分补充实施

原 `NativeQuest` 的启动和门控职责已分开，`NativeVR` 已替换为窄接口。profile 显式连接 `native.device/routing/ingress/channel`，普通手机/CLI 不加载 VR；公共 worker/裁决不依赖 Quest 名称。`NativeSession` 仅续租，租约绑定入口及用途；候选受 epoch、期限和依赖健康约束。real/sim 都接受原 `controller_factory`，默认控制算法不变。具体接口见 [插件工作台](plugin-workbench.md)，依据与代码路径见 [组件复核](arx-component-review.md)。

命令订阅与反馈发布也在原 `RobotBridge` 上分开：managed 的命令端属于 ingress，反馈总线始终由 worker 发布。因此没有 VR 的手机/CLI 运行仍支持原只读 viewer。默认完整桥接保持上游行为。

原生组件的失败由 worker 按当前 owner 的入口处理，撤权并反馈保持；其他入口继续运行。coordinator 的许可和路由健康仍沿原实现流转，额外监督进程存活及 supervisor 心跳。原生故障不会隐式发起 Home；恢复须重新申请会话。装配先等 coordinator 的首个 permit 再启动路由，解决原生共享控制主题的容量创建竞争；源进程等整个 runtime 就绪后启动。

本轮新增证据均在 `outputs/migration/`：

| 证据 | 范围 |
| --- | --- |
| `native-root-tests.log` | 根目录完整 300 项回归通过，4 项 LeRobot 环境依赖跳过；新增 profile 测试另见下项。 |
| `native-source-tests-final.log` | 7 项通过；包括手机真实 UDP、无需 VR 的手机/CLI profile、按能力绑定重命名源、入口不匹配和真机使用合成输入的拒绝。 |
| `native-command-tests-final2.log` | 13 项通过，包括模式重新同步和无 VR 时的独立反馈；任意来源名称的入口绑定、候选 epoch/期限、取消、故障隔离、重复/旧帧不续命、原生 permit、禁止故障 Home、控制器注入。 |
| `native-control-tests-final.log` | 原离线/假 SDK 检查与 12 项入口契约通过，新增 RPC-only worker 的真实 IPC 反馈订阅；unmanaged 原生 demo/MuJoCo 250.00 Hz、3,020 tick、4,021 完整记录，7 项链路检查通过。 |
| `native-components-smoke3.log` | 实际工作台 profile 启动原生 demo 和租约源，约 250 Hz、451 个位姿命令、TCP 位移约 5.5 cm；停止后正常清理。 |
| `native-faults-feedback-final/result.json` | 最终 10 个故障场景通过：4 组件分别退出/挂起、supervisor 挂起、手机活动时同时退出全部原生组件。退出撤权观测 24–72 ms，输入/协调器挂起 72–120 ms，路由挂起约 504 ms，supervisor 挂起约 192 ms；检查无隐式 Home，故障后 RPC 可接管。 |
| `native-multisource-final/result.json` | 共享网关重跑 MoveJ/MoveL → 真实 UDP 手机 → arx 标定适配的快照/运动/保持/释放；12,156 控制 tick、249.86 Hz、遥测丢样 0。合成相机 60 帧/503 条状态命令全部因果对齐；本轮未重复 26 点求解，其证据仍见先前验收。 |
| `native-portable-contracts-final.log` | 最终控制包解压到长独立路径，13 项命令/反馈契约通过，共享源码哈希全部匹配。 |
| `native-portable-fault/result.json` | 独立目录实际运行 managed 原生管线，249.99 Hz；协调器退出后约 48 ms 观测撤权，RPC 随后接管，正常清理。 |
| `native-workbench-final/result.json`、`workbench.log` | 最终 profile 经完整工作台入口启动，等待 runtime 就绪、独立 source 获取 native 租约并跟随，SIGTERM 有序清理。 |

以上时间为本机仿真的观测值，不是最坏时延保证；真实设备尚未连接。首轮 smoke 的 coordinator 容量失败、故障脚本首次读取错误的 tuning 字段均保留在早期日志，修复后重跑。

## 来源与环境隔离

- arx-televr 固定为 `cd557afbb9ec280df2d9e12e1582219b85ee1a53`。`control/UPSTREAM_FILES.json` 冻结原 SHA256/执行位，`LOCAL_PATCHES.json` 记录 13 个局部补丁和理由；新增 adapter 单独保留。原兄弟仓库未修改。
- 手机来源为 robot-teleop `864d6501a3eeef99b7facdc8f592d80e0ff52843` 的实际工作树快照，包含其本地修改。`third_party/phone_input/PROVENANCE.json` 逐文件固定导入的 Python、iOS、golden vectors；MIT 许可同目录保留。
- 运动纯数学来源为 arx5_test `a18d6b67906e450411d56e3d0fd08fd18a01675c` 的工作树；`operations/vendor/arx_motion/PROVENANCE.json` 和 `LOCAL_PATCHES.json` 固定来源与 Python 3.10 cube-root 兼容补丁。没有迁入其大型硬件/网页控制器。
- reBotArm 的拖动/保持/示教思路用于功能设计；其 B601 增益、动力学和已发现不连续的梯形轨迹实现未复制到 X5。
- 所有运行依赖位于本仓库；`dependencies.lock.toml` 登记三组 vendored 来源。已有用户修改保留，先前基线备份在 `outputs/migration/arx-runtime-baseline/`。

控制使用 Python 3.10.20、NumPy 1.26.4、MuJoCo 3.6.0、iceoryx2 0.9.1；工作台/手机使用根目录 Python 3.12。LeRobot 导出使用 `outputs/envs/lerobot-export/`。Python 3.10 的夹爪修复 SDK 构建位于 `control/build/arx-gripper/`，不覆盖旧 3.12 扩展。新组件和 arx 标定入口不会加载旧控制 SDK 路径。

```bash
./run.sh control verify
./run.sh control setup --sdk
./run.sh control build-sdk
./run.sh control doctor --vr
./run.sh workbench validate --profile sim-manual
./run.sh workbench start --profile sim-manual --headless
```

`setup`、`build-sdk` 和诊断不启动 CAN 控制。**arx 真机启动会使能并回 Home；Disabled 也表示 Home；停止请求 SDK 阻尼。** 普通取消/租约释放/换源则保持当前位置，不重新连接设备，不隐式 Home。完整启动语义见 [插件说明](plugin-workbench.md)。

## 本次验证证据

所有数据为仿真/合成输入或假 SDK，不包含真实设备验收。

| 证据 | 实际结果 |
| --- | --- |
| `outputs/migration/plugins-acceptance-1/result.json` | 同一 managed arx 运行完成 CLI MoveJ、MoveL → 手机 → 自动标定 → 显式停止。147.46 秒、36,859 控制 tick、249.96 Hz；9 次周期超过 4.8 ms，最慢 19.26 ms；遥测丢样 0。平均频率通过，未声称硬实时。 |
| 同目录 `collector.log`、原始 episode | 真实本机 UDP 60 Hz、原映射、合成 RGB 30 FPS；503 条状态/命令、60 帧，60 个因果锚点全部可用。所有录制 tick 来自 phone；source 配置随数据保存。 |
| 同目录 `calibration/session.json`、`result.json` | 26 个位姿由 arx gateway 和 MuJoCo 实际执行；每姿态保存 20 帧，共 520 帧；20 个 fit + 6 个独立 validation，求解状态 validated。检测位姿来自合成真值，不等于真实标定精度。 |
| 同目录 `export.log`、`lerobot/` | 上述实际录制的手机 episode 导出并由真实 LeRobot v2.1 loader 加载，60 帧、无排除条目。 |
| `plugins-export-tests.log` | 独立导出环境 5 项全部通过：新/旧 ZIP 解包后在仓库外转换，PNG/视频像素与顺序、action chunk、命令晚于反馈的时间保留、原始 sidecar、schema 混用拒绝、失败清理。 |
| `plugins-root-tests.log`、`plugins-compatibility-tests.log` | 根回归共 300 项：首轮 11 项因旧注册表模块兼容钩子失败；修复后 11 项全部通过，其余 285 项首轮通过。4 项需 LeRobot 的测试由独立导出环境补齐。 |
| `plugins-control-tests.log` | 原 arx 离线检查、9 项假 SDK 检查、7 项新增控制权/期限/计划/Quest 门控测试均通过；原生 demo/router/MuJoCo 250.00 Hz，3,017 控制 tick、4,017 完整 tuning 记录、7 项链路验收通过。 |
| `plugins-sources-final.log` | 5 项手机/配置测试通过：原 7 类 Python/Swift 二进制金样本、真实 UDP 相对映射/夹爪、重复包和断流、追踪/session 重新激活、批次 ESTOP 不被新姿态覆盖。 |
| `workbench-lifecycle/result.json` | 实际 profile 启动手机插件与合成相机，SIGTERM 按顺序结束；522 条状态/命令、62 帧，录制 complete，运行正常清理。 |
| `plugins-portable-final/managed.log`、`contracts.log` | 从最终控制源码包解压到更深的独立目录，managed MuJoCo 249.99 Hz；命令与只读 observer 均可连接，7 项命令契约通过。首次搬移暴露 Unix socket 绝对路径长度限制，已使用目录 fd 修复并在长路径复验。共享源码哈希全部通过。 |
| `plugins-client-final.log`、`plugins-entry-final.log`、`plugins-handeye-cli.log` | 客户端、启动边界和来源检查通过；实际 arx 模型生成 26 点标定计划，入口不加载旧 SDK。 |

后续小范围修改（模块兼容、长路径 Unix socket、来源指纹）分别在对应测试和可搬移运行中复验；不将首次失败日志隐藏或改写为全绿。

先前控制基线的来源校验、夹爪构建、逐步仿真输出一致性、慢消费者、viewer/observer 独立退出与 terminal 采集证据保留于 `outputs/migration/arx-runtime-baseline/`。它们与本次功能验收区分记录。

## 可搬移性与已知边界

控制源码包同时携带原生资源、模型、适配器、纯 Python 共享接口/运动数学及来源哈希，排除虚拟环境和采集数据。完整工作台及 iOS App 以本仓库交付。任务 ZIP 包含 `episode_io`、`dataset_tools`、对齐、注册表等转换必需源码，已实际脱离仓库验证。

一个 episode 绑定一个 owner epoch；换源前先结束录制。动作阶段与普通示教跟随分开；原始数据保留所有阶段，默认训练导出只选 Quest/phone 的位置跟随。raw XR/UDP 全报文尚未落盘，不能据此宣称能逐包重放输入。

当前插件协议是 X5 六轴加夹爪，非任意机械臂通用运行时。MoveL 有离散导数检查，没有碰撞检测。手机软件 ESTOP 锁存停止，网络 RESET 不解锁运行。标定 `damp()` 在 arx 适配中是保持并释放流程，不能解释为主动阻尼控制。

真实 Quest/iPhone 的按键、方向、追踪恢复，RealSense 帧流、真实标定板，CAN/夹爪/起始 Home、跟随和退出是接设备后的验收项。Linux 环境未编译 iOS App。框架、基本能力和无设备验证已具备，无需再等待这些现场验收才编写基础功能。
