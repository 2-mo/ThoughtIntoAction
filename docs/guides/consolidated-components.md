# 三个仓库合并后的使用与维护

`arx5_test`、`reBotArm_control_py`、`robot-teleop` 已从原位置移入 QuestArmTeleop。日常只维护本仓库；可用源码按功能位于 `src/`，前端位于 `apps/console/`，iOS 工程仍在 `apps/ios/`。原项目完整工作树曾归档在本机 `outputs/archives/migration/originals/`（未纳入 git），不参与导入、启动、构建或测试。

默认 Quest 遥操继续使用 `control/` 中的 arx 原生 VR、coordinator/router、控制器、设备后端与现有工作台。合并没有替换它的默认控制算法。

## 功能位置和入口

| 功能 | 唯一维护位置 | 入口/插件 |
| --- | --- | --- |
| 原生 Quest、手机接管、基础动作 | `control/`、`src/embodied/devices/inputs/`、`src/embodied/applications/operations/` | `workbench`、`phone`、`motion`，用法见 [工作台](plugin-workbench.md) |
| Ruckig、圆弧/混合路径、碰撞、运动学 | `src/embodied/motion/`、`src/embodied/devices/phone/controllers/`、`src/embodied/devices/phone/kinematics/` | `trajectory`、`path`、`collision`、`servo_algorithm`、`kinematics` 库插件 |
| B601 DM/RS、分组电机与夹爪 | `src/embodied/devices/robots/rebot/actuator/`、`configs/rebot/`、`assets/robots/rebot/` | `device.rebot_dm/rebot_rs`；`components run rebot.*` |
| B601 运动学、动力学、SE(3)/CLIK | `src/embodied/devices/robots/rebot/{kinematics,dynamics,trajectory}/` | `kinematic_analysis.rebot`、`dynamics.pinocchio`、`path.se3_geodesic`、`trajectory.clik` |
| B601 末端操作、拖动与自动保持、MeshCat | `src/embodied/devices/robots/rebot/controllers/`、`docs/examples/rebot/` | `rebot.endpose/trajectory/gravity/stationary_hold`、`viewer.rebot_*` |
| 浏览器手势、运动编辑、Ghost、任务页面 | `apps/console/`、`src/embodied/applications/console/features/` | `ui.build`、`ui.preview`、`ui.backend`、`ui.console` |
| D405/D455 RGB-D、WebRTC、相机诊断 | `src/embodied/applications/console/features/camera/`、`src/legacy/arx_tools/` | `camera.rgbd`、`diagnostic.d405` |
| 手眼标定（经内核） | `src/embodied/applications/calibration/` | `./run.sh calib handeye`；旧 D405/D455 采集器在 `src/legacy/hand_eye/`（随 K-10 删除） |
| 夹爪、负载标定 | `src/legacy/arx_tools/` | `calibration.gripper/payload` |
| 方块感知、抓取/旋转/堆叠 | `src/legacy/arx_tools/rgb_cube_detector.py`、`src/experiments/`、`src/embodied/applications/console/features/cube/` | `perception.rgb_cube`、`task_planner.*`、`task.cube` |
| 双相机采集、审核、GR00T 导出 | `src/embodied/data/groot/` | `collection.groot`、`episode_store.groot`、`dataset_export.groot_v21` |
| GR00T 权重检查、异步推理及动作分块 | `src/embodied/policy/groot/`、`third_party/Isaac-GR00T/` | `policy.groot`、`policy_actions.groot`、`checkpoint.groot` |
| 手机 mock/replay、握手、诊断、回放检查 | `src/embodied/devices/phone/`、`configs/phone/`、`assets/phone/` | `input.mock/replay`、`test_robot.mock`、`diagnostic.*`、`phone.tools` |

### 一处选择和运行

```bash
./run.sh components list
./run.sh components run calibration.handeye --help
./run.sh components run collection.groot --help
./run.sh components run diagnostic.phone_reset_console --help
./run.sh components run test.rebot
./run.sh components run ui.build
```

`components list` 同时列出插件与可执行操作。工厂登记在 `configs/components/migrated.json`，命令/环境登记在 `configs/components/commands.json`。可执行参数原样传递，不通过 shell 拼接；列举不会加载硬件 SDK。插件创建仍通过 `embodied.contracts.registry.Registry`，例如在 analysis 环境：

```python
from embodied.contracts.registry import Registry
planner = Registry().create("trajectory", "ruckig", runtime="analysis")
# 使用 planner.plan_ruckig_joint_trajectory(...)，显式提供模型限制和起始状态。
```

库插件返回原有的模块 API 或带类型的算法对象；操作插件通过独立进程执行一个功能。它们与 `robot` worker 接口分别建模，避免把仅登记过的 B601 驱动误称为已经接入 X5 worker。B601 的 DM/RS 模型、轴映射与控制环境保持独立，均无需旧目录。

### 控制边界

现有 `workbench` 的 Quest/手机/基础动作继续共享 arx gateway。迁入的网页控制、双相机自动标定、方块任务和 GR00T 实机操作保留原操作实现及其控制器，通过本仓库的独立操作入口调用；**这些入口尚未全部改成共享 arx gateway 的客户端**。这次完成的是源码与操作入口的集中交付，并保留可分别优化的算法/工作流边界。

这些操作与原生 arx worker、手机原 ARX 后端、B601 设备适配已统一合作式设备锁。同一设备被占用时，另一个控制实现连接会失败，需先结束原操作。它们不是同一运行内的无缝来源切换；现有工作台提供的来源切换仍按原规则使用。底层电机维护示例需要单独的维护时段。

软件停止复位、阻尼、Home、示教保持的不同语义随各操作保留。默认手机工作台仍使用锁存停止；迁入的双端复位工作流存在于手机扩展模块，不自动放开默认网关的 RESET。

## 环境与资源

环境在本仓库内重建，互不覆盖：

```bash
./run.sh components setup analysis   # 标定/规划/后台，Python 3.12
./run.sh components setup web        # 相机/视频/导出，Python 3.12
./run.sh components setup phone      # 手机独立工具
./run.sh components setup rebot      # B601/motorbridge/Pinocchio，Python 3.10
./run.sh components setup frontend   # Vue/Three.js
./run.sh components setup groot     # GR00T 模型/数据工具
./run.sh components setup groot-tensorrt  # 可选部署加速；基础导出/模型工具不需要
```

前三个参考项目的原始锁文件在 `configs/environments/`。GPU 模型权重仍按明确的资源路径选择，不随启动下载；共享的数据和权重根目录仍为现有 `~/work/data/arx5` 与 `~/work/pre_weights`，不依赖已移走的三个源码目录。浏览器模型/纹理已进入 `assets/arx_motion/`，B601 网格已进入 `assets/robots/rebot/`。历史标定记录保存在 `outputs/calibrations/arx_motion/`，旧 session/标定板路径已重定位，原始字节仍保存在恢复归档中。

GR00T 导出使用独立的双相机/Rot6D/16 维数据定义，关节 action 来自下一帧测量状态；现有 schema 2 的 sent-action 导出不变。两类数据不能仅改 schema 名称混装。

## 来源、修复和验证

`third_party/migration/manifest.json` 逐项记录 1,297 个原文件、目标位置、原始 SHA256 与导入状态；`patches.json` 记录迁入后的路径适配和修复。已有实现遇到同名文件时保留本地版本，原文件仍可由恢复归档核查。GR00T 依赖来源另记在 `groot.json`。迁移已完成，这两份清单作为历史记录保留；迁入代码此后按平台代码正常修改，不再校验与导入时逐字节一致（arx-televr 的 `control/` 仍与上游同步，保留哈希台账）。

迁移中修复了 reBot 梯形时间剖面越界/不连续、关节限位用 joint ID 错位索引、MeshCat 轨迹数组方向和 Pinocchio 重力读取；自动保持积分按 dt 换算原 500 Hz 增益。运动学与动力学库加载不再隐式导入电机 SDK。相关能力仍须按各机器人实际设备验收，B601 特定增益不应用到 X5。

离线证据位于 `outputs/migration/consolidation/`：迁入测试 328 项通过；迁移契约 5 项通过；现有项目通过正式启动入口回归 302 项、4 项跳过；DM/RS 模型与动力学、梯形轨迹检查通过；网页完成 TypeScript/Vite 构建；原生 arx demo、采集及终止封存检查通过；手机实际本机 UDP/mock 的握手、追踪恢复和失效检查通过。

`groot-synthetic-final/` 使用真实编码器生成双路视频并导出 121 帧 Parquet，核对 16 维状态/动作以及下一帧测量关节动作；普通统计、相对统计与官方 GR00T loader 均通过。没有伪造真实设备数据或模型预测。TensorRT 依赖已拆为可选 extra，版本和补丁由 `third_party/migration/groot.json` 及 `groot-tensorrt.patch` 固定，`components setup groot` 能从锁定提交恢复源码和补丁。

`isolated-checkout/` 从迁入源码、资源和锁文件重新构建了 analysis/rebot 环境，328 项迁入测试与 B601 检查再次通过。运行时禁止访问三个原目录、恢复归档，以及主工作树的源码/配置/模型，验证不会暗中借用旧实现。

迁移收尾另验证了 B601 正运动学执行、逆运动学启动、三个 MeshCat 入口导入，以及手机 golden 生成器与 Python/iOS 两套样本逐字节一致；相关契约与协议测试 13 项通过，入口记录见 `entrypoints-final.log`。

真实 Quest/iPhone、CAN、夹爪、相机和标定精度尚未现场验收；Linux 上没有执行 iOS 编译。原目录可保持退役状态，后续集成与设备验收均在本仓库进行。恢复归档没有删除，日常无需维护其中的副本。
