# 平台框架（阶段 1 草案）

状态：**已确认（2026-10-21）**。B-01、K-03、C-01、R-01、B-02 按本文实施。第 8 节各项已按建议决定，平台包名为 `embodied`。

## 1. 分层与依赖方向

| 层 | 内容 | Python | 允许依赖 |
| --- | --- | --- | --- |
| L0 契约 | 协议常量与 schema、RobotTick 布局、RPC 帧格式、内核客户端、设备锁、注册表、运动 profile、机器人配置读取 | ≥ 3.10，只依赖标准库与 numpy | — |
| L1 内核 | `control/`：arx-televr 原样，加 `adapters/` | 3.10（与上游一致） | L0 |
| L2 平台 | 输入、动作、规划、传感器、数据、策略执行、标定、感知、网页后端、工作台 | 3.12 | L0；同层之间只交换 L0 定义的数据 |
| L3 入口 | 统一 CLI、根目录中文别名脚本、前端 `apps/console`、手机端 `apps/ios` | — | L2、L0 |
| 推理服务 | 各 GPU 模型后端，独立进程 | 各自环境 | L0 |

规则：

- 下层不导入上层。L1 只导入 L0（G-04 的 `control_imports_src` 收敛到只剩 L0）。
- 只有 L1 打开机械臂 SDK（G-04 `sdk_controller`）；相机只由传感器服务打开（S-01）。
- 安全控制只在 L1：限位、速度、工作空间、反馈检查、停止由 worker 与 gateway 统一实施，对所有来源生效。L2 客户端不另做一套安全检查。
- L2 的功能包之间不导入彼此的业务状态机：采集不导入策略，标定不导入采集，等等。
- 现有 `tests/test_package_boundaries.py` 的导入探测，按新包名扩展为上述规则。

## 2. 目录

```text
control/                    L1：arx-televr 上游布局（arx5_vr/ config/ models/ native/ scripts/ vendor/）+ adapters/
contracts/                  语言无关的 schema 与金样本：手机 UDP、commands.sock、RobotTick、episode
src/embodied_contracts/     L0 Python 包；B-01 前随 src/ 一起加入路径，control/ 在本仓库中从这里加载，打包后从 shared/ 加载
src/<平台包>/               L2 Python 包（根 pyproject，package = true）
apps/console/  apps/ios/    L3 前端与手机端
policies/<backend>/         推理服务：openpi、lerobot_pi05、groot，各自声明环境
configs/                    平台配置；机器人参数引用 control/config，不复制
scripts/                    安装、设备准备、一次性诊断
tests/                      按层分目录：contracts/ platform/ …；control 自带检查留在 control/scripts
```

- L0 目前放在 `src/embodied_contracts/`：现有各环境（含系统 Python 运行的工作台与相机服务）都以 `src` 为路径，无需新增路径配置。B-01 包化时再给它独立 `pyproject.toml`，`control/` 改用路径依赖，去掉 `sys.path` 查找。`control/` 保持独立 uv 项目（Python 3.10 与平台环境不能共锁）。
- 推理服务各自一个环境，只依赖 L0。机械臂侧不导入 torch / jax。

## 3. L0 契约包 `embodied_contracts`

全部来自现有代码，只依赖标准库或 numpy，兼容 3.10。前五项已迁入（2026-10-21）：

| 内容 | 来源 |
| --- | --- |
| `registry`：注册表（按 `configs/plugins.toml` 向上查找仓库根） | 原 `src/component_api/registry.py` |
| `rpc`：RPC 帧格式 | 原 `src/component_api/rpc.py` |
| `interfaces`：Protocol 定义 | 原 `src/component_api/contracts.py` |
| `runtime`：`ControlClient`、`ControlSession`、`RuntimeReader` | 原 `src/runtime_client/` |
| `device_lease`：设备锁（控制内核与平台同一实现，X-11） | 原 `src/robot_core/device_lease.py` 与 `control/adapters/device_lease.py` |
| `vendor/arx_motion/profiles`：运动 profile（smoothstep9） | 原 `src/operations/vendor/arx_motion/` |
| 机器人配置读取 | 新增：读取 `control/config/{robot,tool}.json` 与 `control/models/x5/`，返回限位、`park_q`、夹爪、CAN、频率、模型路径与哈希（R-01） |
| 协议常量与 schema 绑定 | 新增（C-01） |

## 4. 内核接口

**控制面** `commands.sock`：长度前缀 JSON，`version: 1` 加 `runtime_id` 封装（`control/adapters/gateway.py`）。现有动作共 14 个：

| 类别 | 动作 |
| --- | --- |
| 查询 | `status` |
| 会话 | `acquire`、`source-metadata`、`heartbeat`、`release` |
| 控制 | `hold`、`cancel`、`stop` |
| 计划 | `preview`、`execute` |
| 流式 | `free-drive`、`joint`、`cartesian`、`gripper` |

C-01 为它们写 schema 与金样本。新增 `describe` 动作，返回机器人配置、能力和限位，供客户端读取，替代各处复制的参数。

**状态面**：observer 发布 RobotTickV2（`control/adapters/robot_tick_v2.json`），客户端只通过 `RuntimeReader` 读取。

**操控语义**：模式、Grip 跟随、Trigger、IK、增益、Home、退出以上游为准。平台新增的检查（例如非原生来源的工作空间约束，C-04）只在 gateway 中实现。

## 5. L2 平台包

平台包名为 `embodied`，下表中的 `<pkg>` 即 `embodied`。

| 子包 | 职责 | 主要来源 | 不再迁入（阶段 3 后归档） |
| --- | --- | --- | --- |
| `<pkg>.config` | 平台配置：端口、存储根、相机 | `robot_core/project.py`、`arx5_paths.py` | — |
| `<pkg>.inputs.phone` | 手机协议、相对映射、gateway 源 | `robot_teleop` 集成子集（约 2.3k 行）、`src/inputs/phone.py` | `robot_teleop` 的旧 runtime / robots / CLI |
| `<pkg>.inputs.quest` | Quest 发现、USB、电量等平台侧工具（输入本身在内核） | `scripts/quest/` | `quest_teleop` 的控制器与预设 |
| `<pkg>.operations` | MoveJ / MoveL、点动、示教点（内核客户端） | `src/operations/` | — |
| `<pkg>.motion` | 离线规划、时间参数化、碰撞，不依赖 SDK | `arx5_control/motion/` | `arx5_control` 控制器与 servo、`robot_control` 轨迹 |
| `<pkg>.sensors` | 相机服务与帧契约 | `robot_core/camera_*` | `arx5_backend` 中自持设备的相机服务 |
| `<pkg>.data` | episode 读写、对齐、采集、质量、导出插件 | `episode_io`、`observations`、`collection`、`dataset_tools`、`quest_teleop` 的数据部分、GR00T 导出 | 旧 `TeleopSession` |
| `<pkg>.policy` | spec、观测、执行器、内核 sink | `zero-shot`、`infer_after_sft`、`gr_zeroshot` 的机器人侧 | `ArmClient` 与直连执行路径 |
| `<pkg>.calibration` | 手眼、夹爪、负载 | `hand_eye_calibration`、`handeye_calibration`、`arx_tools/calibrate_*` | — |
| `<pkg>.perception` | 方块检测等 | `arx_tools/rgb_cube_detector.py`、实验感知 | — |
| `<pkg>.web` | HTTP / WS 后端：监看与操作 | `arx5_backend` 的 protocol / transport 外壳、`robot_simulation` 场景服务 | `arx5_backend/kernel/control.py` |
| `<pkg>.workstation` | 进程编排、工作台、统一 CLI | `workstation`、`scripts/workspace`、`scripts/launch.py` | — |

## 6. 统一入口

```text
./run.sh <组> <命令>
  control    start / stop / verify / check / doctor / observe     内核
  teleop     quest / phone / cli                                   控制来源
  motion     preview / movej / movel / jog / teach                 动作
  camera     serve / check
  collect    start / ...                                           采集
  data       validate / review / export                            数据
  policy     serve / run / preview                                 推理
  calib      handeye / gripper / payload
  web        serve
  workbench  start / status / stop                                 组合启动
  check      test / all                                            离线检查
  setup      deps / sdk / can / udev                               安装与设备准备
```

根目录中文脚本保留为一行别名。现有 49 个 launcher 子命令和 52 个 components 命令逐步映射进来，映射表随 B-02 维护。

## 7. 迁移方式

- 阶段 3 之前，旧真机路径涉及的包（`robot_control`、`quest_teleop`、`robot_core` 中的业务部分）原地不动，只修复 P0。
- 新包从保留的模块迁移而来，使用 `git mv` 保留历史。被旧路径导入的模块迁走时，在同一提交中更新旧路径的导入，不留转发层。
- 每迁一个子包：
  - 更新 G-04 名单；
  - 运行 `./run.sh check-all --only` 中相关的组；
  - 在 `redundancy.md` 勾掉对应条目。

## 8. 决定（2026-10-21，均按建议）

| 项 | 选项 | 决定 |
| --- | --- | --- |
| 平台包名 | 例如 `embodied`、`questarm` 或其他 | `embodied` |
| K-04 按键映射 | ① 恢复上游 A / A+B 语义，采集另配；② 保留工作台启用门并登记为适配 | ①，与"操控参考 arx-televr"一致 |
| K-06 行为差异 | 逐行见 `01-control-kernel.md` | 原生 Quest 路径全部按上游；工作空间约束只在 gateway 中对非原生来源生效 |
| U-01 唯一前端 | ① `apps/console`；② 保留 `robot_simulation/web` 做监看 | ①，Vite + TS，后端协议与内核一致后合并监看页 |
