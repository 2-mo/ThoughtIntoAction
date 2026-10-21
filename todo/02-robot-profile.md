# 机器人参数与模型

X5 的参数和模型以 arx-televr 为准：`control/config/robot.json`、`control/config/tool.json`，以及由 `control/scripts/build_model.py` 生成并带校验值的 `control/models/x5/`。平台其他位置引用它们，不再复制。

- [x] **R-01** 确立唯一来源 `P1`
  - 已做：`embodied_contracts.robot.load_robot_profile()` 读取 `control/config/robot.json` 与 `workspace.json`（附来源与 sha256）；运行中的内核通过 `describe` 返回同样的值，`check_commands.py` 逐字段比对两者。R-02 的使用者迁移仍待做。
  - 做法：
    - 共享包提供读取函数，返回 robot / tool / model 的解析结果，附来源路径和哈希。
    - 平台配置（`configs/`）只引用，不复制。
    - 上游 JSON 格式保持不变，便于同步。
  - 完成：下面 R-02 列出的重复项都改为读取这里。

- [ ] **R-02** 收敛重复参数 `P1`
  - 现状：

    | 参数 | 出现位置与取值 |
    | --- | --- |
    | 控制频率 | 250 Hz（`control/arx5_vr/types.py:59`、`robot.json`）；240 Hz（`configs/project.toml:54`）；200 Hz（`configs/phone/profiles/arkit_arx5.yaml:9`、`src/robot_teleop/robots/arx5.py:74`） |
    | 关节限位 | 至少 5 份：`types.py:62-63`、`robot.json`、`configs/phone/robots/arx5.yaml:25-26`、`robot_control/backends/arx2025.py:18-19`、`src/zero-shot/test_infer.py:18` |
    | Home / 预抓取 | `robot_core/presets.py:2-3`（预抓取 `0,1.5,1.45,-1.52,0,0`）；`robot.json` 的 `park_q` |
    | 夹爪最大宽度 | 0.082（`robot.json`、`project.toml`）；0.078 打开上限（`project.toml:62`）；0.088（`arx5_control` 使用 SDK 出厂值） |
    | 运动默认值 | `robot_control/online_motion.py:5-7`、`project.toml:44-51`、`启动遥操.sh:23-32`、`quest_teleop/cli.py:28-31`，指令超前量 0.15 与 0.05 不一致 |
    | CAN 接口 | `"can0"` 在 30 个文件中硬编码 |
    | 桌面平面 | `control/config/workspace.json`（内核）与 `configs/project.toml` `[arm_safety]`（旧服务）取值相同；旧服务退役（K-10）后只保留前者 |

  - 做法：内核相关参数读 R-01；只属于平台的参数（预抓取位、采集速度等）放平台配置。
  - 完成：G-04 的 `"can0"` 白名单清空。

- [ ] **R-03** 收敛 X5 模型文件 `P1`
  - 现状：
    - `assets/robots/x5/X5_2025_hybrid.urdf`：`robot_core/project.py:11` 默认，`src/zero-shot/infer.py:66` 使用。
    - `assets/phone/robots/arx5/X5.urdf`：`configs/phone/robots/arx5.yaml:12`；joint1 限位 ±10 rad，joint2/3 为 0–3.14。
    - SDK 自带 `models/X5.urdf`：`src/arx5_control/controller.py:684`、`src/quest_teleop/legacy/controller.py:26`。
    - `control/models/x5/x5_2025.{source.urdf,urdf,xml}`。
    - 已核对各 URDF 的关节原点一致；差异在位置 / 速度限位，以及 TCP 关节名（`gripper_fixed_joint` 与 `tool_tcp_fixed`）。
  - 做法：FK、Ghost、标定、导出、离线规划都使用 `control/models/x5/` 产物；其他 URDF 在使用者迁移后删除（X-26）。
  - 完成：其他模块对 X5 模型的引用只剩 `control/models/x5/`。

- [ ] **R-04** 策略运动参数不再解析启动脚本 `P2`
  - 复核（2026-10-21）：风险低于审查时的判断。策略预抓取与遥操 B 键预抓取使用同一组 preset 速度、加速度和 jerk（2.0 rad/s，由机械臂服务的 Ruckig 规划），并非不受限的硬件速度。剩下的只是解析 shell 脚本这一耦合，随策略改为内核客户端（P-06）、旧链路退役（K-10）一并消除。按"安全控制统一在内核"的原则，不给策略单独设保守速度。
  - 现状：`src/robot_core/collection_motion.py:17-21` 用正则读取 `启动遥操.sh`，`src/infer_after_sft/run.py:73` 和 `src/zero-shot/recover.py:21` 用它决定预抓取运动。脚本当前 `MAX_JOINT_SPEED_RAD_S=0`，即 SDK 硬件上限（`启动遥操.sh:20`）；README 仍写 2.0。
  - 做法：
    - 短期：给策略流程单独设定有限的预抓取速度，放平台配置。
    - 内核切换后从内核 `describe()` 或 R-01 读取限制。
    - 删除 `collection_motion.py`（X-19）。
  - 完成：修改 `启动遥操.sh` 不再影响策略或恢复流程的运动。

- [ ] **R-05** 硬编码路径、端口和相机序列号 `P2`
  - 端口已完成（2026-10-21）：`configs/project.toml` 的 `[ports]` 是唯一登记表（监看页与旧遥测沿用 `[viewer_service].port`、`[telemetry].endpoint`），控制台后端、手势、相机信令、采集信令与手机 UDP 的默认端口从这里读取；`./run.sh check ports` 显示占用进程与冲突，工作台启动时提示冲突，总控页显示端口状态。`apps/console` 前端的 8765 / 8767 仍写死，随 U-01 处理。路径仍待做。
  - 相机序列号已完成（2026-10-22）：现有相机是腕部 D435i `405622074531` 与固定 D435 `335522070264`（`configs/local.toml` 的 `wrist_rgb` / `exterior_rgb`）。GR00T 与 WebRTC 相机服务改为按相机 id 读取配置中的序列号，代码中过期的 D405 / D455 序列号已删除（只剩 `src/legacy/`）；GR00T 经内核执行时从共享相机服务取帧。
  - 现状：
    - `/home/r2`：`src/infer_after_sft/checkpoint.py:8,53`、`src/zero-shot/infer.py:101`、`src/zero-shot/policy_worker.py:25`、`apps/console/src/features/collection/runtime.ts:215`、`apps/console/src/features/collection/shell.html:184`。
    - `~/work`：`src/arx5_paths.py:11-13`（被 17 个文件引用）、`scripts/datasets/convert.sh:4`、`src/zero-shot/compute_stats.py:19`。
    - 端口 8000、8765–8768、5555、5567、5173、18081 分散在代码和脚本中；`arx5_backend/core/settings.py:10` 默认 8766，`src/arx_tools/web.sh:48` 却传 8000。
    - 相机序列号：`arx5_backend/features/camera/service.py:84-85`、`arx_tools/d405_profile_benchmark.py:33`、`hand_eye_calibration/configs/x5_d455_eye_to_hand.json:9,16`、`experiments/cube_pick_rotate_place/config.json:6`。
  - 做法：平台配置统一管理存储根目录、端口和相机，本机差异放 `configs/local.toml`。

- [ ] **R-06** 配置格式与加载器 `P2`
  - 现状：
    - 至少 10 个加载器：`robot_core/project.py`、`scripts/launch.py:200-203`、`component_api/registry.py`、`component_plugins/cli.py`、`workstation/workbench.py`、`control/arx5_vr/settings.py`、`robot_teleop/config.py`、`robot_teleop/kinematics/config.py`、`arx5_paths.py`、`arx5_backend/core/settings.py`，以及 experiments、isaac_groot、hand_eye 的各自加载。
    - 3 种 profile 概念：启动参数（TOML）、工作台组合（JSON）、手机遥操（YAML）。
  - 做法：机器人配置保持上游 JSON；平台配置一种格式、一个加载器；profile 只保留工作台组合一种。
