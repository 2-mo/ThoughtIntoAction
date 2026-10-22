# 冗余清单

逐项去掉重复实现。每项写明保留哪一份、删除条件和关联条目。删除前运行引用检查（G-05），一次一个小提交。完成后勾选并写提交号。

## 可直接删除

以下各项审查时已核对：在 `src/`、`control/`、`scripts/`、`configs/`、`apps/` 中没有运行时引用，只出现在 `third_party/migration/manifest.json` 中。

- [x] **X-01** `src/operations/vendor/arx_motion/{contracts,geometry,interpolation,limits}.py`（f1f0425）
  - 与 `src/arx5_control/motion/` 中的同名文件逐字节相同，没有任何导入。
  - 内核只使用 `profiles.py`（`src/embodied/control/commands.py:262,271`），该文件不依赖包内其他模块，并带有 Python 3.10 的 cbrt 补丁，保留。
  - 同步更新 `PROVENANCE.json`。

- [x] **X-02** `tests/migrated/phone/golden/`（0956677）
  - 与 `tests/golden/` 逐字节相同。`scripts/phone/generate_golden.py` 写入 `tests/golden`，iOS 工程也引用 `tests/golden`。
  - 两份 `test_golden_vectors.py` 同名，会造成 pytest 收集冲突。

- [x] **X-03** `src/backend/`（e479344）
  - 只有 `__init__.py` 和一份 README，README 中是过时命令（`.venv-handeye`、`-m backend.arx5_backend`）。
  - 它与 `src/infer_after_sft/backend.py` 同名，目前因脚本目录优先而没有冲突。

- [x] **X-04** `src/quest_teleop/auto_dataset.py`（89a90f1）
  - 没有引用。其中的 `finalize_task` 与 `task_package.py` 中的同名函数重复。

- [x] **X-05** `src/arx5_control/{trajectory,ruckig_motion}.py`（89a90f1）
  - 兼容层，没有引用。

- [x] **X-06** `src/arx_tools/legacy/{move_slightly,read_state}.py`（58eb062）
  - 不拿设备锁，直接打开 SDK（C-03）。

- [x] **X-07** `src/robot_core/interfaces.py`（89a90f1）
  - 没有引用。`ArmBackend` 中带有 X5 夹爪专用方法，不适合作为平台契约。

- [x] **X-08** `src/下载JAX版本.py`（f479fbd）
  - 这是下载脚本，不属于 `src/`；移到 `scripts/` 或删除。

## 替代完成后删除

| ID | 冗余 | 保留 | 删除条件 | 关联 |
| --- | --- | --- | --- | --- |
| X-10 | ~~`src/handeye_calibration/` 与 `src/hand_eye_calibration/`~~ 已完成（2026-10-22） | `embodied.applications.calibration`（一个求解器，腕部 / 固定两种安装） | 旧 D405/D455 采集器在 `src/legacy/hand_eye/`，K-10 删除 | C-06 |
| ~~X-11~~ 已完成 | `src/robot_core/device_lease.py` 与 `src/embodied/contracts/device_lease.py`（同一锁键） | 共享包中的一份 | K-03 共享包建立 | K-03 |
| X-12 | `src/robot_control/gripper_turns.h` 与 `src/embodied/devices/robots/arx5/vendor/arx5-sdk/gripper_turns.h`（当前内容相同） | `src/embodied/devices/robots/arx5/vendor/` | 旧链路退役，`scripts/setup/build_gripper_sdk.py:33` 改指向 | K-10 |
| X-13 | `scripts/setup/can_up.sh`（57 行）与 `src/arx_tools/can_up.sh`（191 行，引用不存在的 `./arx5`） | 合并为一个 | 网页启动脚本不再调用旧版本 | B-02 |
| X-14 | `src/quest_teleop/{recording,mcap_io,video}.py`（只是 `sys.modules` 别名） | `src/episode_io/` | 导入方改为直接导入 `episode_io` | D-06 |
| X-15 | `src/robot_teleop/` 中只服务旧 CLI 的部分：`robots/arx5*.py`、`runtime/*`、`registry.py`、`cli.py`、`config.py`、`controllers/`、`kinematics/` | 集成路径用到的约 2.3k 行：`core/{models,mapping,frames,capabilities,ports}`、`inputs/arkit`、`policies/relative_pose`、`protocol/*`、`transports/udp`，与 `src/inputs/phone.py` 合为一个输入包 | 安全检查与急停复位已移植到 gateway | C-04 |
| X-16 | ~~`scripts/phone/*.py`（5 行包装）与 `src/robot_teleop/tools/*`~~ 已完成：包装删除（只留 `generate_golden.py`），`phase3_mock_validation`（与 `test_arkit_udp_to_mock` 重复）、`phase4_ghost_review`、`phase5_{hold_current,safe_stop}` 删除 | 网络诊断与离线检查经 `./run.sh components run diagnostic.*`；`phase3_mock_session` 改名为 `diagnostic.phone_reset_console`，C-04 复位握手进网关后删除 | — | C-03 |
| X-17 | 旧控制内核：`src/robot_control/{service,client,hardware,online_motion,motion,trajectory,console,startup,cli}.py`、`backends/`；`quest_teleop` 的控制部分；XRoboToolkit placo IK | `control/` | 真机切换完成，`ArmClient` 没有使用者 | K-10 |
| X-18 | `src/quest_teleop/legacy/`、`src/quest_teleop/experimental/`、`src/robot_control/backends/arx2025.py`（命令 `teleop` / `teleop-dry-run`，`scripts/launch.py:57-58`） | — | 确认不再使用 | K-10 |
| X-19 | `src/robot_core/collection_motion.py` | 机器人或平台配置 | 策略与恢复流程改读配置 | R-04 |
| X-20 | IK 5 套：SDK KDL（`arx5_control/cartesian.py:222`）、Pinocchio（`robot_core/model.py`、`robot_teleop/kinematics/pinocchio.py`、`arx5_control/motion/pinocchio_backend.py`）、MuJoCo（`src/embodied/devices/robots/arx5/kinematics.py`）；以及 reBot 自己的 | 内核用上游 MuJoCo IK；离线 FK / 规划保留一个 Pinocchio 包装，读取 `assets/robots/x5/runtime` | 使用者迁移完成 | R-03、C-07 |
| X-21 | 在线 Ruckig 封装 3 套 | `arx5_control/motion/online.py`（如仍需要） | 旧服务与手机旧 CLI 退役 | C-07 |
| X-22 | 相机服务 2 套，另有 4 处直接打开 RealSense | `robot_core/camera_service.py` | WebRTC 改为消费者 | S-01、S-02 |
| X-23 | MCAP 写入 / 读取各 2 套，对齐 5 套，schema 1 校验 2 套 | 统一的 `episode_io` | 导出回归通过 | D-05 – D-07 |
| ~~X-24~~ 已完成 | `quest_teleop/export_lerobot.prepare_episode` 及 `tests/test_training_export.py:40-71` | — | 确认生产路径不使用（测试覆盖的路径与生产不一致） | D-05 |
| X-25 | GR00T modality 配置 4 份（`isaac_groot_data_collection/{,eef_only_,wrist_eef_only_,wrist_only_}modality_config.py`），都登记为 `NEW_EMBODIMENT` | 由 spec 生成 | 导出器插件化 | D-08 |
| X-26 | X5 URDF：`assets/robots/x5/`、`assets/phone/robots/arx5/`，以及 SDK 自带模型的引用 | `assets/robots/x5/runtime/` | 使用者全部改为引用内核模型 | R-03 |
| X-27 | 重复资产：X5 网格 4 份、纹理 2 份、three.js 3 份 | 一份 | 引用改为同一路径 | O-05、U-03 |
| X-28 | `src/arx5_paths.py`（17 个文件引用） | 平台配置 | 引用方迁移完成 | R-05 |
| X-29 | 注册表 4 套（见 B-03） | `component_api.Registry` 加一份数据文件 | 注册表合并完成 | B-03 |
| X-30 | 前端 5 套（见 U-01） | 选定的一套 | 功能并入完成 | U-01 |
| X-31 | `docs/imported/` 与迁移记录 | `docs/history/` 或 git 历史 | docs 重组完成 | O-03 |
| X-32 | `configs/environments/{arx_motion,phone}` | 平台环境 | 包化完成 | B-04 |
