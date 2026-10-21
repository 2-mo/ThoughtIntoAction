# 内核客户端

除内核外，所有功能都通过同一协议访问机械臂：

- 控制面：`commands.sock`（`control/adapters/gateway.py`、`commands.py`）；
- 状态面：只读 observer 发布的 RobotTickV2；
- 客户端库：`src/runtime_client/`（`ControlSession`、`ControlClient`、`RuntimeReader`）。

- [ ] **C-01** 协议说明与金样本 `P1`
  - commands.sock 已完成（2026-10-22）：`embodied.contracts.protocol` 以数据描述全部 15 个动作的租约要求、字段、类型、单位与结果字段，生成 `docs/reference/kernel-protocol.md`；`tests/golden/kernel/samples.json` 为每个动作的真实请求与结果；`tests/test_kernel_protocol.py` 核对表与 gateway 的动作列表、金样本、`ControlSession` 发出的请求和文档；`scripts/check_kernel_protocol.py`（`sim` 组）在 MuJoCo 内核上逐个动作核对结果字段，`--update` 重新生成金样本。待做：observer / RobotTickV2 的类型与字节样本。
  - 先前：gateway 增加只读 `describe`（不需租约或反馈；返回机器人限位、频率、`park_q`、夹爪、CAN、模型 ID、工作空间和动作列表），客户端 `ControlClient.describe()`。schema 与金样本仍待做。
  - 现状：
    - 手机 UDP 协议有版本号，并有 Python / Swift 共用金样本（`tests/golden`），是仓库内的最好做法。
    - `commands.sock` 只有 `version:1` 封装，没有 schema 文件，动作在代码中逐个检查。
    - RobotTickV2 只有布局 JSON（`control/adapters/robot_tick_v2.json`），没有类型、单位或字节样本。
    - observer 描述没有测试；`src/robot_core/session_protocol.py` 没有版本字段。
  - 做法：为 commands / observer / RobotTick 写 schema 与金样本，参照手机协议，放共享位置。`src/component_api/contracts.py` 目前没有运行时引用，可作为 Python 侧契约起点。

- [ ] **C-02** 消除 SDK 直连 `P1`
  - 目标：`Arx5JointController(` 只出现在 `control/arx5_vr/real.py`。审查时的位置：

    | 位置 | 使用者 | 处理 |
    | --- | --- | --- |
    | `src/robot_control/hardware.py` | 旧真机服务 | K-10 |
    | `src/arx5_control/controller.py:703`（`ArmController`） | 网页内核、手眼 collector、方块实验、GR00T、`arx_tools` 6 个脚本、`isaac_groot_data_collection/automated_stack_probe.py` | C-05、C-06、C-09、P-08 |
    | `src/quest_teleop/legacy/controller.py:56` | `teleop` / `teleop-dry-run` 命令 | X-18 |
    | `src/arx_tools/calibrate_gripper_guided.py` | 夹爪标定，锁路径错误 | C-03 |
    | `src/arx_tools/legacy/move_slightly.py:276` | 不拿锁 | X-06 |
    | `src/arx_tools/legacy/read_state.py:23` | 不拿锁 | X-06 |
    | `src/robot_teleop/robots/arx5.py:358`（importlib） | 手机旧 CLI | X-15 |

  - 完成：G-04 的白名单只剩 `real.py`。

- [x] **C-03** 设备锁修复 `P0`
  - 已做：`calibrate_gripper_guided.py` 改用 `robot_core.device_lease.DeviceLease`（网页内核在启动它之前已要求断开机械臂，原流程不受影响）；phase5 两个工具在使用真实 SDK 时获取同一锁，注入测试控制器时不获取；legacy 两个脚本已删除（X-06）。离线验证：锁被占用时三者都拒绝，且不加载 SDK。未真机验证。
  - 现状：
    - `src/arx_tools/calibrate_gripper_guided.py:21-30` 注释说使用"与控制器相同的锁"，实际锁的是 `/tmp/arx5-control-{if}.lock`。其他组件使用 `quest-x5-{uid}-{key}.lock`（`src/robot_core/device_lease.py:13`、`control/adapters/device_lease.py:15`），所以它与机械臂服务、内核都不互斥。
    - 网页内核以子进程启动它（`src/arx5_backend/kernel/control.py:1957-1961`）。
    - 另有 4 个脚本完全不拿锁（见 C-02 表）。
  - 做法：
    - `calibrate_gripper_guided.py` 改用 `robot_core.device_lease.DeviceLease`；
    - legacy 两个脚本删除（X-06）；
    - phase5 两个脚本删除，或用 `component_plugins.ownership.leased_operation` 包裹。
  - 完成：所有打开 SDK 的入口使用同一锁。

- [ ] **C-04** 手机接入的剩余缺口 `P1`
  - 工作空间已完成：`control/adapters/workspace.py` 读取 `control/config/workspace.json`（与旧 `[arm_safety]` 相同：桌面 z=0、余量 0），gateway 在请求线程中检查流式 `cartesian` / `joint` 目标，规划器检查 MoveJ / MoveL / Home 每个采样点；对所有经 gateway 的来源统一生效，原生 Quest 路径不变，机器人线程不增加计算。预抓取与 Home 路径最低 TCP z 为 0.156 m，不受影响。速率由现有 0.15 m 超前与每 tick 关节步长限制。未真机验证。
  - 仍待做：手机双端急停复位握手（下方）。
  - 现状：手机已通过 gateway 接入（`src/inputs/phone.py:49` 使用 `ControlSession`），不再持有 SDK。但原 robot-teleop 的检查在集成路径上不生效：
    - 工作空间（z ≥ 0）与目标速率门不生效；gateway 对笛卡尔目标只检查四元数归一化和 0.15 m 超前（`control/adapters/commands.py:151-156`），`control/` 中没有工作空间检查。
    - 双端急停复位被固定拒绝（`src/inputs/phone.py:55`，`estop_reset_request_callback=lambda _: False`）。
  - 做法：在 gateway 增加对非原生来源的工作空间与速率检查（安全控制统一在内核），桌面平面沿用 `configs/project.toml` 的 `[arm_safety]`，原生 Quest 路径不变（K-06）；急停复位握手按 `docs/plugin-migration-inventory.md` 的 C05 接入。
  - 依赖：K-06。

- [ ] **C-05** 网页后端改为内核客户端 `P1`
  - 已做（2026-10-21）：`embodied.console.kernel.arx.ArxConsole`（`./run.sh web console`，`embodied.console --kernel [--run <RUN>]`）沿用页面协议，经一个 `ControlSession` 驱动内核：关节 / 末端 XYZ / 回参考位为内核预览后执行（gateway 预览新增 TCP 路径 `tcp`，供页面画轨迹），夹爪、阻尼（free-drive）、拖动伺服与手势遥操为短 TTL 目标流；页面只接入运行中的内核，断开只释放租约（U-02）。`scripts/check_console_kernel.py` 在 MuJoCo 上经 `/api/v1/control` 验证（已加入 `sim` 组），真实页面已在浏览器中连接确认。顺带修复：WebSocket 适配层此前丢弃 `teleoperation.state` / `servo.state` 的 `ok`。
  - 差异：内核夹爪为位置控制，“限扭夹取”按位置流到停滞为止，不限力矩；状态中无关节力矩与增益（显示为 —）；`preview_xyz` 一律走内核 MoveL（页面的“平滑曲线”同为直线）。真机验收时确认。
  - 待做：负载 / 夹爪标定与方块演示仍在旧 `ControlKernel`（`./run.sh web legacy`，直连 CAN），随 K-10 与 `kernel/control.py` 一起删除；浏览器采集页由 `collect run` 替代。
  - 现状：
    - `src/arx5_backend/kernel/control.py` 共 6002 行。`ControlKernel` 约 100 个方法、50 个 `action ==` 分支，直接持有 `ArmController`，同时导入机械臂、手眼、方块和 GR00T。
    - `src/arx_tools/web.sh:48` 传入 `--auto-connect-arm`，启动网页即占用 CAN。
  - 做法：
    - 保留 `arx5_backend` 的 protocol / transport / FastAPI 外壳（`app.py`、`protocol/messages.py`）。
    - 各 feature 改为通过 `ControlSession` 访问内核，`ControlKernel` 按 feature 拆分。
    - 可先在仿真内核上完成。
  - 依赖：C-01。

- [x] **C-06** 手眼标定只保留一套 `P1`
  - 已做（2026-10-22）：用户确认可以重新标定，重点是代码清晰好用。
    - `embodied.calibration`（target / solver / session / plan / arm / cli，约 700 行）：同一个 A X C = Z 求解器处理腕部（`T_eef_camera`）与固定相机（`T_base_camera`）；机器人位姿取内核的 `tcp_link`；自动采样在示教的中心姿态周围生成 26 个关节目标，每步由内核预览，被拒绝或看不到标定板的姿态跳过；结果按相机 id 存于 `calibrations/hand_eye/<id>.json` 并绑定序列号。入口 `./run.sh calib handeye marker | capture <相机> [--auto] | solve | status`，说明见 `docs/guides/calibration.md`。
    - 采集的 episode 记录每个相机的外参（`episode.json` 的 `hand_eye`），导出摘要随之带出；网页控制台用保存的结果在三维场景中显示相机。
    - 旧 D405 / D455 浏览器采集器（5.3k 行）移到 `src/legacy/hand_eye/`，仍由 `web legacy` 使用，随 K-10 删除；为它写的内核适配（KernelArm、HandEyeFlow）不再需要，已删除。`calibration/wrist` 并入新包（X-10）。
    - 验证：`tests/test_calibration.py`（两种安装方式、计划、静止采样、结果、CLI）；`check_workbench.py --calibration`（`sim` 组）在 MuJoCo 内核上完成 26/26 个预览后执行的采样位并通过留出验证。
  - 待做：网页控制台的标定页随 U-01 在新包上重做（目前页面提示使用 CLI）。

- [ ] **C-07** 运动规划库 `P2`
  - 现状：MoveJ / MoveL 已由 gateway 执行。在线 Ruckig 封装有 3 套：`arx5_control/motion/online.py`、`robot_control/online_motion.py`、`robot_teleop/controllers/ruckig_servo.py`。停止到停止的 profile 分散在 `robot_control/trajectory.py`（五次）和 `arx5_control/motion/profiles.py`（smoothstep9，另有 vendored 副本）。
  - 做法：`arx5_control/motion/` 作为不依赖 SDK 的离线规划库，执行一律交给内核；碰撞检查（`motion/{collision,link_collision}.py`）是仓库中唯一的实现，予以保留。其余封装随使用者退役删除。

- [ ] **C-08** 策略执行 `P1`
  - 见 `06-policy.md` 的 P-06、P-08。

- [x] **C-09** 方块实验 `P2`
  - 决定（用户，2026-10-22）：只是演示功能，不迁到内核。`tasks/cube`、`console/features/cube/stack.py`、`data/groot/automated_stack_probe.py` 移到 `src/legacy/cube/`，只由旧控制台（`web legacy`）和 `./run.sh task cube` 使用，随 K-10 删除。

- [ ] **C-10** 第二种机器人（reBot B601） `P3`
  - 现状：
    - 只登记为 `device.rebot_dm/rs` 和分析组件（`configs/components/migrated.json`），没有工作台 profile。
    - `actuator/rebotarm.py` 没有关节限位保护。
    - 内核假设六维关节（`control/adapters/commands.py:148`、`control/arx5_vr/model.py`）。
  - 做法：保持隔离。等有带限位的 backend 后，作为第二个内核后端接入，用来验证多机器人抽象。
