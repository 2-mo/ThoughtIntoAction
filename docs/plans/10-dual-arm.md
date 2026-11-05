# 双臂内核（单内核 12 关节）

两台 ARX X5 同时由**一个内核进程**控制：左右手柄各驱动一臂，两臂同时录制。

## 现状（探索结论）

- **已经是参数化的**：tick V3 的 ABI 支持 1–15 关节（`MAX_JOINTS = 15`），capture、episode 写入、schema-2 校验、LeRobot 导出都按 `joint_count` 走；控制台 `arm_views()` 已能把 12 关节拆成两臂显示。
- **单臂写死的部分**：
  - `RobotConfig` 只有一组 6 关节参数和一个 `can_interface`，`load_profile` 拒绝非 6 关节；
  - `RobotState` / `ControlCommand` 只有一 个 `pose` 和一个夹爪标量；
  - `RealBackend` 一个 `Arx5JointController`；`MujocoBackend` + `X5Model` 是单臂 MJCF；
  - `gateway.describe()`、`commands.py`、`motion_limits.py` 都是单向量；
  - native 输入只订阅 `right_controller`，`NativeIngress` 一个 bridge。
- **vendor 能力**：`rollio-device-xrobot` 源码支持同时发布 `left_controller` 与 `right_controller`（配置即可，无需重编译）；`rollio-teleop-router` 一个进程一组 leader→follower，可多开；协调器支持 mode(右手) + episode(左手) 两种角色。
- **下游**：GR00T 16 维、π0.5 各适配器、标定采样都是 6/7 维写死，属于后续独立工作。

## 架构决策

单内核、每臂一组结构，保留单臂兼容：

| 层 | 改动 |
| --- | --- |
| 配置 | `RobotConfig` 增加 `arms: tuple[ArmConfig, ...]`（默认单臂）；新增 `configs/robots/arx5/dual.json` |
| 契约 | `RobotState` 增加 `poses: tuple[np.ndarray, ...]`、`gripper_widths: tuple[float, ...]`；`ControlCommand` 增加 `pose_targets`、`gripper_targets`。单臂时这些为 `None`，旧路径不变 |
| Tick | 新增 V4：`arm_count`、`pose[2][7]`、`gripper[2]`；V3 继续可读 |
| 后端 | `DualRealBackend`：两个 `Arx5JointController`（vendor 的 bimanual 示例已验证可行）；`DualMujocoBackend`：两臂模型，用于无硬件开发 |
| 运动学 | 每臂一个 `X5Model` + `JointTargetController`；复合后端按臂调用 |
| 网关 | `describe()` 返回 `arms[]`（每臂限位/夹爪/CAN）；命令带 `arm` 索引 |
| VR | `native_config.py` 增加 `left_controller` 通道；右→arm1、左→arm2；两个路由组（arm/arm2、gripper/gripper2） |
| 采集 | episode contract 的每臂 pose/夹爪；`q` 12 维 |
| 控制台 | 状态/采集页已就绪；控制页需要双模型 3D 场景与按臂关节卡片 |

## 分阶段

- [x] **D-01** 配置：`configs/robots/arx5/dual.json` + `RobotConfig.arms` + `load_dual_profile()`
  - 两台臂各自的 `can_interface`、限位、kp/kd、park；共享 `control_hz`、夹爪宽度、模型参数。
  - 验证：加载得到 2 个 `RobotConfig`，单臂配置行为不变（现有测试全过）。
- [x] **D-02** 契约：`RobotState`/`ControlCommand` 增加每臂字段（`poses` / `gripper_widths` / `pose_targets` / `gripper_targets`，默认 `None`）
  - 验证：单臂路径字段仍为 `None`；双臂路径可携带两臂姿态与夹爪。
  - Tick V4（每臂 pose/夹爪入 ABI）暂缓到 D-04/D-07，当前 12 关节用 V3 已足够。
- [x] **D-03** 复合后端（仿真优先）：两个 `MujocoBackend` 组合为 12 关节
  - 已实现：`DualBackend` 组合两个 `MujocoBackend`，`DualKinematics` 提供 12 维 FK/IK 边界；`configs/robots/arx5/dual.json` + `arx5-dual` profile/kinematics 注册。
  - 验证通过：`./run.sh control start --backend sim --config configs/robots/arx5/dual.json` 启动后 `status.q` 长度 12、`describe.joint_count=12`，总览/采集页出现「机械臂 1 / 机械臂 2」两组关节。
  - 未完成：observer/telemetry（工作台 MuJoCo 视图）仍按单臂订阅；Tick V4 的每臂 pose/夹爪未做。
- [x] **D-04** 真机复合后端：`DualBackend` + 两个 `RealBackend`（can0/can1）
  - 已验证：两臂同时使能并保持位置（左 `[0.04,0.03,0,‑0.04,0.02,‑0.01]`，右 `[‑0.03,0,0.01,‑0.04,‑0.01,‑0.25]`），`status.q` 长度 12；退出走 SDK 阻尼，tuning 记录完整。
  - **未达标**：控制周期实测约 150 Hz（p95 10 ms），低于 250 Hz 规格；两臂并发反而更慢（93 Hz），已回退串行。遥操前需要优化（SDK 调用流水线或 vendor 的 bimanual 接口）。
- [~] **D-05** 网关与命令：`describe()` 双臂、命令带 `arm`、规划与限位按臂
  - 已做：`describe.robot.arms`（id `arm1`/`arm2`，含 side/can/限位/park）；控制台适配器按 `arm` 计算关节偏移，`move_joint` / `move_joint_target` / `reset_joint` 只改对应臂的关节。
  - 待做：位姿命令（`movel`）按臂规划；`DualKinematics` 目前只对左臂做 IK。
- [x] **D-06** VR：左右手柄各控一臂
  - 已验证：双手各握 Grip，两条机械臂分别跟随对应手柄，`phase=teleop`、`armed/permit=true`。
  - 关键修复：左手输入/位姿中继（丢弃首帧）、协调器先启动（iceoryx 容量契约）、设备先于协调器、按臂反馈（bridge arms=2）、只在协调器下发模式后才镜像、`joint_step_limit` 按臂拆分。
  - 详见 `docs/guides/dual-arm-vr-pipeline.md`。
- [x] **D-07** 采集：12 关节 + 每臂夹爪写入 episode（14 维）
  - Tick 升 V4：新增 `arm_count`、`gripper_widths[2]`、`gripper_targets[2]`，保留 V3 标量夹爪字段（= 第一臂），topic 改 `/telemetry/robot_tick/v4`；`RuntimeReader`/导出接受 version 1–4。
  - 契约：`data_contract.grippers` 声明每臂夹爪（`gripper_left` / `gripper_right`），双臂向量 = 12 关节 + 2 夹爪 = 14 维；单臂契约保持 7 维不变。
  - 写入/校验/导出/对齐按契约走；`RuntimeEpisodeWriter`、`runtime_v2`、`runtime_alignment`、LeRobot features 无需按臂特判。
  - 旧数据（13 维）保持可读：契约存在 episode.json 里，导出按旧契约走。
  - 已录 57 条 13 维数据用 `scripts/datasets/recover_right_gripper.py` 从内核 `tuning.jsonl` 恢复右夹爪**下发值**（逐 tick 对齐，左夹爪对照一致率 >99%）；右夹爪实测值该 SDK 版本不可得。
  - 未做：每臂末端 pose（Tick 里仍是一份）。
- [~] **D-08** 控制台 3D 双模型与按臂关节卡片
  - 已做：状态 12 关节时场景自动创建第二台 X5 模型（`SECOND_ARM_OFFSET`，按实机布局沿台面长边相隔 0.7 m、错开 50 mm），两台模型在同一张工作台/场景里，按 joints 6..11 独立驱动；首次出现双臂时自动把相机拉到能看全桌面又在机械臂一侧的机位；关节卡片单独切片、臂选择器按 `describe.arms.side` 显示 **左臂/右臂**；内核状态增加 `gripper_widths` / `eef_poses`，第二台夹爪跟随。
  - 已做：VR 遥操占用租约时，控制页退到只读接入（`mode=observe`），3D 与关节反馈照常显示，运动命令提示只读。
  - 待做：第二台的目标编辑/预览、摆放位置按实机标定。
- [ ] **D-09**（独立工作）策略与 GR00T 的 12 关节/双夹爪适配

## 风险

- 双臂 MJCF 不存在，仿真验证可能要先做模型；
- 真机双臂首次上电需要逐臂安全检查（限位、力矩、急停覆盖）；
- 策略链路（GR00T 16D、π0.5 7D）与标定采样仍是单臂，双臂数据暂时只能采集、不能直接训练。
