# 独立仿真与 D435i 眼在手上标定

使用项目内 X5-2025 hybrid URDF。`link6` 与当前遥操 EEF 一致；`eef_link` 是另一个固定坐标系，不能混用结果。相机坐标系是 **RGB 光学系**，不是相机外壳或深度光学系。

三维监看在 `src/embodied/monitor/`，虚拟控制在 `src/legacy/robot_control/cli.py`，标定在 `src/embodied/calibration/wrist/`。共同使用当前 X5-2025 模型，控制通过本机接口连接机械臂服务，服务向监看发布状态。

## 监看与虚拟控制

```bash
# 终端 1：持续提供虚拟反馈。
./run.sh arm-service --backend simulated

# 终端 2：只读监看。
./scripts/viewer/启动仿真.sh

# 终端 3：选择一种虚拟控制。
./run.sh control-arm
./run.sh control-arm --input quest
./run.sh control-arm --replay outputs/calibration/plan.json
# 只做 FK / 模型校验后退出：
./run.sh control-arm --headless
```

监看无需知道由终端、Quest 还是真机控制；控制程序重启后自动显示新来源，过期数据会标记。显示采用 Three.js 的材质、灯光、工作台及轨迹。工作台和夹爪仅为示意几何，没有接触动力学或完整碰撞检测；没有迁入旧 Desktop 模型或未标定相机位姿。完整说明见 [独立机械臂监看](monitoring.md)。

终端控制命令：`q J1 ... J6`（rad）、`fk`、`ik x y z rx ry rz`（m、旋转向量 rad，非 RPY）、`exit`。位姿控制保留 Grip/Trigger、A 与长按 B。虚拟服务使用初始姿态 `[0, 1.5, 1.45, -1.52, 0, 0]`、50 Hz、0.2 rad/s 和 0.05 rad 软限位，初始 `--q` 在服务端设置，控制速度在 `configs/profiles/virtual-arm.toml` 设置。

标定计划回放先从服务当前姿态移动到记录的中心姿态，使用计划内的速度、加速度、采样率和限位余量。会话回放在采样姿态间插值，**不代表真实采样过程中的运动轨迹**。`--replay` 只用于终端控制，不能与 Quest 输入同时使用。`--headless` 只做离线校验后退出；控制不发布状态，状态统一由服务提供。

## 标记和手动采样

目前支持固定的单个 ArUco `DICT_4X4_50` 标记，默认 ID=0。以下以黑色外边长 100 mm 为例：

```bash
./run.sh handeye marker --marker-size-m 0.100
./run.sh handeye manual --marker-size-m 0.100
```

生成的 `outputs/calibration/marker.svg` 包含白边。按 100% 比例打印后，用尺测量**黑色方块外边长，不包含白边**；采样参数必须填实测值。固定在平整板面，采样期间板与机器人基座均不能移动。未知真实打印尺寸时不要使用示例值进行标定。

先启动 `./启动机械臂服务.sh`。手动采样独占 RGB，并向服务申请阻尼状态的控制权，由人引导到不同姿态；不会创建第二个 SDK 控制器。开始前退出遥操及其他相机程序，机械臂服务和监看保持运行。自动采样同样使用该服务，启用前由服务根据当前反馈设置位置保持，不自动回零或张爪。

画面按键：`s` 采一组拟合样本，`v` 采一组独立验证样本，`q`/Esc 结束。每姿态尝试 20 帧，至少 12 帧有效；相机/机器人明显移动、标记模糊/歧义、重复姿态都会拒绝。至少 10 个拟合姿态、4 个独立验证姿态；建议 20+6。验证集也要覆盖多个方向，不能在同一姿态重复按键凑数。

输出默认在 `outputs/calibration/时间/session.json`，包括原始 PNG、每帧角点/PnP、设备时间戳、主机接收时间和相机读取前后的机器人快照。主机时间是接收时间，不伪装成曝光时间；目前采用静止采样，没有硬件时钟同步。这里只采集标定数据，不是遥操训练 episode。

## 自动采样

先在合适的中心姿态手动保存一组样本，再生成计划：

```bash
./run.sh handeye plan --from-session outputs/calibration/你的会话/session.json
./run.sh control-arm --replay outputs/calibration/plan.json
./run.sh handeye auto --plan outputs/calibration/plan.json
```

这三条命令不打开 CAN 或相机；中间的回放命令需要预先启动虚拟服务。完成预览后，真机采样改用真机服务，不能让虚拟控制连接真机。计划有 20 个拟合姿态、6 个预先分配的验证姿态。使用六关节偏移、0.05 rad 软限位余量、五次插值，默认最高 0.15 rad/s、0.3 rad/s²、50 Hz。中心不是固定抓取准备位：该预设的 J4 靠近下限，不一定适合标定。

执行前需要现场确定 `link6` 的允许范围，单位 m，坐标相对 `base_link`。重新生成计划时添加 `--eef-bounds XMIN XMAX YMIN YMAX ZMIN ZMAX`，把六个占位符替换成实测边界，重新预览。边界仅限制 EEF，不能证明臂身、桌面、腕部相机、夹爪和线缆无碰撞。预览时必须检查实际安装的这些物体；未提供边界时仅允许离线检查。

确认现场轨迹后执行：

```bash
./run.sh handeye auto --plan outputs/calibration/plan.json --marker-size-m 0.100 --actuate
```

机械臂必须先由人移到计划中心，误差不超过 0.03 rad；程序不会自动回中心。先检查标记可见，再启用位置保持；保留当前夹爪开度，不照搬旧项目的自动张爪。每点到位、静止后才采集；跟踪误差、超时、标记丢失/歧义或 Ctrl+C/SIGTERM 都中止并请求阻尼，不执行回零或回参考位。正常完成也进入阻尼。

这是尚未真机验收的自动采样实现。当前边界检查不是全连杆碰撞检测；SDK 时间戳反映控制循环推进，不等于逐电机反馈新鲜度。突然断电、进程 SIGKILL 或 CAN 物理故障也无法由 Python 的退出清理保证阻尼成功。

## 离线求解与验收

```bash
./run.sh handeye solve outputs/calibration/你的会话/session.json
```

求解约定：`T_a_b` 把 b 系坐标变到 a 系；`T_base_eef @ T_eef_camera @ T_camera_target = T_base_target`。程序检查会话模型哈希、关节顺序、EEF 名称和保存的 FK，输出 `result.json`。

当前 OpenCV 5 缺少 `calibrateRobotWorldHandEye`。本项目用 NumPy 的相对运动 SVD 求解；若环境提供 OpenCV Shah/Li，也参与拟合集比较。合成已知真值、噪声、离群点和退化运动测试验证变换方向与求解。

拟合集最多剔除 20% 样本，剔除后重新检查运动多样性；验证集不参与拟合、方法选择或剔除。结果包含每点误差、排除 ID、安装变换留一法稳定性与独立验证误差。初始门槛为 8 mm / 2°，不是下游任务精度承诺。

- `candidate`：有拟合解，但独立验证缺失/失败或稳定性未过关。
- `validated`：通过当前数值门槛，仍需检查安装、打印尺寸及实际任务误差。

程序不改 `hand_eye_calibrated=false`，不会把数值解自动投入推理控制。无有效解时不覆盖已有结果。标定外参、相机安装和板尺寸的现场验收另行进行。

## 依赖与验证记录

当前由 uv 管理项目 `.venv`，保留 NumPy 2.3.5、Pinocchio 3.7.0、Meshcat 0.3.2、OpenCV 5.0.0、pyrealsense2 2.58.4；没有引入桌面工程依赖。迁移中的原生依赖约束和环境边界见 [环境说明](../../environments/README.md)。

2026-09-08 已验证 D435i `233522074971` RGB 640×480@30，SDK 返回 `inverse_brown_conrady`，系数为零；内参始终从实际流读取。未执行真机自动运动或宣称已完成手眼标定。
