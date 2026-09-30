# 轨迹控制边界

`src/arx5_control/motion/` 是不接触 CAN 和 SDK 的轨迹算法层：

- `profiles.py`：cosine、smoothstep9 和夹爪 via-point 插值。
- `offline.py`：离线 Ruckig 点到点轨迹及时间戳采样。
- `online.py`：实时 Cartesian 拖动使用的在线 Ruckig 状态机。
- `limits.py`：十档速度共用的速度、加速度、jerk 缩放和离散动力学校验。
- `pinocchio_backend.py`：Pinocchio 解析 FK、frame pose 和 Jacobian。
- `jacobian.py`：解析/数值 Jacobian 共用的奇异值与降速指标。
- `cartesian_time.py`：Cartesian 标量路径 `s(t)` 的 jerk-limited 时间参数化。
- `interpolation.py`：固定控制周期的 C2 五次 Hermite 时间缩放与连续重采样。
- `collision.py`：可插拔 sphere/AABB TCP 扫掠预检查。
- `link_collision.py`：Coal（hpp-fcl 后继项目）capsule link、自碰撞和环境碰撞。
- `geometry.py`：Cartesian line/arc 几何路径、quaternion SLERP 和 blend radius。

普通关节相对移动、绝对移动、全关节移动、单关节往返和预览统一使用离线 Ruckig
S 曲线；cosine/via-point 仅保留给夹爪和兼容的纯插值工具。

执行器接口使用 SDK 的 `cubic` 插值。每个 Ruckig waypoint 同时提交位置、速度和
时间戳；SDK 在 waypoint 之间使用 Hermite 曲线，而不是把速度字段丢给线性插值。
Ruckig 速度上限按关节取 `min(应用层上限, RobotConfig.joint_vel_max)`，加速度和
jerk 使用本地显式上限。反馈没有加速度通道时，控制器用相邻时间戳的速度差估计，
并限幅后作为下一段规划的初始状态。

`controller.py` 负责 IK、预览、软件限位、反馈监控和 SDK 提交；`servo.py`
负责固定周期线程和看门狗。旧的 `arx5_control.trajectory` 与
`arx5_control.ruckig_motion` 仍是兼容导入入口，新代码应从 `motion` 子包导入。

Cartesian 直线和 waypoint 路径先按 100% 生成固定空间/姿态采样，并以连续 IK
求出一组 canonical 关节锚点。动力学检查通过后，速度 `s` 只把相对时间变成
`t / s`、命令速度变成 `s * v`；不重新求解 IK。之后按固定控制周期用五次
Hermite 插值增加慢速命令点，并保持锚点间加速度连续。规划器和 SDK 提交前都校验
同一组稠密实际数组，避免插值后才出现加速度或 jerk 超限。

Cartesian 命令边界要求机械臂静止，并以零起始速度规划；普通关节曲线才使用实测
起始速度。允许大范围的网页 Cartesian 操作上限为 `300 s`，超过上限在预览阶段
拒绝。这样低速会更平滑，而不是在少数 waypoint 之间产生停顿。

直线运动的进度由 Ruckig 规划的标量 `s(t)` 驱动，而不是直接对关节做
smoothstep。控制器还会用 FK 重新计算 TCP 实际速度；若 IK 离散误差或奇异位形
导致速度超限，则整体拉伸时间。Pinocchio 提供解析 Jacobian 和逐 link frame pose；
Coal 可选检查整条路径的 link/self collision 和 link/environment collision。

调用方可通过 `ArmController(tcp_collision_obstacles=[...])` 注入
`SphereObstacle` 或 `AabbObstacle`。primitive 始终用于 TCP 检查；仅在显式设置
`enable_link_collision=True` 时同时用于整臂检查。

ARX wheel 中部分 collision STL 未随包发布，因此可选的 X5 碰撞模型使用保守 capsule
包络，不依赖 mesh。该模型默认关闭，以免未标定的包络误拒绝正常姿态；启用前应完成
真机低速标定。相邻 link 因机械连接天然重叠，不参与自碰撞 pair。

## Blend radius

```python
from arx5_control import CartesianPathSegment, Pose

targets = [
    CartesianPathSegment(
        Pose([0.35, -0.05, 0.25], [0.0, 0.0, 0.0, 1.0]),
        blend_radius_m=0.015,
    ),
    Pose([0.35, 0.05, 0.25], [0.0, 0.0, 0.0, 1.0]),
]

preview = arm.plan_move_path(targets, velocity_scale=0.5)
result = arm.execute_motion_plan(preview)
```

内部 waypoint 的 radius 生成相切圆弧；首尾 radius 必须为零。半径超过相邻线段、
相邻 blend 重叠、180 度折返都会在 IK 前拒绝。圆弧改变路径但保持位置切向连续，姿态
在切点间使用 quaternion SLERP。之后统一执行 IK、奇异、碰撞、速度、加速度和 jerk
检查。

这实现了 UR 风格 blend 的离线几何与连续速度提交，但不是 UR 控制器内部的实时
blend：SDK 没有暴露控制周期级 path queue。当前版本不会自动绕开障碍，只会在提交前
拒绝碰撞路径；复杂避障仍交给后续模型、Tesseract 或 MoveIt。
