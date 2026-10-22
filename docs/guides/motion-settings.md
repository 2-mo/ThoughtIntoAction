# 统一运动参数

日常统一内核只从 [`configs/control/motion.json`](../../configs/control/motion.json) 加载应用运动参数。MoveJ、MoveL、move_path、总控命令、robot 的 P/H、Python 示例及网页控制台共用这份配置；入口不再填写自己的速度默认值。原生 VR 输入和 RPC 流式目标的逐周期关节速度边界也由同一个 MotionLimits 解析。

| 位置 | 含义 |
| --- | --- |
| `defaults.speed` | rad/s。`0` 只采用后端逐轴能力；正数与逐轴能力取较小值 |
| `defaults.acceleration` | rad/s²，轨迹规划加速度 |
| `defaults.jerk` | rad/s³，轨迹规划加加速度 |
| `cartesian.tcp_speed / tcp_acceleration / tcp_jerk` | m/s、m/s²、m/s³。`null` 不额外添加末端约束 |
| `cartesian.angular_speed / angular_acceleration / angular_jerk` | rad/s、rad/s²、rad/s³。`null` 不额外添加旋转约束 |
| `settling` | 位置容差、稳定速度、稳定窗口、到位等待超时及反馈间隔 |
| `native_router` | VR 原生路由器的 Ruckig 开关与平滑参数；`smoothing_enabled=false` 时这组参数不参与限速 |

默认 `speed=0`，加速度 `30`、jerk `300`，TCP 六个约束均为 `null`。网页速度默认 100%，表示完整使用内核配置；用户主动选择的档位通过 `speed_scale` 缩放关节速度。加速度和 jerk 仍影响达到峰值的时间，去掉额外速度上限不等于短行程能达到后端最大速度。

ARX X5 的零位到预抓取姿态使用上述默认值，离线规划约 `0.57 s`，部分关节达到 SDK 的逐轴速度上限；旧 `2/10` 参数约需 `1.95 s`。这说明旧规划参数拖慢了动作，不代表更快的轨迹已经通过真机验收。`30/300` 是应用规划配置，不是厂家公布的硬件加速度/jerk 额定值。

VR 原生路由器当前关闭 Ruckig 平滑，保留的有限正数是原生配置格式的要求，不是当前动作额外生效的限速。其笛卡尔平移单位为 m，旋转为 rad，夹爪为 m；对应速度、加速度和 jerk 分别除以 s、s²、s³。包括直接控制模式在内，VR 送到控制 worker 的关节步长统一由 `defaults.speed` 与后端能力换算。

显式调用参数（例如 `robot.movej(q, speed=1, acceleration=3)`）覆盖对应默认值，并进入预览的 `limits`；没有显式参数时使用运行内核的快照。`describe().motion_limits` 返回默认值、逐轴能力、来源、单位和到位条件；预览的 `metrics.limiting_factors` 说明哪些约束决定了轨迹时间。robot 面板显示当前运行的生效值，不直接把磁盘上刚修改的值当作已生效。

设备适配器报告的能力是底层边界：真机来自当前 SDK 配置，仿真/离线来自机器人模型配置。它们不等于实测硬件额定值；当前 ARX SDK 没有提供加速度/jerk 的额定字段，所以这两项明确属于应用规划参数。位置限位、工作空间、增益和电流保护保持各自原有职责，不混入速度配置。

ARX 的 `configs/robots/arx5/robot.json` 使用 SDK `joint_controller_6` 的逐轴增益：`kp=[80,70,70,30,30,20]`、`kd=[2,2,2,1,1,0.7]`。此前所有轴使用 `kp=20`，弱化了 J1–J5 的位置跟踪。增益恢复不等于静态误差已经消除；仍使用原有约 `0.20°` 的位置容差，以实测反馈判定到位。SDK 重力补偿保持一次，未添加额外力矩补偿。

配置在内核启动时读取一次，修改后需要重新启动内核。真机退出会进入阻尼，应先支撑机械臂；程序不会在运动或保持期间热替换参数。到位超时会报告最大误差关节及容差，动作总耗时包括轨迹时间和到位等待，不能把总耗时直接当作运动时间。

`web legacy` 和 `src/legacy/` 的历史控制服务不属于统一内核，不应与日常控制混用；其旧运动配置未迁移到此入口。
