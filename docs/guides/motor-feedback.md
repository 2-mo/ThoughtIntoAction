# 电机诊断采集

只读 SocketCAN 接收器 `src/legacy/robot_control/motor_feedback.py` 在真机适配器初始化后启动，和 SDK 各自接收 CAN 广播；不发送任何 CAN 帧，不查询寄存器、不修改电机状态。通过 `recvmsg` 的 `MSG_DONTROUTE` 排除本机下发的指令，避免把同 ID 的控制帧当反馈。仿真不创建接收器。

## 已核对的硬件和字段

本项目固定 SDK `third_party/arx5-sdk` 8c797d5 的 X5 配置：

| 部位 | 电机 | CAN ID | 电流 A | 温度 °C | 状态/故障 |
| --- | --- | --- | --- | --- | --- |
| J1/J2/J3 | EC_A4310 | 1 / 2 / 4 | 反馈 12 bit，量程 ±30 A | `(byte6 - 50) / 2` | byte0 低 5 bit，0 为无故障 |
| J4/J5/J6 | DM_J4310 | 5 / 6 / 7 | 缺失：MIT 负载反馈不能冒充电流 | byte6 驱动/MOS 温度，byte7 转子温度 | byte0 高 4 bit；0/1 是正常状态，8–15 保留为故障码，其他保留未知 |
| 夹爪 | DM_J4310 | 8 | 同上，缺失 | 同上 | 同上 |

EC 只解码 ACK1（byte0 高 3 bit 为 1）；DM 响应 CAN ID 为 0，byte0 低 4 bit 为电机 ID。标准 8 字节帧才进入解析。负载原始码来自 `((byte4 & 15) << 8) | byte5`。

确认依据：

- 固定 SDK `include/app/config.h` 的电机映射、`include/hardware/arx_can.h` 的字段，以及 `controller_base.cpp::update_joint_state_` 的电流/力矩转换路径。
- 安装的 `arx5-interface==0.1.3` 硬件库导出 `RV_can_data_repack` / `DM_can_data_repack`；核对解码指令确认 EC ±30 及温度编码。SDK 的 DM 解码器没有向上层传递温度和状态，而且将负载反馈命名为 `current_actual_float`。
- [DM 协议参考实现](https://github.com/cmjang/DM_Control_Python/blob/main/DM_CAN.py) 的 `getTorque` / MIT 解码将负载定义为力矩；因此本采集器不把 SDK 字段名当作安培单位依据。DM 原始负载码与协议值单独保留，不修改现有 SDK 力矩行为。
- 本机 can0 的真实 RX 帧已被只读观察：七个电机持续更新，EC 返回约 32–42°C，DM 两路温度约 25–36°C；这些是核对时的样本，不是固定值。

周期反馈没有母线电压，`bus_voltage_v` 为 null。EC 温度没有细分传感器位置，不能标成绕组温度。未支持的型号、无反馈、接收错误及超过 250 ms 的帧不产生有效值，不能用 0 填充；有故障时保留原始数值，但 `valid=false`。

## 记录主题

- `/robot/state`：`joint_current_a`（J1–J6）、`gripper_current_a`、`motor_load_raw`（六关节＋夹爪）、`motor_feedback_monotonic_ns`、`feedback_valid`。DM 电流是 null，原始负载码不是电流。
- `/robot/diagnostics`：1 Hz，包含电机映射、各电机温度、电压缺失值、原始反馈十六进制、来源、接收时间、数据年龄和当时状态；每条开始保存基线。
- `/robot/events`：状态、故障码或有效性变化时记录；带原始单调时间、电机 ID、接收流 ID 和递增事件序号。接收器保留有界事件缓冲；检测到事件丢失时将录制标为失败，而非静默忽略。
- `episode.json`：`diagnostic_counts` 保存诊断样本和事件计数。SDK/控制程序异常退出仍由 episode 的 `stop_reason` / 生命周期标记描述。

温度字段在服务快照中可查询，写入 MCAP 时从高频状态拆到低频诊断主题。训练导出和数据校验会拒绝含明确无效/故障反馈的状态样本；缺失的电流或电压本身不意味着关节位置无效。

不需要重编译 SDK。新机械臂服务、采集会话加载代码后接入；不会热替换运行中的电机控制进程。

## 网页监看与回放

网页“电机反馈”表显示 J1–J6 和夹爪的真实电流、温度、DM 转子温度、母线电压、故障码和反馈有效性。正常反馈使用淡底色，故障使用暖色底色，未提供的值显示 `—`；整条遥测失联后不保留实时数值。母线电压和 DM 电流目前未提供。

实时链路为机械臂服务 → 本地遥测 → 网页；更新后须重新启动机械臂服务才能发布新增字段，单独刷新网页不会更新已运行的服务代码。回放按所选状态样本时间读取 MCAP 诊断基线和后续故障事件，电流与有效性取高频状态，温度保持最近一次低频采样；不会提前显示未来采样。缺少诊断主题的已有数据保留占位。
