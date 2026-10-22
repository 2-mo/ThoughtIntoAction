# 手机遥操控制通道

共享手机服务复用 UDP 5005 的输入连接。二进制 v1.1 姿态、Settings、Ping 和 Reset 帧保持原格式；UTF-8 JSON 控制消息标识为 `tia.phone.control.v1`，提供监看、获取/释放现有 robot 控制权以及当前租约下的预设动作，不支持启动 robot、CAN 或 SDK。UDP 本身没有认证，沿用可信可互通网络的使用边界。

请求不超过 1024 字节：

```json
{"protocol":"tia.phone.control.v1","id":1,"session_id":123,"client_time_ns":456000000,"action":"enable","runtime_id":"robot-example"}
```

- `action`：`monitor`、`enable`、`disable`、`home`、`pregrasp`、`align`。监看不获取控制权。
- `session_id` 必须匹配最近有效二进制姿态会话。`id` 是该会话内递增的 UInt32。
- 新操作时间戳必须在最近有效姿态的客户端时间 ±500 ms 内；延迟的开始请求被拒绝。
- 最新请求重试返回同一回复，不重复执行。旧序号丢弃，因此乱序的开始请求不会覆盖较新的停止。
- 开始须携带监看回复的 `kernel_runtime_id`，并满足追踪正常、Clutch 松开和内核原有租约条件。机器人实例变化时拒绝旧请求。
- 预设动作同样绑定 `runtime_id`，必须已持有手机租约、Clutch 松开且输入新鲜。`home` 先通过现有夹爪指令张开到最大开度，反馈误差 ≤2 mm 后才预览/执行回零，回零轨迹保持张开；张开超时取消，不继续移动机械臂。超时按现有夹爪全行程速度加 2 秒计算。`pregrasp` 复用 X5 的 `DEFAULT_GRASP_READY_JOINTS`，保留开度；其他未配置此预设的机器人拒绝。两者均不覆盖统一速度参数。
- 回复的 `preset` 为 `null` 或 `{name, phase, message}`；阶段 `opening`、`moving`、`completed`、`cancelled`。ACK 表示接收，只有内核完成反馈才进入 `completed`，执行失败通过 `message` 报告并释放控制权。停止、会话切换、追踪丢失或超过 100 ms 无新输入会中止预设；不中途切换或排队动作。
- `align` 要求正确的 `runtime_id`、新鲜正常追踪、Clutch 松开且无预设动作；仅监看也可使用，不获取租约。它读取手机和 TCP 的当前姿态并立即应用方向校准，不移动机械臂。回复 `aligned=true` 即已生效，无需另一次确认或按住 Clutch 才校准。手机、终端 C 和网页共用此请求。

回复不超过 2048 字节，回显协议、会话和请求 ID；包含 `ok`、`message`、`kernel_runtime_id`、`enabled`、`active`、`estop`、`tracking`、映射倍率、跟随误差及归一化的 `robot_gripper`。客户端只接收当前待确认请求的回复；`enabled` 代表真实接管状态，UDP socket ready 不等于已接管。客户端每 250 ms 查询状态，500 ms 重试，3 秒未确认显示失败；状态失联时解除 Clutch。

恢复复用二进制 `EStopResetRequest` / `EStopResetAck`。共享服务验证当前新鲜会话、正常追踪、Clutch 松开，释放手机租约并清除手机停止锁定后回 ACK。恢复请求不能早于最近一次停止输入；已确认请求的重复包只补发 ACK，不释放后来重新获取的控制权。恢复不获取控制权、不退出/启动 robot，也不解除物理急停或 SDK 过流保护。失败原因通过状态通道返回；客户端等待 5 秒未收到匹配 ACK 后允许重试。停止锁定落盘 `outputs/runtime/phone-stop.json`，重启接收器不会自动清除。

手机侧的位移与旋转默认均为 1:1 相对映射。`configs/inputs/phone.json` 定义默认倍率和目标相对反馈的最大超前量（5 cm、0.35 rad）；超出部分从映射参考中扣除，避免反向手势仍被之前积累的目标抵消。按下 Clutch / Recenter 时重新建立位姿锚点。它限制跟随误差，不限制总行程，也不改变统一运动速度配置；不声称覆盖碰撞检查或所有奇点。

默认插件 `phone_calibrated` 采用旧 Quest 手动校准的固定坐标变换：操作者把手机前端与夹爪前端摆成同向，点一次对齐。手机前方定义为相机 −Z，上方为手机 +Y；TCP 前方为 +X，上方为 +Z。使用 `K = R_tcp · R_phone_to_tool · R_phone⁻¹`，其中 `R_phone_to_tool` 与旧 `R_CONTROLLER_TO_TOOL` 一致；同一个 K 同时映射位移和世界坐标下的旋转增量。它使用当前 TCP 姿态，不要求预抓取位或夹爪朝下，也不把输入的 Y 假定为重力方向。采集元数据保存 `input_to_base_quaternion_xyzw`。

校准前不下发 Cartesian 跟随；回零、预抓取、松开重握、启停遥操保留 K。普通按住只重建相对位姿锚点，不猜测方向。新的手机会话、追踪不可用、超过 0.5 秒断流或 robot 实例变化后清除校准；短暂 `limited` 只暂停跟随并要求先松手，保留 K。校准准确性依赖操作者实际摆放方向，并非算法自动测得两台设备的物理外参。`phone_tool`、`phone_base` 保留为显式选择的其他映射语义。

网页的手机卡片位于 Quest 手柄下方。`phone_pose` 使用 `frame=phone_reference` 和 `quaternion_xyzw`，直接呈现 iOS 输入姿态：App 将 ARKit 首个有效相机帧作为参考，Y 是初始手机上方，并非保证重力向上。独立 3D 手机图在仅监看、松开 Clutch、没有 robot 时仍可显示；追踪受限或数据过期时隐藏。两种输入共用的姿态对齐区优先显示持有控制权的手机；手机映射与执行目标均由相对映射策略输出到基座 TCP 坐标系，不混入手机参考坐标。松开、追踪受限、数据过期或 robot 实例变化后隐藏目标。手机接管时，网页卡片和终端显示青色“遥操已开启”；预设进度、松手保持与正在跟随分开显示，错误/追踪警示仍保留。

输入映射和机械臂伺服保持分层。关于伺服侧奇点、平滑和速度约束的职责，参见 [MoveIt Servo 官方说明](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html)；本实现没有引入 MoveIt 依赖或声称实现其完整奇点处理。
