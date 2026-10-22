# 手机遥操控制通道

共享手机服务复用 UDP 5005 的输入连接。二进制 v1.1 姿态、Settings、Ping 和 Reset 帧保持原格式；新增 UTF-8 JSON 控制消息，标识为 `tia.phone.control.v1`。此通道只提供监看、获取现有 robot 控制权、释放控制权，不支持启动 robot、CAN 或 SDK。UDP 本身没有认证，沿用可信可互通网络的使用边界。

请求不超过 1024 字节：

```json
{"protocol":"tia.phone.control.v1","id":1,"session_id":123,"client_time_ns":456000000,"action":"enable","runtime_id":"robot-example"}
```

- `action`：`monitor`、`enable`、`disable`。监看不获取控制权。
- `session_id` 必须匹配最近有效二进制姿态会话。`id` 是该会话内递增的 UInt32。
- 新操作时间戳必须在最近有效姿态的客户端时间 ±500 ms 内；延迟的开始请求被拒绝。
- 最新请求重试返回同一回复，不重复执行。旧序号丢弃，因此乱序的开始请求不会覆盖较新的停止。
- 开始须携带监看回复的 `kernel_runtime_id`，并满足追踪正常、Clutch 松开和内核原有租约条件。机器人实例变化时拒绝旧请求。

回复不超过 2048 字节，回显协议、会话和请求 ID；包含 `ok`、`message`、`kernel_runtime_id`、`enabled`、`active`、`estop`、`tracking`、映射倍率、跟随误差及归一化的 `robot_gripper`。客户端只接收当前待确认请求的回复；`enabled` 代表真实接管状态，UDP socket ready 不等于已接管。客户端每 250 ms 查询状态，500 ms 重试，3 秒未确认显示失败；状态失联时解除 Clutch。

恢复复用二进制 `EStopResetRequest` / `EStopResetAck`。共享服务验证当前新鲜会话、正常追踪、Clutch 松开，释放手机租约并清除手机停止锁定后回 ACK。恢复请求不能早于最近一次停止输入；已确认请求的重复包只补发 ACK，不释放后来重新获取的控制权。恢复不获取控制权、不退出/启动 robot，也不解除物理急停或 SDK 过流保护。失败原因通过状态通道返回；客户端等待 5 秒未收到匹配 ACK 后允许重试。停止锁定落盘 `outputs/runtime/phone-stop.json`，重启接收器不会自动清除。

手机侧的位移与旋转默认均为 1:1 相对映射。`configs/inputs/phone.json` 定义默认倍率和目标相对反馈的最大超前量（5 cm、0.35 rad）；超出部分从映射参考中扣除，避免反向手势仍被之前积累的目标抵消。按下 Clutch / Recenter 时重新建立参考。它限制跟随误差，不限制总行程，也不改变统一运动速度配置；不声称覆盖碰撞检查或所有奇点。

网页的手机卡片位于 Quest 手柄下方。两种输入共用姿态对齐区，优先显示持有控制权的手机；手机映射姿态与执行目标均由相同相对映射策略输出到基座 TCP 坐标系，不直接混入 ARKit 世界坐标。松开、追踪受限、数据过期或 robot 实例变化后隐藏手机目标。手机接管时，网页卡片和终端显示青色“遥操已开启”；松手保持与正在跟随分开显示，错误/追踪警示仍保留。

输入映射和机械臂伺服保持分层。关于伺服侧奇点、平滑和速度约束的职责，参见 [MoveIt Servo 官方说明](https://moveit.picknik.ai/main/doc/examples/realtime_servo/realtime_servo_tutorial.html)；本实现没有引入 MoveIt 依赖或声称实现其完整奇点处理。
