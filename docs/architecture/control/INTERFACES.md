# VR 与 ARX 控制接口

## 职责

`native/bin/rollio-device-xrobot` 读取 XRoboToolkit；`rollio-teleop-router` 的 coordinator 和两个 router 负责按键模式、相对位姿、Grip、Trigger 和超时。`native_config.py` 将 `config/vr.json` 与机器人夹爪宽度转为本次运行的原生配置。

`ipc/RobotBridge` 消费末端目标并发布机器人反馈；不处理 IK 或调用 ARX SDK。`worker.py` 管理 250 Hz 循环、反馈检查、日志和退出。`model.py` 处理 FK/IK 与关节目标。`real.py` 将目标发给 SDK；`sim.py` 提供相同上层契约的 MuJoCo 后端。

导入控制模块不会导入 VR 或机械臂 SDK。构造 `RealBackend` 会初始化、使能电机，不能把构造器当作只读探测。

## 单位与契约

| 数据 | 定义 |
| --- | --- |
| `RobotState.q` / `dq` | 六轴，rad / rad/s |
| `pose` / `pose_target` | base_link 中的指尖 TCP `[x,y,z,qx,qy,qz,qw]`，m 与单位四元数 |
| `gripper_width` / `gripper_target` | 两指间距，m |
| `now` / `received_at` | 同一时间基准的 `time.monotonic()` 秒 |
| `ControlCommand.mode` | Disabled、FreeDrive、CommandFollowing |

`ControlCommand` 表示模式和末端目标，不是六关节 SDK 数据包。`RobotBridge.poll_commands(now)` 仅在有新输入时返回目标；`publish_state(state, now)` 不会把过期反馈当成新测量。

后端接口为 `read_state(now)`、`step(command, now)`、`telemetry()`、`runtime_parameters()` 和 `close()`；真机另有 `request_stop()`。反馈超过 100 ms 或包含非有限值时拒绝继续普通控制。

## 接入自己的 IK

最小替换点：`X5Model.solve_ik(target, q_seed, q_anchor=None)`，返回 `(q, converged, residual)`。输入使用上述 base/TCP 约定，输出必须是六维有限弧度向量。保留同一模型的 `fk(q)`，否则 VR 对齐会错误。

`JointTargetController.update()` 管理模式、Home、IK seed、发送目标与 `dq_target`。ARX 后端支持 `controller_factory(config, kinematics)` 注入兼容控制器。改 IK 时保留反馈检查、Home 和退出处理。

实际发送位于 `RealBackend.step()`：

```python
# 接口示意，省略初始化与反馈验证，不能直接当作运行脚本。
cmd = sdk.JointState(6)
cmd.pos()[:] = q_rad
cmd.vel()[:] = dq_rad_s
cmd.torque()[:] = extra_feedforward_nm
hardware.set_joint_cmd(cmd)
hardware.set_gain(gain)
hardware.send_recv_once()
```

自动重力开启时，`cmd.torque()` 只放额外前馈。没有额外的协议系数或 Kd 换算。

## 替换 VR 来源

`XRPublisher(prefix).publish(XRFrame(...))` 可发布原生兼容输入；pose 须已转换到机械臂基座，不能重复做基变换。序号递增，连接和追踪无效时必须报告无效。`demo.py` 是仿真示例，应用禁止 demo 驱动真机。

模式动作窗口 150 ms、输入超时 100 ms、模式确认 1000 ms、Disabled 等待 3000 ms 保留原实现。位姿保持/失效阈值为 30/100 ms；普通 Linux 与 Python 不提供硬实时保证。

原生程序仍监听 Rollio 全局关闭总线。其他 Rollio 程序发出的关闭事件可能使这些子进程退出；主程序会检测退出并清理自己的后端。运行目录独立不代表完全隔离所有全局事件。
