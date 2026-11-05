# 双臂 VR 遥操链路

单内核控制两台 X5：左手柄驱动左臂（can1），右手柄驱动右臂（can0）。本文记录完整链路、启动顺序、协议语义和已经踩过的故障，遇到问题先按"故障对照表"定位，不要再从头试。

## 1. 拓扑

```
Quest 头显
  │ XRoboToolkit 客户端（服务器 127.0.0.1，Connect）
XRoboToolkit PC 服务 :63901
  │
rollio-device-xrobot（两个通道）
  ├─ xr/right_controller/states/{pose,input}
  └─ xr/left_controller/states/{pose,input}
        │
        └─ input_relay（仅左手）→ xr/left_controller/states/{pose,input}_filtered
              │
rollio-teleop-coordinator（group/permit + 模式仲裁，Mode=右手 / Episode=左手）
  ├─ arm 路由        (leader 左 pose_filtered → robot/arm/commands/end_pose)
  ├─ gripper 路由    (左 trigger → robot/gripper/commands/parallel_mit)
  ├─ arm2 路由       (leader 右 pose → robot/arm2/commands/end_pose)
  └─ gripper2 路由   (右 trigger → robot/gripper2/commands/parallel_mit)

内核 embodied.control（--managed，real 后端）
  ├─ NativeIngress(dual)：右输入做模式/激活判定，两臂目标合并成 pose_targets
  ├─ RobotBridge(arms=2)：发布 robot/{arm,arm2,gripper,gripper2}/info/mode 与 states/*
  ├─ Gateway/CommandEngine：仲裁、限位、命令
  └─ DualBackend：两个 RealBackend(can0/can1)，命令按臂拆分

component-source --plugin quest：持有控制租约（owner=quest）
```

## 2. 启动顺序（每一步都有原因）

| 步骤 | 原因 |
|---|---|
| 1. `./run.sh can-up`（can0、can1） | 两臂各一条 CAN |
| 2. 头显连接 127.0.0.1，确认控制器（不是手部）追踪 | 手部追踪模式下按钮/Grip 全为 0 |
| 3. `./run.sh kernel profiles start --profile real-dual-quest` | profile 会带 `component-source`，缺它就没有 owner |
| 4. XR 设备先于协调器启动（app.py 的 dual 分支自动处理，等待 1.5 s） | 协调器看到无效 Episode 首帧会立刻锁组 |
| 5. 协调器先于路由/中继启动 | iceoryx 容量契约（4/32/32）只能由协调器创建，否则 `DoesNotSupportRequestedMinBufferSize` |
| 6. 中继在协调器之后启动 | 继承同一服务契约；它只过滤首帧，不改变语义 |

## 3. 协议语义

- **组模式是全局的**：`Disabled` / `FreeDrive` / `CommandFollowing`，四个成员一起切换。
- **右手柄是 Mode**：长按 A 150 ms 切 FreeDrive ↔ CommandFollowing；A+B 回 Home。
- **左手柄是 Episode**：只提供位姿与扳机，没有独立开关。
- **Grip 是跟随开关**：`armed` 由右手输入的"非空闲"样本置位；`permit` 只有组处于 CommandFollowing 才为真。
- **内核 `armed/permit/phase`**：三项同时为真表示机械臂正在跟随 VR。

## 4. 关键实现点（改代码时别破坏）

| 位置 | 作用 |
|---|---|
| `configs/robots/arx5/dual.json` | 双臂 profile：左 can1 / 右 can0（按实机对应关系配置） |
| `configs/workbench/real-dual-quest.json` | 工作台 profile，引用上面的 config |
| `quest/native_config.py` | 双通道、四路由、Episode=左手 filtered、`mode_ack_timeout_ms=8000` |
| `quest/composition.py` | 生成 7 个原生进程（含 relay） |
| `quest/input_relay.py` | 丢弃首个有效帧之前的 input/pose 样本 |
| `control/app.py` | dual 分支：设备先起，等 1.5 s 再起协调器/路由 |
| `control/ipc/__init__.py` | `RobotBridge(arms=N)`：多臂通道与按臂反馈 |
| `control/worker.py` | 按 dual 创建 bridge(arms=2) 和 ingress(dual=True) |
| `quest/sources.py` | 双输入合并；只在协调器真正下发过模式后才镜像 Disabled/FreeDrive |
| `arx5/dual.py` | `DualBackend`：12 维状态、按臂拆分命令（含 `joint_step_limit`） |
| `arx5/profile.py` | `DualArmProfile`：拼接向量、`describe_arms()` |

## 5. 故障对照表

| 现象 | 根因 | 处理 |
|---|---|---|
| 协调器 `ControllerInputInvalid { role: Episode, tracking_valid: false }` | 设备启动首帧无效，Episode 角色不容忍 | 确认 relay 在跑；不要绕过它 |
| `InitialReadinessTimeout` | 双通道启动慢于 1 s 默认窗口 | `mode_ack_timeout_ms=8000`（已配） |
| `InitialReadinessTimeout` 且带 `input_sources={}`，之后按 A 无反应、内核 `armed` 一直 false | 内核启动时**头显还没连上**，首帧为空 → 协调器进入 Disabled 并锁住，之后连上头显也不会自动恢复 | 先让 XRoboToolkit 显示已连接，再启动内核；已锁住就重启内核（`coordinator.log` 里有 `entering Disabled` 就是它） |
| `PublishSubscribeOpenError(...MinBufferSize)` | 中继/路由抢在协调器前创建服务 | 保持"协调器先启动"，中继在其后 |
| `RouteFault { route_id }` | 某路由 leader/follower 500 ms 无更新（vendor 硬编码） | 确认双手被追踪、中继在跑；再看对应 `arm*.log` |
| 内核日志 `Invalid command step limit` 后退出 | 12 维 `joint_step_limit` 未按臂拆分 | `DualBackend.arm_commands` 必须切分（已修） |
| `armed` 永远 false | 没有 `component-source`（无 owner，ingress 不被选中） | 用 profile 启动，不要只跑 `control start` |
| 按钮/Grip 全是 0 | 头显处于手部追踪 | XRoboToolkit 里关闭两个 hand |
| `OperatorAPlusB` | 长按 A 被识别为 A+B（回 Home） | 切模式时按 A 不要同时碰 B |
| 预览/状态偶发 0 样本 | 订阅端与发布端 iceoryx 缓冲区配置不同 | 用内核自带工具读，不要临时起订阅脚本 |
| 控制频率约 150 Hz | 两臂串行 SDK 往返（并发更慢，93 Hz） | 已知限制，遥操可用 |

## 6. 日常检查

```bash
# 启动后：协调器不应有 Disabled 记录
grep -c "entering Disabled" outputs/runtime/control/<RUN>/coordinator.log

# 内核状态：phase=teleop、armed/permit=true 表示正在跟随
PYTHONPATH=src outputs/envs/control/bin/python -c "
from pathlib import Path
from embodied.contracts.runtime.control import ControlClient
print(ControlClient(Path('outputs/runtime/control/<RUN>')).status()['ingresses'])"
```

## 7. 控制台在遥操时做什么

控制租约同一时刻只有一个持有者：VR 遥操期间由 `quest` 持有，控制页再用 `ControlSession` 接入会被拒（`Control is already owned`）。所以控制页采用**只读接入**：

- `connect` 拿不到租约时退回 `ControlClient`（`mode=observe`），状态、`describe`、12 关节反馈都能读，3D 场景照常跟随两台臂；
- 所有运动命令返回"只读"提示，模式按钮与急停禁用；
- 状态栏显示「只读 · VR 遥操」，臂选择器用 `describe.arms` 的 side 标成 **左臂 / 右臂**，与左右手柄一一对应；
- 3D 里两台 X5 按实机布局摆在工作台上（`SECOND_ARM_OFFSET`：沿台面长边相隔 0.7 m，略微错开 50 mm），首次收到 12 关节时自动切到「能看全桌面、两台臂都在台面上」的机位（`frameDualScene`），之后可自由拖动；
- 停止遥操后重新点「连接」即可拿到租约转为可控制。

## 8. 尚未完成

- 采集已按 14 维记录（12 关节 + 每臂夹爪，Tick V4）；末端 pose 仍是一份。
- 第二台模型的位姿目标编辑、按臂末端预览未做（3D 双臂跟随与按臂关节卡片已完成，D-08）。
- 按臂夹爪宽度已进内核状态（`gripper_widths` / `eef_poses`），旧内核需要重启后才带这两个字段。
- 控制频率 150 Hz，策略推理前需要优化 SDK 调用流水线。
