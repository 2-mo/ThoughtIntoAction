# VR 遥操采集运行手册

用 Quest 遥操驱动 ARX X5、在控制台录制 episode 的完整流程。本文按代码中的实际启动顺序和状态判定写成，每一步都有确定性的检查方法；不满足前置条件时不要启动，启动后按状态表处理，不靠反复试。

## 1. 组件与数据流

```
Quest 头显（XRoboToolkit 客户端）
    │ USB / ADB 反向转发 127.0.0.1:63901
XRoboToolkit PC Service  :63901
    │
rollio-device-xrobot（PICO SDK 桥，进程名 xrobot）
    └─ 发布 <prefix>/xr/right_controller/states/{pose,input}
         │
         ├─ rollio-teleop-router arm      → robot/arm/commands/end_pose
         ├─ rollio-teleop-router gripper  → robot/gripper/commands/parallel_mit
         └─ rollio-teleop-coordinator     → group/permit（模式仲裁）
         │
内核 embodied.control（--managed，real 后端，250 Hz）
    ├─ 机器人后端 arx.real（CAN can0）
    ├─ native ingress：根据 input/permit/route health 判定 armed / permit
    └─ commands.sock ← 网页控制台、动作 CLI、采集进程

相机：共享相机服务（camera.sock）→ 采集进程录制、网页 MJPEG 预览
```

## 2. 进程与判定文件

内核每次启动在 `outputs/runtime/control/<RUN>/` 下写以下文件，用于确定性判断，不需要看终端输出：

| 文件 | 含义 |
| --- | --- |
| `runtime.json` | 本次运行的 backend、input、plugins、`supervisor_pid` |
| `backend.ready` | 机器人后端已就绪（电机使能、反馈正常） |
| `runtime.ready` | 全部组件启动完成 |
| `native-processes.json` | coordinator / arm / gripper / xrobot 四个原生进程的 PID |
| `coordinator.log` | **模式状态机日志**，每次切换写一行 `entering X from=Y reason=Z` |
| `backend.log` | `mode arm=... gripper=...` 模式切换记录 |
| `arm.log` / `gripper.log` / `input.log` | 路由与输入进程日志 |

## 3. 状态字段

从 `GET /api/v1/system/status` 或 `ControlClient(<RUN>).status()` 读取：

| 字段 | 含义 |
| --- | --- |
| `phase` | `hold`（保持）/ `teleop`（跟随中）/ `free-drive` |
| `owner` | 当前控制入口：`quest`（VR 接管）/ `console`（网页）/ 空 |
| `ingresses.native.ready` | 追踪有效、连接正常、Grip 数值可读 |
| `ingresses.native.armed` | 操作者已脱离"全松开"状态（按过 A/B 或握过 Grip） |
| `ingresses.native.permit` | 协调器下发了跟随许可（处于 CommandFollowing） |
| `teleop.source` | profile 的输入源：`quest` / `cli` |
| `teleop.quest_usb` / `teleop.xr_service` | 头显 USB 在线 / XR 服务端口监听 |

`armed` 的判定逻辑（`src/embodied/devices/inputs/quest/sources.py`）：

```
idle = 没有按 A/B 且 grip < 0.2
出现过一次非 idle 的输入 → armed = True
```

## 4. 启动前置条件（全部满足再启动）

| 条件 | 检查 | 期望 |
| --- | --- | --- |
| CAN 接口 | `ip -br link show can0` | `can0 UP` |
| 头显客户端 | `./run.sh quest-transport status` | `当前连接：127.0.0.1` |
| 控制器追踪 | 戴上头显、拿起右手柄 | — |
| 相机服务（采集需要） | `curl -s -o /dev/null -w "%{http_code}" http://<host>:8000/api/v1/collection/camera-preview/wrist_rgb.jpg` | `200` |
| 无其他内核 | `ls outputs/runtime/control/*/runtime.json` 后确认进程存活 | 只有一个 |

**最关键的一条：头显必须在启动内核之前就已经连上、且右手柄正在被追踪。**

原因在代码里（`src/embodied/control/app.py`）：

```
1. 启动 backend，等待 backend.ready（≤15 s）
2. 启动 coordinator（等待其发布 permit，≤5 s）
3. 启动 arm / gripper 路由和 xrobot 设备
```

协调器比 xrobot 设备先起来，它的几道等待都很短：

| 阶段 | 超时 | 超时后写入 coordinator.log 的 reason |
| --- | --- | --- |
| 初始就绪 | 约 2.4 s | `InitialReadinessTimeout` |
| 模式确认 | `mode_ack_timeout_ms = 1000` | `ModeAcknowledgementTimeout` |
| 输入/路由健康 | `input_timeout_ms = 100` | `InputOrRouteHealthTimeout` |

所以头显不在线时启动内核，协调器会在设备开始发布输入之前就超时进入 `Disabled`，之后只能重启内核恢复。

## 5. 网页向导（推荐）

总览页右上角的「遥操准备」把下面 5 步做成了实时清单，每一步显示状态并在可操作时给出按钮：

| 步骤 | 判定 | 按钮 |
| --- | --- | --- |
| 1 CAN 接口 | `can0` 是否 UP | 建立接口（`./run.sh can-up`） |
| 2 Quest 头显 | USB 授权 + ADB 转发 + 客户端连接 | 准备 ADB 转发（`./run.sh quest-transport prepare`） |
| 3 控制内核 | 内核连通且输入源为 `quest` | 启动 real-quest（跳转到「系统电源」确认流程） |
| 4 遥操通道 | `ready / armed / permit` 三项标志 | 无（握住 Grip 即可） |
| 5 相机服务 | 共享相机服务 socket 在线 | 启动 / 停止相机服务 |

顶部显示完成度（例如 `4 / 5`）。第 4 步长期停在「等待握持 Grip」时，说明前三步已就绪，握一次 Grip 就会变成「遥操中」。

命令行方式（与向导等价）：

## 6. 启动顺序（严格按此执行）

```bash
# 1. CAN（每次重启机器后一次）
./run.sh can-up

# 2. 头显：XRoboToolkit 客户端 → 服务器 127.0.0.1 → Connect，右手柄被追踪

# 3. 相机服务（采集和预览需要；控制台视频 8767 与它互斥，不要同时开）
./run.sh camera-service

# 4. 内核：网页「系统电源 → 真机 · Quest」或
curl -X POST -H 'content-type: application/json' \
     -d '{"profile":"real-quest","confirm":true}' \
     http://<host>:8000/api/v1/operations/kernel/start
```

## 7. 启动后的确定性验证

```bash
RUN=$(ls -t outputs/runtime/control | head -1)

# 协调器是否干净启动：没有 entering Disabled 就是好的
grep -c "entering Disabled" outputs/runtime/control/$RUN/coordinator.log

# 四个原生进程都在
cat outputs/runtime/control/$RUN/native-processes.json

# 内核侧状态
curl -s http://<host>:8000/api/v1/system/status
```

期望：`entering Disabled` 计数为 **0**，`ingress.native.ready = true`。

然后**握住右手柄 Grip**，再检查一次：`armed=true`、`permit=true`、`phase=teleop`。

## 8. 状态 → 动作对照表

| coordinator.log 最后一条 | 含义 | 唯一正确动作 |
| --- | --- | --- |
| （无 `entering Disabled`） | 正常待命 | 握 Grip 开始遥操 |
| `from=AwaitingInitialReadiness reason=InitialReadinessTimeout` | 启动时头显未连或未追踪 | 确认头显在线后**重启内核** |
| `from=WaitingForModeAcks reason=ModeAcknowledgementTimeout` | 路由 1 s 内未确认（输入未到位） | **重启内核** |
| `from=Stable(FreeDrive) reason=InputOrRouteHealthTimeout` | 输入中断超过 100 ms | **重启内核** |
| `from=Stable(CommandFollowing) reason=OperatorAPlusB` | 收到 A+B（回 Home / 关闭跟随） | **重启内核**；遥操过程中不要按 A |

`ingress` 的三个标志同时为 true，才表示"内核已经接受 VR 指令"。

## 9. 采集流程

1. 采集页填任务描述，选采集方式（固定时长 / 手动停止）；
2. 点「开始采集」——只起录制进程，不申请控制权，不会和 VR 抢；
3. 戴头显，握 Grip 做演示，Trigger 控制夹爪；
4. 到点自动停止，或点「停止」；
5. 页面「已录数据」会立刻出现新条目（任务、tick 数、帧数）；
6. 命令行核对：`ls outputs/datasets/runtime/<episode>/`，应有 `episode.json`、`episode.mcap`、`*_rgb.mp4`。

## 10. 已知限制

- **A 键不要用**：实测按住 A 松开时，协调器记录的是 `OperatorAPlusB`（回 Home），会直接关闭跟随。模式切换在这套组合上不可用；进入 `Disabled` 只能重启内核。
- **XR 客户端是独占的**：`./run.sh check-xr`、任何直接调用 `xrobotoolkit_sdk` 的脚本都会抢走客户端连接，导致内核 `native input stale`。诊断请读内核自己的话题或日志，不要在遥操中调用 SDK。
- **相机二选一**：WebRTC 视频服务（8767）与共享相机服务（camera.sock）都独占 RealSense，不能同时开。采集走共享服务。
- **控制台重启会停掉它启动的内核**：内核会进入阻尼，机械臂需要重新托住。长期运行请用工作台的 robot 窗口启动内核。
