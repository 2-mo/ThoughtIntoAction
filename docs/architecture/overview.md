# 架构概览

## 分层

```text
L3 入口     ./run.sh <功能> <命令> · tmux 工作台 · apps/console · apps/ios · 18081 监看页
L2 平台     src/embodied/：inputs · phone · operations · motion · sensors · data · policy · calibration
            · monitor · console · workstation · tasks
L1 内核     control/：arx-televr（Python 3.10）——worker 250 Hz、IK、MuJoCo / 真机后端、gateway
L0 契约     src/embodied/contracts/：注册表、RPC、内核客户端、设备锁、运动 profile、机器人配置
```

- 下层不导入上层。内核只导入 `embodied.contracts`（`tests/test_convergence.py` 检查）。
- 只有内核打开机械臂 SDK；相机只由 `embodied.sensors` 的相机服务打开。
- 安全控制只在内核：关节限位、桌面平面、反馈检查、停止。客户端不各自实现安全检查。
- 操控行为以上游 arx-televr 为准；平台需要的改动依次选择：上游原样 → `control/adapters/` 外围适配 → 登记在 `control/LOCAL_PATCHES.json` 的局部补丁。

## 内核的进程与接口

```text
                    ┌───────────────────── control/（一次运行 = control/run/<RUN>/）─────────────────────┐
 Quest ─▶ rollio-device-xrobot ─▶ rollio-teleop-router ─▶ native ingress ┐                                 │
                                                                          ├─▶ worker 250 Hz ─▶ real / sim   │
 手机 / CLI / 动作 / 标定 / 策略 ─▶ commands.sock ─▶ gateway ─────────────┘        │                       │
                                                                                    ▼                       │
                                                       observer ◀── RobotTick（共享内存）                   │
                    └───────────────────────────────────┬────────────────────────────────────────────────┘
                                                        ▼
                                       observer.sock：采集、监看、诊断（只读）
```

| 接口 | 用途 |
| --- | --- |
| `commands.sock` | 长度前缀 JSON。`describe`（只读：限位、频率、模型、工作空间）、`status`、`acquire` / `heartbeat` / `release`（租约，带 epoch 和 TTL）、`joint` / `cartesian` / `gripper` / `free-drive`（流式目标；`free-drive` 可带 `gripper_width`，手臂自由、夹爪保持）、`preview` / `execute` / `cancel`（MoveJ / MoveL / Home；预览返回关节路径与 TCP 路径 `tcp`）、`hold` / `stop` |
| `observer.sock` | RobotTickV2 流：反馈、命令、来源、owner epoch、操作 ID；`latest` 或 `full` 模式 |
| `embodied.contracts.runtime` | `ControlClient`、`ControlSession`、`RuntimeReader`，平台侧唯一的内核客户端 |

右手柄由原生 coordinator 处理，语义与上游相同；工作台只加一层“允许 / 暂停”，并要求先松开全部按键。其他来源经 gateway 进入，统一检查限位、桌面平面与步长。

## 数据流

| 阶段 | 位置 | 产物 |
| --- | --- | --- |
| 采集 | `embodied.data.capture`（内核 tick）、旧链路会话（`src/legacy/quest_teleop`） | episode：MCAP（反馈、命令、相机帧时间）+ 视频 / PNG + `episode.json` |
| 查看与回放 | `embodied.data.datasets`、`embodied.data.replay` | 质量报告、标注 |
| 导出 | `embodied.data.export` | LeRobot v2.1 + `robot/*.npz` 原始样本 + `meta/quest_export.json`（action 定义、控制栈、相机、缺口） |
| GR00T | `embodied.data.groot` | 双相机 Rot6D 16 维数据集 |
| 训练 | `integrations/openpi`（H800） | checkpoint |
| 推理 | `embodied.policy.pi05_sft`、`pi05_zeroshot`、`groot` | 机械臂动作 |

原始数据从不改写；格式变化只增加新的读取适配。

## 配置

| 文件 | 内容 |
| --- | --- |
| `control/config/robot.json`、`tool.json`、`workspace.json` | 机器人参数、TCP、桌面平面（内核与平台的唯一来源，`embodied.contracts.robot` 读取） |
| `control/models/x5/` | 由 `control/scripts/build_model.py` 生成并校验的 URDF / MJCF |
| `configs/project.toml` + `configs/local.toml` | 平台：路径、端口、相机、旧服务参数；本机覆盖不提交 |
| `configs/plugins.toml`、`control/config/components.json`、`configs/components/*.json` | 组件注册与命令 |
| `configs/workbench/*.json` | 内核工作台组合 profile |

## 现状与计划

真机日常仍走旧机械臂服务；内核真机入口待验收。客户端改走内核、重复实现合并和旧链路删除的计划见 [todo/](../../todo/README.md)。
