# 平台重构待办

QuestArmTeleop 从"多个项目合并后的仓库"整理为具身智能平台的工作清单。按架构分层规划，冗余随各层替代完成逐步删除。依据为 2026-10-21 对 `172982f` 的只读审查；执行前先核对条目引用的代码位置，代码可能已变化。

## 基本原则

1. **操控以 arx-televr 为准。** 上游 <https://github.com/GZF123/arx-televr>，固定在 `cd557afbb9ec280df2d9e12e1582219b85ee1a53`（`control/UPSTREAM.toml`），审查时与上游 HEAD 一致。
   - 范围：XR 输入与原生 coordinator/router、Disabled / FreeDrive / CommandFollowing 模式、Grip 相对跟随、Trigger 夹爪、`X5Model.solve_ik()`、关节增益、250 Hz worker、反馈检查、Home、退出阻尼、模型生成与校验。
   - 平台需要改动时依次选择：上游原样 → `control/adapters/` 外围适配 → 局部补丁（登记到 `control/LOCAL_PATCHES.json` 并写明理由）。
   - 旧链路（`robot_control`、`quest_teleop`）与上游行为不同的地方默认采用上游；确需保留的旧行为作为适配单独登记，不顺带改变上游控制。
2. **一个内核。** `control/` 的 worker 是唯一机械臂 SDK 所有者。输入、动作、标定、采集、策略和网页都是内核客户端，只通过 `runtime_client` 协议访问。
3. **单一来源。** 机器人参数、模型、数据格式和启动入口各保留一份。
4. **先替代，再删除。** 删除前确认没有运行时引用、替代方案可用；每次删除一个小提交，提交信息写明依据。
5. **安全控制统一在内核。** 限位、速度、工作空间、反馈检查、停止只在 `control/`（worker 与 gateway）实现。输入、策略、操作、标定等客户端不各自实现安全检查；需要新约束时加在内核，对所有来源生效。
6. **验证从简。** 遵循 `AGENTS.md`，只运行与改动直接相关的检查；真机动作由操作者执行。

## 目标架构

```text
 输入        Quest（原生 xrobot）| iPhone | 网页点动 | CLI
 操作        MoveJ / MoveL / Home / 示教点 / 标定动作
 策略        policy executor ── GPU 推理服务（独立环境）
 采集        只读订阅 tick 与相机帧
 界面        唯一前端
               │ 控制面  commands.sock：acquire / heartbeat / command / preview / execute / cancel / stop
               │ 状态面  RobotTick（版本化，带序号、来源与时间）
 ┌─────────────┴──────────────────────────────┐
 │ control/  arx-televr 内核（Python 3.10）       │
 │ worker 250 Hz · 租约 / epoch / TTL · 反馈检查   │
 │ backend: real（ARX SDK）| sim（MuJoCo）        │
 └─────────────┬──────────────────────────────┘
   control/config/{robot,tool}.json + control/models/x5/（同源生成并校验）
```

## 文件

| 文件 | 范围 |
| --- | --- |
| [framework.md](framework.md) | 阶段 1 框架草案：分层、目录、L0 契约、内核接口、平台包、统一入口、待决定项 |
| [00-guardrails.md](00-guardrails.md) | 基线标签、测试配置、引用检查、CI |
| [01-control-kernel.md](01-control-kernel.md) | arx-televr 内核：上游同步、适配边界、真机验收与切换 |
| [02-robot-profile.md](02-robot-profile.md) | 机器人参数、模型、平台配置的单一来源 |
| [03-kernel-clients.md](03-kernel-clients.md) | 客户端协议，以及输入、标定、网页后端、实验接入内核 |
| [04-sensors.md](04-sensors.md) | 相机服务与帧契约 |
| [05-data.md](05-data.md) | 采集、episode 格式、对齐与导出 |
| [06-policy.md](06-policy.md) | 推理运行时与动作执行 |
| [07-ui.md](07-ui.md) | 前端收敛 |
| [08-build-and-entrypoints.md](08-build-and-entrypoints.md) | 包结构、统一入口、注册表、环境 |
| [09-docs-and-repo.md](09-docs-and-repo.md) | 文档重组与仓库瘦身 |
| [redundancy.md](redundancy.md) | 冗余清单：保留项、删除条件和状态 |

## 推进顺序

先定框架，再按框架直接推进；真机验证集中放在最后。

| 阶段 | 内容 | 主要条目 |
| --- | --- | --- |
| 0 | 安全网 | G-01 – G-04 |
| 1 | 定框架：包结构、共享契约、机器人配置、统一入口，以及影响框架的待定项 | B-01、K-03、C-01、R-01、B-02、K-04、K-06、U-01 |
| 2 | 按框架推进：客户端接入内核、传感器、数据、策略、前端、注册表与环境；P0 随所在模块修复 | C、S、R、D、P、U、B、K-01 – K-05、K-07、K-11 |
| 3 | 集中真机验收，然后切换日常入口、退役旧内核 | K-08、K-09、K-10 |
| 4 | 文档与仓库瘦身 | O |

`redundancy.md` 贯穿全部阶段，满足删除条件即可处理。

### 真机验证推迟期间的规则

- 改动真机相关代码时，只做离线、假 SDK 或 MuJoCo 验证，并在条目中注明"未真机验证"。阶段 3 统一补做。
- 阶段 3 之前，现有真机日常入口（`启动机械臂服务.sh`、`启动遥操.sh`、`启动工作台.sh` 真机模式）保持可用、行为不变。新的真机路径可以先写好，但不作为默认入口。
- 旧控制内核（X-17、X-18）在阶段 3 验收通过后才删除，保证任何时候都有一条已验证的真机路径。

## P0 状态

审查列出的 P0 已在 2026-10-21 处理：C-03、D-02 已修复；P-02、R-04、K-07 复核后不是缺陷或已降级；C-04 工作空间部分已完成，手机急停复位握手仍待做；P-01、P-03、D-01 按用户决定处理。
## 条目格式

```markdown
- [ ] **ID** 标题 `优先级`
  - 现状：问题与代码位置
  - 做法：改动方向
  - 完成：可检查的完成标准
  - 依赖：前置条目
```

`[ ]` 未开始，`[x]` 完成并在条目末尾写提交号。需要决定的条目标 `待定`，写明选项。某个文件全部完成后删除，git 历史保留记录。
