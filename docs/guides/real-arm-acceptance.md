# 真机验收：arx 内核（K-08）

把日常真机入口从旧机械臂服务切换到 arx 内核之前，按本清单做一次现场验收。统一入口是 `./run.sh`：打开 tmux 工作台、监看网页和各设备状态窗口，运行目录自动关联。仿真里的同类检查（`./run.sh check all` 的 `sim` 组）已经通过；这里只验证必须由真机回答的问题。每一步记下结论与 `outputs/runtime/control/<RUN>/` 目录名。

## 工作台内完成本轮测试

在项目根目录运行 `./run.sh`，无需另外开终端启动网页或复制 `RUN`。

| 总控按键 | 行为 |
| --- | --- |
| `3` | 启动无 Quest 的真机内核，按内核窗口提示确认；初始保持当前位置 |
| `M` | 读取当前状态与设备能力 |
| `B` | 关节 1 转动 10° 后返回，使用后端每轴速度上限，生成性能报告 |
| `X` | MoveL：沿基座 Z 方向上移 2 cm，TCP 速度上限 0.1 m/s |
| `T` | 连续路径：Z +2 cm，再 Y +2 cm，转角半径 5 mm，TCP 速度上限 0.1 m/s |
| `W` | 打开新版监看网页，显示网页服务状态与日志 |

`B/X/T` 先把完整命令填到窗口 2，按回车开始；也可直接修改参数。动作完成后 `Ctrl+B → 1` 回总控。
重复 X/T 会从新的当前位置继续移动。测速结果及原始 tick 保存在 `outputs/benchmarks/`，用于判断速度、跟踪、周期和停顿候选；无需凭页面刷新速度猜测控制周期。
网页随工作台启动并自动跟随运行中的内核；本机桌面待服务监听后请求打开浏览器，SSH 在客户端打开窗口 4 的地址。
动作中在窗口 2 按 `Ctrl+C` 取消；窗口 3 的退出操作会关闭内核并进入阻尼，退出前先支撑机械臂。

以下命令作为各功能的独立调用参考，日常操作优先使用总控按键。

## 准备

| 检查 | 命令 / 动作 | 通过条件 |
| --- | --- | --- |
| 工作台已关闭 | `./run.sh sim shutdown`（与 `real shutdown` 相同；仿真工作台自带的 MuJoCo 内核会占用 Quest） | `./run.sh check ports` 中 `arm_service`、`monitor` 为空闲 |
| 内核与补丁台账 | `./run.sh kernel verify` | `"status": "verified"` |
| 夹爪修正 SDK | `./run.sh kernel build-sdk`（只需一次） | 编译成功；启动日志不出现“Gripper startup patch off” |
| 端口 | `./run.sh check ports` | 无冲突 |
| 现场 | 支撑机械臂、急停在手边、夹爪周围无物体 | — |

启动会使能电机。原生 Quest 模式会回 Home 并打开夹爪；managed 模式初始保持反馈姿态与夹爪宽度，之后由接入的输入来源决定动作。`real-manual` 默认没有输入来源，适合先验收原位保持，再由动作客户端逐步执行小幅运动；自定义 profile 的启动行为还需检查其输入配置。Ctrl+C 退出请求 SDK 阻尼，退出前先支撑机械臂。

## 一、上游操控（原生 Quest）

内核启动后打印运行目录 `outputs/runtime/control/<RUN>`；其他终端可以这样取最新一个：

```bash
RUN=$(ls -td outputs/runtime/control/*/ | head -1)
```

```bash
./run.sh kernel real
```

| 步骤 | 预期 | 记录 |
| --- | --- | --- |
| 启动 | 使能后回 Home（`park_q`，默认全零）并打开夹爪，终端打印运行目录 | 回 Home 用时、是否平稳 |
| 右手 A 按住 ≥150 ms 后松开 | 按上游：Disabled 后进入 FreeDrive，之后在 FreeDrive / 跟随间切换（仿真工作台的 managed 模式实测首次即进入跟随） | 实际顺序 |
| FreeDrive | 重力补偿，可用手推动 | 是否下坠或抖动 |
| 跟随：握住 Grip 小幅移动 | 末端跟随；松开 Grip 保持当前位置 | 延迟感、是否有跳变 |
| Trigger | 夹爪随扳机开合 | 开合范围、是否卡顿 |
| A+B | 回 Home，夹爪打开 | — |
| 遮挡头显 / 手柄失去跟踪 | 保持，不跳变；恢复后需重新握 Grip | — |
| Ctrl+C | SDK 阻尼退出（先支撑机械臂） | `report.json` 中 `error` 为空 |

## 二、采集

内核运行时，在另一个终端：

```bash
./run.sh collect run --run outputs/runtime/control/<RUN> --task 验收 --cameras wrist_rgb,exterior_rgb --duration 20
./run.sh data runtime validate <输出的 episode 目录>
./run.sh data export --source <episode 所在目录> --output /tmp/acceptance-dataset
```

通过条件：`lifecycle` 为 `complete`；两路 MP4 存在；`validate` 返回 `"valid": true`；导出成功，`meta/quest_export.json` 中 `backends` 为 `["real"]`、`anomalous_episodes` 为空或缺口可解释。

## 三、多来源（managed 模式）

先选择一个 profile 启动。首次小幅动作验收建议用 `real-manual`：初始保持启动姿态与夹爪宽度，待现场确认稳定后，再预览并执行动作。Quest、手机或自定义来源接入后可能发出动作命令，不能仅根据 managed 模式判断会持续保持。

```bash
./run.sh kernel real --profile real-manual              # 无输入来源，初始保持；后续由客户端发命令
./run.sh kernel profiles start --profile real-quest      # Quest 经工作台控制权
./run.sh kernel profiles start --profile real-phone      # iPhone，UDP 5005
./run.sh motion run --run outputs/runtime/control/<RUN> status       # 动作客户端
./run.sh motion run --run outputs/runtime/control/<RUN> movej ...    # 小幅 MoveJ，先 preview
```

| 检查 | 通过条件 |
| --- | --- |
| `real-manual` 启动保持 | 没有来源或客户端发动作命令时，机械臂与夹爪保持启动状态，无下坠、持续抖动或异响 |
| 手机接管 | 手机获得租约后跟随；断网后按租约超时保持 |
| 桌面平面 | 手机或 MoveL 目标低于桌面时被 gateway 拒绝（错误信息含 table plane） |
| MoveJ / MoveL | 预览后执行，到位后保持；取消立即停止 |
| 运动平顺性 | 现场无停走、突进或抖动；记录规划位置 / 速度、实际下发位置 / 速度与反馈，区分规划、下发和电机跟随问题 |
| 运动速度 | 记录请求速度、规划峰值速度、实际运动时间与反馈跟随误差；短距离可能受加速度 / jerk 限制，不能只用完成状态判断速度达标 |
| `describe` | `ControlClient(<RUN>).describe()` 返回的限位与 `configs/robots/arx5/robot.json` 一致 |

动作到位条件读取 `configs/control/motion.json` 的 `settling`：默认位置误差小于 `0.0035 rad`（约 `0.20°`），反馈持续稳定 `0.25 s`。正常完成及释放后保持最终目标；超时、取消、失联则撤销旧目标并保持当前反馈位置。平顺性、速度和到位精度分别验收，不能仅凭 `completed` 判断平顺。测试在明确倒计时后的固定窗口进行，用日志统计首次运动延迟、中途位置平台、实际速度、终点误差和控制周期，避免零散动作与反复询问可直接观测的数据。

速度验收同时记录请求参数、`describe.motion_limits` 的每轴后端能力及预览 `metrics`。`--speed 0` 表示使用后端能力，并不保证短行程达到该速度。单关节 10°、速度 `0.5 rad/s`、加速度 `2 rad/s²`、jerk `10 rad/s³`：原九次轨迹为 `1.112 s`，共享 Ruckig 离线规划为 `0.824 s`。这只是相同参数下的离线对照；新执行链尚需真机验收。SDK 未提供加速度与 jerk 额定字段，不能把应用参数标成硬件额定值。配置在内核启动时加载；真机退出会进入阻尼，重启前先支撑机械臂。

## 四、网页控制台

```bash
./run.sh web console       # http://127.0.0.1:8000，页面上点“连接”
```

| 检查 | 通过条件 |
| --- | --- |
| 连接 / 断开 | 连接后关节角、末端位姿、夹爪宽度与内核一致；断开后内核保持当前位置，租约释放 |
| 单关节与 XYZ | 预览轨迹与实际一致；执行到位后保持；急停按钮取消运动并保持 |
| 自由阻尼 | 切到阻尼可手推，切回关节位置模式后保持 |
| 夹爪 | 位置控制到目标宽度；“限扭夹取”为位置流到停滞（无力矩限制），夹住物体时观察电机温度与电流 |
| 手势遥操与拖动伺服 | 跟随平顺，停止后保持；超出 0.15 m 的目标被内核拒绝 |

## 五、标定（可选）

```bash
./run.sh calib handeye capture wrist_rgb --auto       # 标定板固定在桌面
./run.sh calib handeye capture exterior_rgb --auto    # 标定板夹在夹爪中
./run.sh calib handeye status
```

步骤见 [手眼标定](calibration.md)。

| 检查 | 通过条件 |
| --- | --- |
| 示教 | 手臂可手推，夹爪保持当前宽度（夹住的标定板不掉） |
| 自动采样 | 每个采样位到位后保持，静止采样通过；被内核拒绝的目标跳过；Ctrl+C / 急停后原地保持 |
| 结果 | 两个相机均为 `validated`；网页控制台三维场景中相机位置与实物一致 |

## 结论

全部通过后，在 `docs/plans/01-control-kernel.md` 勾选 K-08，并按 K-09 把 `scripts/shortcuts/启动工作台.sh` 真机模式、`scripts/shortcuts/启动机械臂服务.sh`、`scripts/shortcuts/启动遥操.sh` 默认切到内核；随后按 K-10 删除 `src/legacy/`。
