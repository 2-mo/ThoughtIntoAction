# 真机验收：arx 内核（K-08）

把日常真机入口从旧机械臂服务切换到 arx 内核之前，按本清单做一次现场验收。仿真里的同类检查（`./run.sh check all` 的 `sim` 组）已经通过；这里只验证必须由真机回答的问题。每一步记下结论与 `control/run/<RUN>/` 目录名。

## 准备

| 检查 | 命令 / 动作 | 通过条件 |
| --- | --- | --- |
| 旧服务已退出 | `./run.sh real shutdown` | 无输出错误；`./run.sh check ports` 中 `arm_service` 为空闲 |
| 内核与补丁台账 | `./run.sh kernel verify` | `"status": "verified"` |
| 夹爪修正 SDK | `./run.sh kernel build-sdk`（只需一次） | 编译成功；启动日志不出现“Gripper startup patch off” |
| 端口 | `./run.sh check ports` | 无冲突 |
| 现场 | 支撑机械臂、急停在手边、夹爪周围无物体 | — |

## 一、上游操控（原生 Quest）

```bash
./run.sh kernel real
```

| 步骤 | 预期 | 记录 |
| --- | --- | --- |
| 启动 | 使能后回 Home（`park_q`，默认全零），终端打印运行目录 | 回 Home 用时、是否平稳 |
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
./run.sh collect run --run control/run/<RUN> --task 验收 --cameras wrist_rgb,exterior_rgb --duration 20
./run.sh data runtime validate <输出的 episode 目录>
./run.sh data export --source <episode 所在目录> --output /tmp/acceptance-dataset
```

通过条件：`lifecycle` 为 `complete`；两路 MP4 存在；`validate` 返回 `"valid": true`；导出成功，`meta/quest_export.json` 中 `backends` 为 `["real"]`、`anomalous_episodes` 为空或缺口可解释。

## 三、多来源（managed 模式）

```bash
./run.sh kernel profiles start --profile real-quest      # Quest 经工作台控制权
./run.sh kernel profiles start --profile real-phone      # iPhone，UDP 5005
./run.sh motion run --run control/run/<RUN> status       # 动作客户端
./run.sh motion run --run control/run/<RUN> movej ...    # 小幅 MoveJ，先 preview
```

| 检查 | 通过条件 |
| --- | --- |
| 手机接管 | 手机获得租约后跟随；断网后按租约超时保持 |
| 桌面平面 | 手机或 MoveL 目标低于桌面时被 gateway 拒绝（错误信息含 table plane） |
| MoveJ / MoveL | 预览后执行，到位后保持；取消立即停止 |
| `describe` | `ControlClient(<RUN>).describe()` 返回的限位与 `control/config/robot.json` 一致 |

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

全部通过后，在 `todo/01-control-kernel.md` 勾选 K-08，并按 K-09 把 `启动工作台.sh` 真机模式、`启动机械臂服务.sh`、`启动遥操.sh` 默认切到内核；随后按 K-10 删除 `src/legacy/`。
