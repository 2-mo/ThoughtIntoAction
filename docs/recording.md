# 命令行遥操采集与只读监看

主入口仍是 `teleop-presets-x5`：A 预抓取后开始，B 保存／长按回零，摇杆管理任务和上一条作废，Grip 跟随／取消预设。新增会话层由这个遥操进程持有；录制命令和网页读取它发布的快照，不另外实例化机械臂。

本阶段实现原始 episode 的录制、回放、质量检查和结果标注。本机 π0.5、LeRobot 数据导出和 H800 训练提交尚未接入；后续将使用这里保留的反馈、目标与指令定义适配。

## 先用虚拟设备演练

```bash
# 终端 1：虚拟反馈 + 合成 RGB，无需 Quest/CAN/相机；Ctrl+C 退出。
./run.sh session demo

# 终端 2：查看状态，开始/结束一条演练数据。
./run.sh session status
./run.sh record start --task "把物体放入托盘"
./run.sh record stop --outcome success

# 可选终端 3：打开打印的网址。
./run.sh monitor --port 0
```

`--port 0` 自动选择空闲端口；省略时使用 8080。合成图像直接标有 SYNTHETIC，数据单独写到 `outputs/demo-episodes/`。即使标记 success，也不会通过动作训练资格检查。

## 真机采集

```bash
# 只读检查环境、CAN 接口、USB、ADB、相机枚举、磁盘和已有会话。
./run.sh doctor --output outputs/logs/doctor.json

# 单独终端保持机械臂服务运行（初始化进入阻尼）：
./启动机械臂服务.sh

# 终端 1：原 presets 参数 + 会话和 D435i；这条命令会启用真机动作。
./run.sh --profile collection teleop-presets-x5 --actuate
# 或在原快捷脚本后加： ./启动遥操.sh --enable-session

# 终端 2：相机启动、机械臂快照就绪后开始录制。
./run.sh session status
./run.sh record start --task "把物体放入托盘"
# 使用 Quest 完成任务，然后选择实际结果。
./run.sh record stop --outcome success
```

失败用 `--outcome failure`，暂不判断用 `unknown`，作废用 `discarded`。以上均保留原始数据，不删除文件、不自动回位，也不会改变遥操使能或控制模式。需要阻尼/结束遥操时，在主遥操终端 Ctrl+C；机械臂服务和监看继续反馈。

不传 `--actuate` 时会话处于只读/阻尼，也能用于检查录制链路，但不存在可用于指令动作训练的完整执行数据。原 `presets` profile 默认不开相机和会话；`collection` 是显式选择的新档位。

机械臂与相机各有一个所有者。手眼标定与真机遥操需要先后运行，不能同时占有同一组设备。后台相机启动失败时，遥操可以继续，`record start` 会拒绝录制并给出原因。

## 检查、回放和复核

下面的 `EPISODE_DIR` 替换为 `record start/stop` 输出的实际目录：

```bash
./run.sh dataset inspect EPISODE_DIR
./run.sh dataset validate EPISODE_DIR --output outputs/logs/episode-validation.json
./run.sh dataset replay EPISODE_DIR
./run.sh dataset replay EPISODE_DIR --headless
./run.sh dataset label EPISODE_DIR --outcome success --note "回放确认任务完成"
```

回放显示原始 RGB、当前 URDF 下的反馈姿态和提交目标，按保存的 RGB 接收时间播放，`--speed 0.5` 可慢放。它不会连接 CAN；轨迹是反馈/提交记录，不会交给真机执行。q/Esc 或 Ctrl+C 退出。

验证包含文件/图像完整性、分辨率、模型哈希/坐标系/关节顺序、有限数值与限位、时间戳递增、源帧重复/丢失、状态/视频间隙、实际频率、首尾覆盖与接收时间对齐误差。结果分为：

- `valid`：通过当前完整性和数值检查。
- `outcome`：操作者给出的真实任务结果，检查程序不判断任务成功。
- `training_eligible`：valid、success、完整使能提交记录同时满足；仍不代表已导出为 π0.5 训练格式。

`label success` 会重新检查数据；损坏、未完成的 episode 无法被这条命令改为成功。标注会保留之前的结果、备注和时间，图像/状态保持原样。

手眼未标定不会阻止 RGB/关节数据通过检查。未知外参不会用单位矩阵代替；这些数据不能直接当作已标定的三维视觉定位数据。

## 保存内容

默认实际数据位置为 `paths.dataset_dir`，本机是 `~/work/datasets/quest_x5/`：

```text
<episode-id>/
  episode.json        任务/结果/生命周期、模型、相机、实际控制参数、源码指纹
  states.jsonl        每个控制周期的反馈、请求目标、提交记录与 SDK 输出
  frames.jsonl        图像路径、源帧编号、设备时间戳和主机接收时间
  wrist_rgb/*.png     单 D435i RGB，无损 PNG，保存 BGR 图像时使用标准 PNG 编码
```

每条状态明确区分：

- `actual_pos/actual_vel/actual_torque`：反馈；`gripper_actual_*` 是夹爪反馈。
- `requested_pos/requested_gripper_pos`：最近的输入目标；`source` 区分 Quest、预设和保持。
- `submission`：Python 最近一次成功交给 SDK 的完整 waypoint，包括限幅后的关节目标、夹爪位置/力矩/模式、提交序号和主机时间。
- `sdk_cmd_*`：SDK 内部插值和补偿后的输出快照，可能与 submission 不同。

不把 `state[t+1]` 偷换为 action。夹爪限力保持时，请求开度、提交位置和夹爪力矩一起保存，后续转换器可以明确选择动作含义。

相机设备时间和机器人 SDK 时间各自保存；对齐检查使用同机 monotonic 接收/采样时间，**不是硬件曝光同步**。启动内参来自实际 RGB 流。源码指纹覆盖项目 Python 文件与依赖锁，模型另有 URDF 哈希。

## 配置与运行边界

`configs/project.toml` 的 `[recording]` 提供默认值，本机可在 `local.toml` 覆盖：

| 参数 | 默认值 |
| --- | --- |
| 后台队列 | 128 条状态/图像事件 |
| 最低剩余空间 | 2 GiB |
| 单 episode 时长上限 | 120 s |
| 最短有效重叠时长 | 2 s |
| 状态最大间隙 | 0.1 s |
| 相机最大间隙 | 0.15 s |
| 接收时间最大对齐差 | 0.06 s |

编码和写盘在后台，控制线程不做文件写入或图像编码。队列溢出/写盘失败标记 failed；断流、低磁盘、达到时长上限或主进程退出会结束录制并保留 interrupted 数据。异常停止不自动标记成功，不删除原始数据。SIGKILL/掉电可能留下 recording 状态，验证会拒绝把它当成完整 episode。

会话诊断日志在 `outputs/logs/session-*/`，包括实际配置/源码指纹、约 5 Hz 状态快照和退出记录。完整控制周期数据保存在 episode 内。SDK 时间戳只能说明控制循环推进，不能证明每个电机最近收到新 CAN 帧。

网页绑定 `127.0.0.1`，只读显示相机、关节反馈/提交/误差、最近一分钟误差曲线和采集状态。关闭网页或监看服务器不影响遥操/采集。CLI 与会话通过权限为 0600 的本地 Unix socket 通信，接口仅支持状态、预览、录制开始/结束，不提供任意代码或电机指令入口。

## 本轮验证

覆盖原有 presets、模型/标定回归，以及虚拟反馈下的实际 Unix socket 通信、录制与按时间回放、队列溢出、图像写入失败、断流、重复启动、异常退出、只读 HTTP 和数据损坏检查。另通过独立 CLI 进程跑通 demo → record → monitor → validate → replay。

未执行真机动作。检查时 CAN 与 D435i 可见，ADB 未连接 Quest；真实遥操控制频率、相机写盘吞吐和训练动作定义仍需现场验收。

服务拆分后，采集保留服务采样时间 `host_monotonic_ns`，另存控制端接收时刻 `host_received_monotonic_ns`。`arm_service_id` 标识服务实例，`service_command` 是最近已在服务周期中处理的输入请求，`submission` 仍是最近实际提交给 SDK 的安全航点；不把异步请求确认当成运动已完成。控制参数中记录服务配置和有效速度上限。三维状态流仅用于监看，录制仍通过原会话写盘。


## 手柄采集流程

日常 `./启动遥操.sh` 默认开启会话，但任务采集初始未开启。任务描述在脚本顶部 `COLLECTION_TASK` 设置（或传 `--collection-task`）。

| 操作 | 行为 |
| --- | --- |
| A 短按 | 先前往预抓取位；任务开启时到位后自动开始新 Episode，未开启或暂停时只移动。录制中重复按 A 无动作。 |
| B 短按 | 结束并保存当前 Episode；没有录制时不操作。 |
| B 长按约 1 秒 | 先丢弃正在录制的一条，再回 Home；不同时触发短按保存。 |
| 右摇杆左推 | 开启／恢复任务，不立即录制。重复左推不结束任务。 |
| 右摇杆右推 | 保存当前条并暂停任务，保留任务标识和计数。 |
| 右摇杆按下保持 1 秒 | 保存当前条、结束任务并清零计数，保留所有文件。 |
| 右摇杆上推 | 作废上一条，保留文件与标注历史，计数相应减一；不影响当前录制。 |
| 右摇杆下推 | 预留，无操作。 |

预抓取动作不录入 Episode。Grip 需释放才能启动预抓取；运动中握住 Grip 可取消动作及待启动录制。B、任务暂停／结束和头显暂停同样取消待启动录制。Home 等待 Grip 释放，运动中握住 Grip 可取消。扳机和 Grip 原有机械臂跟随／夹爪逻辑不变。

A/B 采用 30 ms 消抖；方向推到阈值立即触发，无需保持；每次需回中后才能再次触发。启动时已偏转、斜推和 BUSY 时的新动作不排队；持续按住不重复触发。保存和作废在后台完成，不阻塞控制循环。保存失败时显示错误，不静默切换任务；丢弃失败不执行 Home。

保存使用 `outcome=unknown`，不表示任务成功；作废使用 `discarded`，保留原始文件且排除训练用途。重复作废上一条不会继续影响更早的数据。

## 单页采集监控与计数

`./启动仿真.sh` 的网页在 3D 场景下方大字显示任务未开启、已暂停、预抓取移动中、准备就绪、录制中和异常状态，以及本轮已保存条数、时长、RGB 帧数和状态样本数；显示当前／上一条 Episode、队列和写入错误。深度目前只预览，不计入录制帧数。网页只读，固定单页布局。

本轮已保存统计当前任务中完整保存且未丢弃的 Episode，不代表任务成功。暂停／恢复保留计数和 `task_run_id`；结束任务清零计数，文件保留。开启新任务生成新的 `task_run_id` 并清空上一条显示，不会跨新任务作废旧任务的数据。重启遥操从任务未开启和零计数开始。
