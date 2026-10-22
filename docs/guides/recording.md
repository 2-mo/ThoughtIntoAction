# 数据采集、技能标注与复核

## 统一内核工作流

`./run.sh` 工作台默认打开带采集面板的新版监看页。已有内核运行时，也可在仓库根目录使用以下命令；`<RUN>` 是该内核打印的运行目录，示例中的相机 ID 必须已配置并由相机服务提供。

```bash
# 只读录制；可以只选择外部相机，不要求安装腕部相机。
./run.sh collect --run <RUN> --task "把物体放入托盘" \
  --cameras exterior_rgb --anchor-camera exterior_rgb --duration 30

# 无相机时记录状态与指令，供诊断，不产生视觉训练样本。
./run.sh collect --run <RUN> --task "运动诊断" --duration 5

# 同一次内核运行中，已结束的带相机录制可与任务事件生成待复核标注。
./run.sh examples annotate <EPISODE> <RUN>/tasks/<TASK_ID>/events.jsonl \
  --annotator operator --confidence 0.8
```

原始数据默认保存在 `outputs/datasets/runtime/episode_*/`。`episode.json` 保存运行身份、任务、录制状态、参考相机和 `data_contract`；`episode.mcap` 保存反馈、实际下发指令和相机帧索引，RGB 保存在对应相机的视频或图像文件。导出时仍需复制完整 episode 目录。

`data_contract` 描述关节名称与顺序、rad 单位、夹爪是否存在及米制范围。数据写入、对齐和导出按契约决定维度：七轴机械臂不截断，无夹爪机器人不补一维，未知夹爪上限不套用 ARX 标定。相机使用配置 ID，`--anchor-camera` 决定对齐参考。当前视觉训练导出仍要求 30 FPS 相机、有效因果时间匹配和允许的遥操来源；无相机诊断、RPC 脚本动作或被标记失败的录制不会因此自动获得训练资格。旧版六轴数据仍按其原始协议读取。

每路相机必须声明自己的 `driver`，不继承腕部相机配置。新版导出布局名为 `aligned30_runtime_v2`，每条报告的 `robot_timing` 分别给出配置控制频率与反馈、指令时间戳统计的平均采样率；缺口会降低该平均值，不能用它代替周期抖动分析。旧布局仍可读取。随仓库提供的 OpenPI 训练适配器仍面向六轴、夹爪和腕部图像；其他数据契约需要对应训练适配，通用导出不等于现成模型已支持任意硬件。

使用 `with robot.task(...) as task:` 编写的任务会将阶段事件和结果保存到 `<RUN>/tasks/<TASK_ID>/`，参见[操作原语](../reference/robot-primitives.md)。事件标注会核对内核运行 ID 和主机启动 ID，只取实际相机帧覆盖的阶段区间；不能把另一轮运行的时间戳拼进当前录制。`--confidence` 是显式提供的标签置信度，不是算法算出的成功率。

标注保存在派生目录 `annotations/skills.v000001.json`，记录阶段、对象、证据、帧范围和置信度，初始状态为 `needs_review`。在新版网页的“任务 / 技能复核”里点击“刷新记录”，可查看任务阶段、选择已保存的 episode 并批准或拒绝标注。每次复核新增版本；过期版本或已变化的源记录会被拒绝，原始 MCAP、图像和任务结果不会被覆盖。

网页顶部“实测速度 · 质量”显示关节反馈速度、反馈帧龄和连续 tick 的周期统计；命令与反馈的偏差保留各自时间戳，不能当作严格对齐的跟踪误差。相机源帧缺口与网页预览跳帧分开统计。界面和字段说明见[监看模块](../../src/embodied/applications/monitor/web/README.md)。

## 旧采集会话工作流

以下内容适用于 `src/legacy/` 的 presets 会话与 schema 1 数据，其按键、240 Hz 数据格式和会话端口不用于替代上面的统一内核接口。

旧入口是 `teleop-presets-x5`：A 开始／保存，B 预抓取／长按回零，摇杆管理任务和上一条作废，Grip 跟随／取消预设。会话层由这个遥操进程持有；录制命令和网页读取它发布的快照，不另外实例化机械臂。

本阶段实现原始 episode 的录制、回放、质量检查和结果标注。本地生成含全部条目的自说明原始压缩包；服务器转换为 30 FPS 观测及独立 240 Hz 动作数据，使用随包 OpenPI 适配器训练。完整模型训练和真机策略效果尚未验证。

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
./scripts/shortcuts/启动机械臂服务.sh

# 终端 1：原 presets 参数 + 会话和 D435i；这条命令会启用真机动作。
./run.sh --profile collection teleop-presets-x5 --actuate
# 或在原快捷脚本后加： ./scripts/shortcuts/启动遥操.sh --enable-session

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

默认实际数据位置为 `paths.dataset_dir`，本机是 `/home/r2/work/data/quest_x5/`。新采集原始备份位于 `.raw/`；结束任务后生成 `任务名_时间-编号.zip`，上传服务器后转换为 LeRobot：

```text
.raw/<episode-id>/
  episode.json        任务/结果/生命周期、模型、相机、实际控制参数、源码指纹
  episode.mcap       Zstd 无损压缩的机械臂状态与 RGB 帧索引
  wrist_rgb.mp4       单 D435i RGB，H.264 / libx264，CRF 20；逐帧时间索引保存在 MCAP
```

MCAP 使用 `/robot/state`（完整原始 JSON 状态）和 `/camera/wrist/rgb/index`（视频帧号、相机设备时间及主机接收时间）两个主题。`log_time` / `publish_time` 用录制开始时的墙钟与单调钟差值映射到 Unix 纳秒；原始单调时间仍完整保留在消息里，回放和训练对齐继续使用它。并不声称相机硬件曝光同步。

`episode.json` 是最终生命周期、结果及完整性标记的权威来源；MCAP 内保存开始录制时的元数据快照。视频为旁置文件，传输时保留整个 episode 目录，不能只拷贝 MCAP。当前不录制深度。录制、回放和训练导出仅支持 MCAP，不提供 JSONL / JSONL.gz 兼容分支；已有历史文件保留原样。

网页顶部「实时 / 回放」共用主三维场景、RGB 卡片和位置/速度/力矩面板。时间轴支持定位、暂停、0.25× / 0.5× / 1× / 2×；力矩窗口为播放时刻之前 5 秒，暂停时冻结，跳转时重建。缺失字段显示 `—`，缺失 RGB 保留占位，深度默认折叠。实际关节/夹爪速度直接读取反馈；末端线速度与角速度由同一末端坐标系的 URDF 雅可比计算。实时机械臂连接与 Quest 控制状态独立显示，网页回放不发送机械臂运动命令。当前网页回放的数据接口仍由遥操采集会话提供。

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

日常 `./scripts/shortcuts/启动遥操.sh` 默认开启会话，但控制和任务采集初始均未开启。待机期间只连接并监看，不发送机械臂或夹爪命令，A/B 和采集按键暂不生效。右摇杆持续下拨 1 秒启用后，重新从当前机械臂姿态建立参考；头显暂停／休眠后恢复需要再次下拨 1 秒。任务描述在脚本顶部 `COLLECTION_TASK` 设置（或传 `--collection-task`）。

| 操作 | 行为 |
| --- | --- |
| A 短按 | 任务未开启或已暂停时不响应；先左推右摇杆开启／恢复任务，再按 A 预抓取后开始新 Episode；录制中按 A 保存当前 Episode。 |
| B 短按 | 前往预抓取位，到位后不自动录制；录制中需先按 A 保存。 |
| B 长按约 1 秒 | 先丢弃正在录制的一条，再回 Home；不同时触发短按预抓取。 |
| 右摇杆左推 | 开启／恢复任务，不立即录制。重复左推不结束任务。 |
| 右摇杆右推 | 保存当前条并暂停任务，保留任务标识和计数。 |
| 右摇杆按下保持 1 秒 | 保存当前条、将本轮全部条目（含丢弃和失败标记）生成一个任务压缩包；成功后结束任务并清零计数。 |
| 右摇杆上推 | 作废上一条，保留文件与标注历史，计数相应减一；不影响当前录制。 |
| 右摇杆持续下拨 1 秒 | 待机时启用控制：先回中，再持续下拨 1 秒；持续保持只切换一次，回中后才能再次触发；松开 Grip、Trigger、A/B 和摇杆按键。已启用时再次下拨 1 秒发起平滑停车，取消预设动作，停稳后释放控制权并主动保持位置；采集任务与计数保持不变。 |

预抓取动作不录入 Episode。Grip 需释放才能启动预抓取；运动中握住 Grip 可取消动作及待启动录制。B、任务暂停／结束和头显暂停同样取消待启动录制。Home 等待 Grip 释放，运动中握住 Grip 可取消。扳机和 Grip 原有机械臂跟随／夹爪逻辑不变。

A/B 采用 30 ms 消抖；左／右／上推到阈值立即触发，无需保持；下拨切换控制需持续保持 1 秒；每次需回中后才能再次触发。启动时已偏转、斜推和 BUSY 时的新动作不排队；持续按住不重复触发。保存和作废在后台完成，不阻塞控制循环。保存失败时显示错误，不静默切换任务；丢弃失败不执行 Home。

保存使用 `outcome=unknown`，不表示任务成功；作废使用 `discarded`，保留原始文件且排除训练用途。重复作废上一条不会继续影响更早的数据。

## 单页采集监控与计数

`./scripts/viewer/启动仿真.sh` 的网页在 3D 场景下方大字显示任务未开启、已暂停、预抓取移动中、准备就绪、录制中和异常状态，以及本轮已保存条数、时长、RGB 帧数和状态样本数；显示当前／上一条 Episode、队列和写入错误。深度目前只预览，不计入录制帧数。网页只读，固定单页布局。

本轮已保存统计当前任务中完整保存且未丢弃的 Episode，不代表任务成功。暂停／恢复保留计数和 `task_run_id`；结束任务清零计数，文件保留。开启新任务生成新的 `task_run_id` 并清空上一条显示，不会跨新任务作废旧任务的数据。重启遥操从任务未开启和零计数开始。

网页监看采用三列一屏固定分区，姿态对照与数据回放始终展开；说明文字通过悬停显示。Quest 面板以模拟按键显示 X/Y/A/B/R3、摇杆方向、Trigger 和 Grip。B、R3、下拨的圆环进度来自控制程序计时，环绕对应按键显示，输入过期后清零。Trigger／Grip 显示百分比，回放通过秒数输入定位。Y 按下并松开后触发真机回放，面板显示输入已识别或拒绝原因；这不等同于已经完成真机运动。更新后重启遥操和网页监看服务以加载状态字段。

A 开始采集时先自动到预抓取位，到位后才录制；再次 A 先保存，保存成功后返回预抓取位，返位不录入本条数据。握着 Grip 时等待松开再自动移动；运动中握 Grip、切换待机或执行 B 等取消操作会取消待开始的录制。B 单独预抓取不触发录制。

训练导出、时间戳语义和 π0.5 接入见 [OpenPI 数据集说明](openpi-dataset.md)。

网页末端位姿包含 XYZ 与 XYZW 四元数，二者来自同一 URDF 末端坐标系的实际关节反馈；四元数显示保持符号连续，不重复写入 MCAP。电机诊断由独立只读 CAN 接收通道补齐：EC 电流随状态记录，温度 1 Hz，故障/有效性按变化写事件。DM 电流及母线电压缺失，详见 [电机诊断采集](motor-feedback.md)。
