**多输入与运动功能接入方案**

> 实施更新（2026-09-29）：本文保留最初的设计推导和扩展目录，并非当前完成清单。插件框架、手机、多源控制权、MoveJ/MoveL、arx 标定适配和 schema 2 LeRobot 导出已实现；实际用法和验收范围以 [插件工作台](plugin-workbench.md) 与 [实施记录](arx-runtime-implementation.md) 为准。

> 范围更新：三个参考仓库的完整功能审查见 [插件迁入清单](plugin-migration-inventory.md)。以下“第一批/第二批”与实施前语句保留为历史设计，不代表当前进度，也不再作为功能取舍的完整范围。


日期：2026-09-29。状态：历史设计提案，部分已实施。配合 [主架构方案](arx-runtime-redesign.md) 使用；手机与基础运动的当前状态见 [实施记录](arx-runtime-implementation.md)，其余扩展以新清单为准。三个参考仓库保持不变。

**扩展目标是一个控制内核支持多种操作。** arx-televr 提供 X5 的机器人循环、IK、SDK 和 MuJoCo 基线；其他项目提供设备输入、交互功能、运动原语和实验方法。所有实际运动最终由同一个 worker 提交，所有记录使用同一套时间和数据契约。

**先直接复用完整 arx Quest 链路，再接入本页功能。** 包括 XR 输入、coordinator/router、相对映射和按键状态机在内的上游实现均优先保留。手机、点动、通用动作 API 和控制权切换从主方案阶段五开始实施；本页“第一批”指扩展的第一批，不是整个迁移的第一批。每项先尝试配置或边界适配，确需修改 worker 时登记局部补丁，并保留原 Quest 基线的独立回归入口。

所有选用代码统一交付在 QuestArmTeleop：Swift App/Xcode 工程放入 `apps/ios/`，协议与设备适配放入 `src/embodied/inputs/`，运动算法按环境边界纳入操作模块或 `control/motion/`。来源和补丁固定记录，构建及运行不引用兄弟仓库。现有 `src/embodied/calibration/wrist/` 的手动/自动采样、计划、求解与独立验证同样保留，详见主方案的自动手眼标定迁移；它与后续负载辨识分别交付。

各功能的依赖方向、状态所有权和独立验证见 [组件边界与独立优化方案](component-boundaries.md)。运动操作接口供手机、网页、标定和策略使用；相机、采集、标定流程与模型推理保留各自模块，不集中到一个操作管理器。

本轮参考工作区：arx5_test HEAD `a18d6b67906e450411d56e3d0fd08fd18a01675c`，reBotArm_control_py HEAD `0324b734c86866ea23d6ea4c097833739329d6d3`，robot-teleop HEAD `864d6501a3eeef99b7facdc8f592d80e0ff52843`。三者均有本地修改；实施时还需固定实际导入文件、依赖和来源，不能仅用 HEAD 声称完全可复现。

**优先接入已存在且边界清楚的能力。** 下表区分操作能力与需要带入的实现范围。

| 来源 | 值得纳入的能力 | 复用边界 | 优先级 |
| --- | --- | --- | --- |
| robot-teleop | iPhone ARKit 位姿、Clutch、Recenter、夹爪、比例设置、延迟状态 | Swift App、UDP codec、sequence/session、时钟估计、相对映射与复位握手；不用其第二套 ARX SDK 后端 | 第一批 |
| arx5_test | 单关节点动/绝对目标、末端位置与姿态编辑、参考点、Ghost 预览、执行/取消 | 交互/API 设计、独立数学和计划结构；所有设备执行改接 arx worker | 第一批 |
| arx5_test | 夹爪位置打开、限扭矩闭合/保持、速度档位 | 明确单位和后端能力，作为 arx 夹爪策略与 motion profile | 第一批/动作阶段 |
| arx5_test | MoveJ/MoveL/MoveC、waypoint、圆角、TCP/工件坐标、预览计划校验 | 提取 `motion/` 的纯算法和契约；执行器及生命周期保持唯一 | MoveJ/L 先，MoveC/混合路径后 |
| arx5_test | 浏览器摄像头手势、离合、夹爪手势、只预览模式 | MediaPipe/UI 输入适配、状态和消息；不带入原 FastAPI 硬件所有者 | 第二批 |
| arx5_test | 固定工具与额外负载辨识、命名档案 | 采样计划、静态质量/质心拟合、质量诊断；输出转为统一工具配置 | 第二批 |
| reBotArm | 自由拖动、静止后保持、终端输入末端目标、轨迹实验 | 操作流程和算法对照；在 X5 上重新实现/校验策略 | 普通拖动与示教先，自动保持实验后 |
| reBotArm | SE(3) 路径、CLIK、重力/动力学工具 | 可选离线算法实验；使用统一 X5 模型，不覆盖主 IK | 第三批 |

“自由拖动”在 arx FreeDrive 已有基础；保存示教点、动作编排和静止后保持是新增功能。输入枚举中出现某种设备不代表已经支持，不提前宣称 SpaceMouse、Android 或其他机械臂可用。

**控制来源、操作类型和执行模式分别建模。** 避免把 `phone`、`FreeDrive`、`recording` 混在一个模式枚举里。

| 维度 | 示例 | 所有者 |
| --- | --- | --- |
| source | quest、phone、web、cli、hand、policy、replay | 输入/操作适配器 |
| operation | teleop、jog、trajectory、teach、calibration、replay | 工作台操作会话 |
| backend mode | arx 的 Disabled、FreeDrive、CommandFollowing；明确的 session hold/stop 行为 | worker 与后端 |
| recording | idle、recording、saving、failed | collection-service |

录制可以和合法操作并行，记录 source/operation；来源切换时首版关闭当前 episode，再开启新条目，避免把不同映射策略静默混成一段示教。

**在 worker 内加入轻量控制权检查，保持设备连接常驻。** 多源接入需要的新逻辑集中在命令入口，不把网页/手机业务搬进 IK 循环。

1. 多个来源可以连接、接收状态和做 Ghost 预览。同一时刻仅一个 owner 可以提交 arm 与 gripper 目标，首版不拆分两者控制权。
2. owner 由 `runtime_id + source_session + epoch` 标识，带主机单调时钟上的有效期。worker 每 tick 验证一次；普通连接心跳不能让旧姿态无限续期。
3. 切换来源必须先撤销旧 epoch、清理待处理命令、完成停止/保持过渡并检查反馈，再建立新映射参考和新 epoch。旧来源迟到的动作及模式消息一律拒绝。
4. 普通来源切换不重新构造 SDK，不调用启动 Home，也不将“取消/松手”翻译成 arx Disabled。Quest 原生 coordinator 的模式入口也受同一 owner 控制。
5. 新 owner 需要输入释放后重新激活；第一目标锚定当前反馈。跟随目标、IK seed、输入参考与上次源绑定，交接时同步重置。
6. 已进入 arx `request_stop()` 锁存的 worker 不通过切换 source 解锁。故障恢复走显式恢复流程；确需重建运行时，生成新的 runtime_id。

这里的常驻交接尚不是 arx 原有功能，也不是首轮复用的前置条件。阶段五先在仿真和假 SDK 中验证模式 ACK、反馈锚点与暂停行为。支持常驻之前，可保留显式停止/重启的临时入口，但必须展示可能发生的启动 Home，不能假装无缝切换。

owner 规则适用于运动、夹爪和普通模式请求。本机操作员/supervisor 的授权停止走独立管理入口，不以原 owner 仍在线为条件；cancel 则只作用于相应 operation。多源阶段，未活动输入故障不直接停止当前来源；当前操作依赖的反馈或相机失效，按该操作的终止规则处理。

**共用命令契约，保留输入设备自身语义。** 建议使用下面几种请求，不用一个可随意组合字段的“大字典”覆盖所有控制。

| 请求 | 内容与语义 | worker 处理 |
| --- | --- | --- |
| CartesianTarget | base 中 TCP 绝对目标、有效期、可选夹爪宽度；来源已完成相对映射 | arx IK 与原关节目标策略 |
| JointTarget | 六轴有限 rad 目标、明确运行 profile | 受选定运动策略约束；不再做 IK |
| JointTrajectory | motion_id、起点约束、时间戳及 q/dq 采样、终止行为 | 校验后由唯一执行器按本机时间采样 |
| Jog | 轴/坐标系、方向、速度或步长、按住有效期 | 固定步长作为操作；连续点动使用 latest-only 意图，不积压增量 |
| GripperCommand | width_m 或 torque_limit_nm 等明确策略 | 同一 owner 和夹爪能力检查；不可把 N·m 标为指尖力 N |
| SessionAction | acquire/release/cancel/hold/home/stop/reset 等离散事件 | 分别定义行为；Home 与停止分开，事件有 ACK 和幂等 ID |

公共头包含 protocol version、runtime/source/session/epoch、sequence、主机接收时间和有效期、model/TCP/profile revision。过期、错源、错版本、无支持能力的请求明确拒绝，不静默降级为另一种动作。

上表是扩展契约目录，按功能逐项实现。手机先增加 CartesianTarget 的适配和必要会话操作；点动、关节动作、轨迹执行到各自接入时再增加相应处理，不在首次导入 arx 时预建全套通用执行框架。

实时目标使用 latest-only；执行、取消、模式切换使用有界且可靠的事件通道；录制使用完整有序流；页面状态使用降采样快照。事件 ACK 表示已接收/已生效/已完成中的哪一级必须写明，不能用网络送达代替动作完成。

耗时操作通过 operation ID 查询权威状态。请求超时表示结果未知，应查询同一 ID；不得以新 ID 自动重发运动。取消经过 cancelling，在执行端完成约定终止行为后再报告 cancelled；进程失联不能当作物理停止已确认。详细的资源准备、状态和重试规则见组件方案。

Quest 原生 router 的绝对目标可在 worker 原有入口包装身份后使用，保持控制路径短。手机或手势的绝对目标接入同一入口，不再通过 `XRPublisher` 伪造 A/Grip 事件。两者都只进行一次参考捕获和一次目标映射。

**手机接入优先保留已有 App 和协议。** robot-teleop 的实际能力是 iOS ARKit，不是普通手机浏览器。其 App 当前面向 iOS 26，Android 需要独立实现，首轮不扩展。

建议路径：Swift ARKit/UDP；Python 3.12 的协议和会话适配；已授权的相对映射；CartesianTarget；arx worker。手机适配不加载 `robots/arx5.py`、其 IK/Ruckig 或 SDK。源码中已经存在受配置开关控制的实际 ARX 命令路径，不能依据较旧 README 将整个项目判断成 Mock-only。

需要保留/适配的协议行为：

- Python/Swift 的字节级 golden vectors、精确报文长度、有限值/四元数校验、会话与序号回绕检查。
- 每个新 ARFrame 最多一个新样本；UDP 批次处理离散控制后只发布最新有效 Pose。重复收包不能刷新姿态年龄。
- session 变化、追踪恢复、Clutch 释放后重新锚定；重连或从后台恢复时不自动恢复跟随。
- SettingsAck 返回实际采用的缩放参数；活动跟随时不因滑块更新跳变，下次 Clutch/Recenter 才应用。
- Ping/Pong 估计时钟偏移与 RTT；未同步时只报告主机接收后的年龄。手机 monotonic 不能直接作为 worker deadline。
- ESTOP/RESET 保留锁存和匹配 ACK。手机请求复位不能独自解锁后端；收到 ACK 前 UI 仍显示锁定。arx 的停止锁存恢复需要独立适配和验收。
- 手机 Home、关节回零等已有操作按显式事件接入；不能和 Recenter 混用。未知/未支持事件返回 unsupported。

当前网络协议没有内建认证和加密。部署沿用受控局域网或现有 Tailscale 边界，并约束获准的来源；session_id 只用于会话排序，不当作身份认证。新增的公网访问不属于此次迁移范围。

手机项目当前 `arkit_arx5` profile 使用工具参考系：手机顶端对应工具 +X，手机右侧对应工具 -Y，屏幕法向对应工具 +Z。代码同时支持 base 输出。首轮保存原 `phone_tool` profile，另设显式 `phone_base` 对照，不因为 Quest 使用 base 就悄悄修改手机方向。新框架最终目标统一为 base 中 TCP pose，记录 mapping_profile 和输入参考系。

手机 gripper 的 `[0,1]` 在其 ARX 后端映射为 `[0,max_width]`，即 0 闭合、1 张开。Quest Trigger 越深越闭合，语义相反。适配器在边界统一为米，不能直接复用 trigger 数值。

**手动控制和轨迹规划复用 arx5_test 的分层思路。** 第一批提供关节正负点动、绝对关节目标、base/tool 下末端小步移动、姿态编辑、夹爪开度、保存/返回示教点、Home 和取消。终端与网页共享 operation API；监看默认只读，控制面板显式获取 owner。

示教点保存 q、TCP pose、夹爪宽度、模型/tool/profile 身份及采样来源。返回示教点是规划动作，不能把历史 q 当成可以立即跳到的目标。自由拖动录制标记 `source=teaching`，没有位置指令时不伪造 commanded action；若后续以实测轨迹构造训练动作，另设导出 action_definition。

预览返回 `plan_id + plan_hash`，绑定起始 q/dq、模型/TCP、负载与运动限制版本、owner_epoch、失效时间。执行使用同一份计划并重新检查起点和上下文，不重新求解一条与预览不同的路径。纯 Ghost 预览没有执行权限，也不抢占活动遥操使用的可变 IK/MuJoCo data。

现有微调推理入口的 `--preview` 会先预抓取并张爪，不能直接复用成这里的 Ghost。迁移时明确区分无动作预测、仿真显示和预抓取操作，模型加载/预测本身不取得机械臂控制权。

MoveJ/MoveL/MoveC、waypoint 和圆角作为可组合运动原语。arx5_test 的 Ruckig、时间参数化和独立几何模块用于规划动作；不将其在线平滑重新串入 Quest 跟随。纯算法可以提取；强依赖 Pinocchio/Coal 的规划放入独立可选环境，仍由统一生成的 X5 模型派生，避免把不同 Pin/NumPy ABI 强塞进 arx 环境。

速度档位保存真实的 rad/s、m/s、加速度和 jerk 上限。UI 的“精细/普通/快速”只是所选 profile 的显示名，不能直接复制另一台机器的比例或临时倍数。夹爪策略与 Home 行为也纳入能力和 profile。

**浏览器手势排在手机之后。** arx5_test 已有 MediaPipe、离合状态、夹爪手势、preview-only 和约 30 Hz 目标发送逻辑。其 README 的部分手势描述与当前代码不同，应以冻结的前端版本和手势实测为准，不承诺沿用“捏合”等未核对映射。

迁移时保留手丢失/重捕获、输入年龄、会话和序号；浏览器前台/后台切换必须失效并要求重新激活。后端仍检查坐标、有效性和目标，不只信任前端置信度。网页摄像头的估计尺度不是 ARKit 的公制定位，单独校准映射 profile，不共用平移比例。先交付 Ghost，再启用控制。

**reBotArm 的操作思路可借鉴，具体实现需要修正与适配。** 它针对 B601 和 motorbridge，不直接复用其 actuator、CAN ID、零点工具、增益或设备启动/退出代码。

本轮审阅发现三个具体问题：

1. `RebotArmEndPose._loop_cb()` 对 J2/J3 重力前馈乘以 1.55，这是特定模型补偿，不应用到 X5。arx SDK 已有自动重力，接入新的重力算法还需明确关闭或替换原重力项，避免相加两次。
2. `10_gravity_compensation_lock.py` 用 `_integral += q_error * 1.0` 按循环累加，没有 dt 缩放。直接改成 250 Hz 会改变其积分效果；速度阈值和增益也需要重新验证。停稳保持只能作为显式实验策略，不能称为经过标定的力控。
3. `sampler.py` 的 TRAPEZOID 分支有数值问题。独立提取该纯函数验算，`accel_ratio=0.25` 时 `s(0.75)=1.6666667`，`s(0.750001)=0.6666693`；既超过 `[0,1]`，又不连续。该分支不能迁入默认轨迹库。

SE(3) 测地线与 Cartesian 直线也不是同一种路径。复用 reBot 的路径/CLIK时需保留类型区分，验证位置和姿态残差、连续性、限位和时间参数化。笛卡尔五次多项式的名称包含 min-jerk，不等于 IK 后各关节满足其速度、加速度和 jerk 上限。

建议先提供明确的 FreeDrive/位置保持切换、保存示教点和拖动录制。之后再评估“移动时跟随实际位置、停稳后保持”的辅助策略；没有力传感器时不把速度阈值解释成接触力检测。

**工具和负载档案可以补齐主方案。** arx5_test 的 baseline/loaded 静态力矩差、质量/质心拟合与质量指标，适合成为独立标定操作。输出先成为待采用的 profile；显式采用后更新 tool.json，生成并校验 URDF/XML，再启动使用新模型的运行。

静态重力数据只能辨识质量和一阶矩，不能生成已测转动惯量。模型变更不能在运动中热更新；历史示教点、预览计划、标定和录制继续关联原模型版本。

**功能发现与界面统一，避免重复建设。** 后端 capabilities 按来源和后端分别报告：支持、未实现、仅仿真、实验、不可用原因，以及允许的命令类型。UI 据此显示控件；不支持 torque-close 的后端不显示可用按钮，不支持的动作也不偷偷替换为位置闭合。

保留当前监看页与采集页，逐步加入“输入来源、当前 owner、操作模式、手动控制、预览/执行、示教点”。无需同时搬入 arx5_test 整个 Vue/FastAPI 产品。其当前 `kernel/control.py` 约 6000 行、`controller.py` 约 4000 行，直接整合会重新引入硬件、相机、标定、任务的耦合。

**交付顺序与验收具体到功能。** 前提是主方案阶段零至四的 arx Quest 基线已交付。A、B 属于阶段五，C、D 属于阶段六；各批作为独立增量，不反过来阻塞原遥操链路的复用。

| 批次 | 内容 | 关键验收 |
| --- | --- | --- |
| A | 控制权入口、iPhone、关节/末端点动、夹爪、Ghost | 多源只一方能执行；手机协议金样本与原 App 兼容；坐标/夹爪方向一致；切换无跳变/隐式 Home；松键、后台和重连失效正确 |
| B | 示教点、MoveJ/MoveL、预览执行、夹爪限扭矩、采集来源标签 | 预览与执行同计划；上下文改变拒绝旧计划；取消无残留目标；不同来源的 raw/command/action 语义可追溯 |
| C | 浏览器手势、MoveC/路径混合、负载档案 | 先 Ghost；路径/时间/关节约束通过；负载结果不伪造惯量；原 Quest/手机基线仍通过 |
| D | 自动停稳保持、备选 CLIK、动力学实验 | 独立 profile 和开关；重力只加一次；不同周期效果受控；已知轨迹缺陷修正后才参与对照 |

不将所有参考项目作为运行依赖安装。导入到统一操作库的函数、App/协议快照分别固定来源和许可；复制实现与借鉴设计分开登记。默认控制内核保持一套 IK、一套模型生成和一个 SDK owner。

现有手眼标定在阶段五获得必要控制权与轨迹执行能力后即可恢复，不等待 B 批全部功能或 C 批负载辨识。自动采样保留独占控制、位置保持、夹爪开度、到位采样及明确的结束/异常行为；完成迁移验收前保留旧标定入口。

**源码依据与验证范围。** 以下链接指向当前本地参考工作区，不构成运行时路径依赖。

- arx5_test：[动作契约](../src/embodied/motion/contracts.py)、[几何](../src/embodied/motion/geometry.py)、[速度参数](../src/embodied/motion/limits.py)、[机械臂操作目录](../src/embodied/console/features/arm/commands.py)、[前端手势](../apps/console/src/features/teleoperation/runtime.ts)、[负载标定](../src/legacy/arx_tools/calibrate_payload.py)。
- reBotArm：[末端控制器](../src/embodied/robots/rebot/controllers/rebotarm_endpose_controller.py)、[轨迹采样](../src/embodied/robots/rebot/trajectory/sampler.py)、[CLIK](../src/embodied/robots/rebot/trajectory/clik_tracker.py)、[停稳保持示例](../examples/rebot/10_gravity_compensation_lock.py)。
- robot-teleop：[ARKit 采样](../apps/ios/RobotTeleop/Tracking/ARKitPoseProvider.swift)、[协议适配](../src/embodied/phone/inputs/arkit.py)、[相对映射](../src/embodied/phone/policies/relative_pose.py)、[协议约定](../docs/imported/robot-teleop/docs/protocol.md)、[手机 profile](../configs/phone/profiles/arkit_arx5.yaml)。

设计阶段完成源码审阅和上述 TRAPEZOID 标量验算；后续已完成基础插件链路的仿真验收，具体证据见实施记录。Swift/真实设备尚未验收；浏览器手势、MoveC/混合路径及负载实验仍属扩展目录。兄弟原项目未被更改。
