# 项目 X5 遥操控制器

当前 `teleop-presets-x5` 使用本项目维护的 X5 控制器。A/B 采集按键与 Home和录制会话在 `src/legacy/quest_teleop/`；设备通信、待机反馈与命令提交在 `src/legacy/robot_control/service.py`；默认运行不导入上游 R5 控制器或 `arx_r5_python`。

`target_limits.CartesianTargetLimiter` 在手柄坐标转换后、IK 前分别限制平移与旋转目标变化率，参数由 `启动遥操.sh` 的 `MAX_EEF_LINEAR_SPEED_M_S` 和 `MAX_EEF_ANGULAR_SPEED_RAD_S` 提供；0 表示不附加该项限制。旋转沿最短四元数路径推进，重新握持时清除旧目标差，循环卡顿不累积速度额度。预抓取和 Home 提交终点及独立速度／加速度／jerk 配置，由 arm-service 的 Ruckig 统一规划，仍遵守服务的关节速度、超前量与限位设置。参数语义和默认值见 [安装与启动](setup.md#遥操速度与响应参数)。

```text
run.sh teleop-presets-x5 → legacy.quest_teleop.cli.presets_main
  → legacy.quest_teleop.presets.PresetSingleX5Controller
  → legacy.quest_teleop.controller.X5TeleopController
  → legacy.quest_teleop.base_controller.X5TeleopBase
  → XRoboToolkit 通用 HardwareTeleopController / BaseTeleopController

控制端 arm 插件：service → ArmClient → 独立 arm-service
服务端 arm 插件：stanford → OfficialX5HardwareAdapter；simulated → SimulatedX5
input 插件：quest → XrClient；idle → 离线未激活输入
服务端 LocalPublisher → 持续状态发布
独立 view-arm 进程：订阅状态 → Three.js Scene
```

`base_controller.py` 维护六关节到 Placo 的映射、反馈更新、Grip 激活时的关节目标发送、Trigger 夹爪目标和日志接口。项目自行创建选中的输入/后端/状态发布器，复用上游 IK 方法；不调用会硬编码创建 XrClient 的上游构造函数。IK 与命令提交使用同一状态锁，主线程检查服务反馈，场景在独立监看进程刷新。组件初始化失败或运行退出时释放已创建资源，不包含上游 R5 的自动回零代码。

`controller.py` 维护 X5 服务接入、采集会话、指令来源记录和退出时的阻尼清理。`presets.py` 维护 A 开始／保存采集、B 预抓取／长按回 Home、右摇杆左开始／右暂停／上作废上一条／按下长按结束清零、Grip 取消预设并重建跟随参考。关节限速、可配置的指令超前限制、SDK preview、夹爪限力保持由共享后端处理；遥操脚本可覆盖当前会话的运动参数，硬件速度上限保留。

参数解析在 `src/legacy/quest_teleop/cli.py`，启动器直接运行该模块。Python 调用方直接使用 `X5TeleopController`，旧文件转发和类名别名已移除。无预设基线的会话元数据类名为 `X5TeleopController`，主入口记录 `PresetSingleX5Controller`，控制参数和数据 schema 不变。

`control-arm --input quest` 组合主 presets 控制器与虚拟后端，真机继续用现有 presets 入口。控制端只连接服务，不创建设备后端、状态发布器或三维场景；`scripts/viewer/monitor.sh` / `view-arm` 独立订阅与渲染，控制端退出或重启不会关闭监看。旧 `simulate` 命令已移除。历史 `teleop` / `teleop-dry-run` 保留独立实现，官方 X5 实验入口选用 `arx2025` 插件。官方 SDK 的真机位置执行禁用仍有效。主采集入口继续用 `--enable-session` 共享单 D435i；相机按配置中的 `driver` 加载，历史 `--enable-camera` 保留其已有采集实现。

默认依赖获取/检查仅要求 XRoboToolkit 与 PC-Service-Pybind 源码；`uv sync --locked` 安装主环境并构建 XR 服务绑定，移动仓库后使用 `./run.sh rebuild-xr-sdk` 修复绑定。启动器不再添加 R5 的动态库目录。两份 R5 源码都保留为 `required=false` 的参考，只有 `fetch-dependencies --all` / `check-dependencies --all` 才处理它们；新的 `.venv` 不安装 R5。

配置中沿用的 `/dev/arx_r5` 是 USB-CAN 适配器的 udev 别名，与 `/dev/arxcan0` 指向同一个设备；它不是 R5 SDK 依赖，本次未更换 CAN 接口或设备绑定。

## 来源与许可

复用逻辑来自 [XRoboToolkit-Teleop-Sample-Python](https://github.com/XR-Robotics/XRoboToolkit-Teleop-Sample-Python)，固定 commit `79e5cb8a56e3455515ce1b476e993c764ec58739` 的 `xrobotoolkit_teleop/hardware/arx_r5_teleop_controller.py`。迁入关节映射、反馈/命令与兼容日志方法，重写 X5 初始化配置；不迁入 R5 SDK 导入、R5 模型、示例 CAN/相机配置或自动回零行为。

Copyright (c) 2025 XR Robotics，原 MIT 许可保存在 [LICENSE.XRoboToolkit](../../src/legacy/quest_teleop/LICENSE.XRoboToolkit)。上游源码和锁定版本保持原样；项目对迁入代码的修改在主仓库维护。

## 验证边界

离线检查覆盖 R5 导入被阻止时的 X5 命令帮助、真实 Placo/Pinocchio 加虚拟反馈、A/长按 B/Grip 和 Trigger、录制来源与退出阻尼。新增插件替换、仿真拒绝硬件/真实录制、输入和显示故障清理，以及共享场景服务退出验证。进程加载库列表用于确认没有间接加载 R5 动态库；依赖检查使用可选仓库缺失的临时目录验证。

本次不执行真机运动。现场遥操与自动标定验收仍使用既有步骤。

主入口硬件参数已集中到 `[arm_service]`，不再转发 `--backend`、`--telemetry`、CAN 或 SDK preview。手眼标定共享该服务，控制退出只释放控制权，设备与状态发布继续运行。详见 [机械臂服务与监看](monitoring.md)。

## 实测方向标定与夹指尖端 TCP

日常脚本使用 `ROTATION_FRAME="base"`，无需四步方向标定，使用脚本配置的固定方向映射（当前 yaw 为 0°，与 8adcaf4 的默认方向配置一致）。每次按下 Grip 记录当前位姿起点，网页可直接查看姿态。

以下为可选的 `calibrated` 模式说明，日常启动不启用：启动后先保持待机，在 `遥操>` 输入 `c`，或另一个终端运行 `./run.sh quest-calibrate`。机械臂不参与标定运动，只采样右手柄沿基座 +X／+Y 两个已知方向的位移。+X 是机械臂零位时夹爪伸出的方向，+Y 是沿 +X 看过去的左侧，+Z 向上；这些方向不随操作者站位改变。

按终端提示分别记录起点和移动后的位置，每次水平移动约 15–20 cm，停稳、松开手柄按键后按回车。程序检查移动距离、水平性、正交性与左右手系，用两次位移和竖直方向拟合 Quest → 基座的旋转矩阵 K。标定不读取手柄姿态来猜测前后轴，不要求夹爪朝下或手柄与夹爪朝向一致。拟合误差反映两次方向的一致性，并不是对基座方向绝对准确性的保证；操作者必须按正确的基座轴移动。

标定期间需保持控制待机且没有正在录制的 Episode。未完成标定不能启用控制。成功后右摇杆下拨 1 秒启用，握住 Grip 建立当前相对起点即可。Grip 松握、待机切换和预设动作保留 K；重启遥操或头显暂停／恢复会清除 K，需要重新输入 c。Quest 主动重置追踪空间后也需返回待机重新标定。测量和结果写入 `outputs/logs/frame-calibration-*.json`，不会在新追踪会话中自动加载旧矩阵。

同一个 K 用于平移与旋转：`p_tcp = p0 + scale_factor K (p_hand - p_hand0)`，`R_tcp = K (R_hand R_hand0⁻¹) K⁻¹ R_tcp0`。每次 Grip 只重建 p0、p_hand0、R_tcp0、R_hand0；保持当前姿态偏置，不强迫手柄和夹爪摆成相同姿态。`OPERATOR_YAW_DEG` 在 calibrated 模式中不叠加。手柄绕哪个空间轴转，夹爪就绕标定后对应的空间轴转，不直接将某个动作指定给 J6。

日常脚本已恢复旧版 pose 配置：`--control-link link6 --control-mode pose`，平移比例 1.0（手柄与机械臂目标位移 1:1）；遥操 SDK 预览时间为 0.01 s，Home / 预抓取保持 0.05 s。按后续速度要求，日常遥操不附加软件速度上限（`max_joint_speed_rad_s=0`），预抓取上限为 2 rad/s，目标超前量限制关闭；Home 前半段上限 2 rad/s、后半段 0.75 rad/s。硬件速度边界、加速度及 jerk 平滑约束继续生效。旋转以腕部为控制点，指尖会绕腕部运动；回放、快捷键及服务层轨迹规划保留，因此不是整个历史版本的回滚。桌面检查仍以指尖 `eef_link` 为准，客户端也按目标姿态换算指尖高度。

需要指尖控制时可显式选择 `--control-link eef_link`。该控制点为 URDF 的 `eef_link`，即模型中两根夹指尖端之间的中心点；`link6` 到 `eef_link` 的固定偏移为工具 +X 方向 0.145 m。IK 同时约束该 TCP 的位置与姿态：手柄位置数据不变、仅旋转时，TCP 目标位置固定，手腕位置自动补偿；手柄 XYZ 变化时，TCP 位置按比例变化。若拧手腕同时改变了手柄追踪点的位置，TCP 仍会跟随这部分真实的 XYZ 输入。

145 mm 是当前 URDF 的标称值，尚未实测夹指 TCP。若夹指长度、安装工具或尖端中心不同，需要实测后修改 `gripper_fixed_joint` 的固定偏移；仅做方向标定不能修正 TCP 几何误差。离线 IK 验证了模型尖端位置保持与手腕补偿，实际漂移还受 TCP 几何、运动学模型和伺服误差影响，需真机复测。

状态采样保存 `position_alignment` 和 `rotation_alignment`（均为实测 K，未标定为 null）；控制元数据记录 `rotation_frame=calibrated`、`position_frame=measured_quest_to_base`，机器人元数据记录 `eef_frame=eef_link`。`quest_to_base_rotation` 仍是旧固定朝向配置，在该模式中不应用。日常使用 `base` 固定方向映射；`tool`、`aligned` 模式保留作对照。

更新后在旧终端输入 q，重新运行 `./启动遥操.sh`，然后输入 c。无需重启机械臂服务。非交互调用可在另一个终端运行标定命令。

网页场景右下角的“姿态对齐对照”是只读窗口，可点击标题收起。
青色示意手柄（前端箭头与上方短柱），橙色示意当前 Cartesian IK 目标，白色示意机械臂反馈的末端朝向。
三个模型的位置人为分开，使用同一个基座坐标系与同一视角；拖动窗口内模型可一起改变观察视角。
手柄朝向使用当前模式的方向映射矩阵转换，Quest 局部 -Z 为前、+Y 为上；夹爪示意使用工具 +X 为前、+Z 为上。
按下 Grip 后，目标包含当时记录的手柄与夹爪姿态偏置，因此青色与橙色不要求绝对重合。
松开 Grip 或待机时隐藏橙色目标，仍可查看手柄与实际朝向；未标定、暂停或输入过期时不显示手柄模型。
“目标与实际角度差”是两个姿态之间的最短旋转角，不是 J6 角度，也不是方向标定误差；两路数据独立采样，适合观察对齐，不用于精确测量延迟。
更新后需重启遥操进程并刷新网页。日常 base 模式无需输入 `c`。

当前日常脚本的跟随与预抓取速度参数均为 0，即采用各关节硬件速度上限；跟随加速度 6 rad/s²、jerk 30 rad/s³，预抓取加速度 6 rad/s²、jerk 24 rad/s³。
Home 使用 2 rad/s²、16 rad/s³；起始关节位置与 Home 的中点作为分段点，前半段速度上限 2 rad/s，中点停稳后后半段上限 0.75 rad/s。`home_final_speed_rad_s=0` 可使用原单段模式。实际峰值速度受行程、加速度及硬件执行影响。
网页“控制误差”标题显示 `IK 实测/配置 Hz · 服务 实测/配置 Hz`，实测来自近一秒执行次数和单调时钟，配置默认均为 50 Hz；待机无 IK 更新或数据过期显示 `—`。
此频率不是电机内部伺服频率，也不是网页刷新率。服务频率采样需要重启机械臂服务，遥操频率采样需要重启遥操；参数覆盖在新遥操会话连接时生效。

桌面安全配置集中在 `configs/project.toml` 的 `[arm_safety]`，由独立机械臂服务读取和执行，控制客户端不能通过运动参数覆盖。遥操从服务读取同一设置做目标截断。启动脚本仅保留运动调速参数。
桌面与基座底面齐平时 `table_z_m=0.0`；按小物体抓取需求，`table_clearance_m=0.0`，允许夹指尖端中心到达桌面高度。
遥操将低于该平面的 Cartesian 目标 Z 截到下限，XY 与姿态目标保持原有映射。
独立机械臂服务在接收新路径时，以 `eef_link` 检查 Ruckig 的预测路径（每 10 ms 采样，包括切换预设前的制动段），涵盖遥操、Home、预抓取；越界新目标不被采用，继续之前通过检查的路径，网页显示拦截提示。初始模型 TCP 已低于下限时拒绝取得控制权。
这是模型 TCP 的软件限制，不是整夹爪碰撞检测或实际接触检测。零余量时，模型尺寸误差、反馈误差、两次采样之间及实际伺服超调仍可能造成接触；夹爪倾斜时单根夹指也可能低于中心点。
更新后退出遥操并重启机械臂服务，再启动遥操、刷新网页。旧服务没有统一安全配置，遥操会提示重启服务，不能仅刷新网页。
预抓取速度上限为 2 rad/s，加速度 6 rad/s²、jerk 24 rad/s³；各关节硬件限速仍生效。

左手柄 X：网页回放上一条完整保存且未丢弃的数据；不发送真机命令，待机也可使用。
左手柄 Y：真机回放上一条的实际关节位置与夹爪开度。需先启用控制、保存当前采集、松开右 Grip/扳机，并结束预设运动；先按预抓取限速定位到录制起点，再按记录时间提交目标，最后等待末点反馈到位。仍受当前服务的速度、加速度、jerk 和桌面路径检查约束，实际动作可能滞后于记录时间，不保证精确复现原始轨迹时序。
再次按 Y、握住 Grip、B 操作或切换待机取消真机回放；输入掉线仍由已有控制租约保护。回放本身不会开始采集或覆盖原始数据。
网页“数据回放”模块可选已保存的数据、播放、暂停、拖动时间轴，同屏显示原始 RGB 图片序列、录制机械臂姿态、六关节角和夹爪开度。视频与状态使用记录的主机时间对应；网页控件只操作网页回放，不启动或暂停真机回放。
上一条优先取本次会话最后保存的 Episode；重启后取数据目录中最近的完整、未丢弃记录。未完成或模型不匹配的记录拒绝回放。
更新后需重启遥操与网页监看进程，并刷新网页以加载回放模块。

基线 `ca25e19` 保留 50 ms / 6 rad/s² / 30 rad/s³。响应优化分支的日常遥操改为 10 ms / 12 rad/s² / 120 rad/s³，减少 SDK 插值延迟并放宽加速度与 jerk 约束；坐标映射和 Grip 对齐逻辑保持基线实现。Home 和预抓取使用上文的独立分段／速度参数。此组参数需实机验证；离线正弦目标跟踪测试用于比较规划器延迟，不代表真机精度。保留 Ruckig 和桌面路径检查。配置切换按规划器实际 profile 决定 SDK 预览时间，切换中的制动段沿用原 profile。真机回放的播放阶段也使用 teleop profile，前往起点使用 preset profile。

### 重启后的快速方向对齐

待机时输入 `c`，把右手柄前端大致水平指向机械臂正前方（零位时夹爪伸出的方向），松开按键，按回车。采样约 0.6 秒，不移动机械臂，无需做位移标定。对齐同时应用于 XYZ 和旋转；Grip 重新建立相对位置和姿态起点。

仅重启电脑端遥操，且 Quest 未重启、未重新居中或改变追踪坐标系时，可在待机输入 `cr` 显式复用上次方向。Quest 坐标改变后请用 `c`；保存文件不会自动加载。此功能用于日常 `base` 模式。
