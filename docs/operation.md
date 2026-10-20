# 日常操作（真机工作台与仿真）

中文 [一屏架构图](architecture/index.html)：在浏览器打开即可总览控制源、arx 核心、机械臂、相机和数据流，支持点击部件、全屏与高清海报导出。插件与命令、目录和整理建议保留在 [详细导览](architecture/explorer.html)；离线使用与更新方法见 [导览说明](architecture/README.md)。

`arx5_test`、`reBotArm_control_py`、`robot-teleop` 已集中迁入本仓库，三个原位置无需保留。算法、设备、标定、相机、数据与操作入口见 [合并后的组件说明](consolidated-components.md)；统一入口为 `./run.sh components list`。默认 Quest 遥操继续使用 arx 原生链路。完整原工作树保存在本仓库 `archives/migration/originals/`，仅作恢复归档。

Quest Pro 遥操 ARX X5-2025。主入口 `teleop-presets-x5`：右 Grip 跟随、Trigger 控制夹爪、任务开启后 A 预抓取并开始采集、B 保存／长按回 Home、右摇杆左开始／右暂停／上作废上一条／按下长按结束清零；Grip 可取消预设动作。

已整理目录、依赖和启动配置，支持独立运动学仿真、单腕部 D435i 手眼标定，以及主遥操进程内的原始 episode 采集。命令行/脚本为主，网页只读监看。本机 π0.5 与 LeRobot 导出尚未接入；H800 只负责训练。

## arx 控制架构迁移（新增入口）

完整 arx-televr 已导入 `control/`，原 Quest 输入、原生路由、IK 与 250 Hz 设备循环优先复用。机械臂、VR、控制源、映射、规划、相机、标定适配器和导出器通过注册表装配；所有代码都在本仓库内。控制使用独立 Python 3.10 环境，工作台与手机协议使用 Python 3.12。

```bash
./run.sh control verify
./run.sh workbench plugins
./run.sh workbench validate --profile sim-manual
./run.sh workbench start --profile sim-manual --headless
# 手机 + 合成相机组合：
./run.sh workbench start --profile sim-phone --headless --record-task "手机检查"
```

启动日志给出 `control/run/<目录>`。另一个终端用 `motion --run <目录>` 做预览、MoveJ/MoveL、点动、夹爪、示教和自由拖动；`phone --run <目录>` 接手机，`handeye --runtime <目录>` 使用 arx 自动标定适配。采集通过只读流独立运行，新 schema 2 已能导出 LeRobot v2.1，保持反馈与命令独立时间及原始 sidecar。

手机、多源控制、标定适配和新数据导出的基本链路已通过离线/仿真验证。真实 Quest、iPhone、RealSense 和 CAN 现场验收尚未进行；旧采集按键、任务页面、回放和策略执行仍保留原入口。**arx 真机启动会使能并回 Home，退出请求 SDK 阻尼。** 使用说明见 [插件工作台](plugin-workbench.md)，证据与当前边界见 [实施记录](arx-runtime-implementation.md)，设计依据见 [架构方案](arx-runtime-redesign.md)。

## 日常启动

环境由 uv 管理，首次安装执行下方的“安装与验证”。日常脚本直接使用项目 `.venv`，无需激活环境，也不会在启动机械臂时自动安装或更新依赖。脚本按自身位置解析路径，可以从任意工作目录调用。

推荐在一个终端运行 `bash 启动工作台.sh`：默认进入 Rich 总控页，在一页管理机械臂、遥操和网页；tmux 同时保留 `1 机械臂 / 2 遥操 / 3 网页 / 4 总控` 详情窗口。需要 sudo 密码时在窗口 1 输入；USB-CAN 未连接时保持等待，机械臂初始化尚无电机反馈时每 5 秒重试。遥操等待机械臂反馈、Quest 开机和 USB 调试授权就绪后启动，不设开机等待时限；网页同时启动共享相机预览。启动机械臂仍会使能电机并进入阻尼，不会自动回零。已有机械臂或遥操服务会复用，其控制台和退出操作仍在原终端。权限、配置及其他初始化故障仍提示错误；运行中断线不自动恢复控制。等待通电也可通过工作台 `stop` 退出。

- 总控直接按键：`A` 机械臂启动 / 重连，`T` 遥操启动 / 恢复，`S` 停止遥操，`W / E` 开 / 关网页，`X` 有序停止本工作台的遥操和机械臂（保留总控、网页和相机），`Q` 离开并保持运行；`Z` 关闭遥操、机械臂、网页、相机服务并退出整个工作台；`R` 先完整关闭上述服务，再重新启动。
- 总控还可按 `H / P` 回零 / 预抓取，空格停止并保持，`D` 切入阻尼（先支撑机械臂）。这些操作交给原机械臂控制台执行，状态与拒绝原因可按 `1` 查看。
- 总控按 `1 / 2 / 3` 查看对应详情，`Ctrl+B` 再按 `4` 返回总控，也可点击底部窗口名。
- `Ctrl+B` 再按 `D` 离开，服务继续运行；再次执行入口即可返回，不会重复启动。
- `bash 启动工作台.sh status` 查看窗口状态；设备在线状态以各控制台和网页为准。
- `bash 启动工作台.sh stop` 先等待遥操收尾，再退出本工作台启动的机械臂控制台；超时保留现场，不强制终止。网页、相机及复用的外部服务继续运行。
- 完整关闭 / 重启会等待录制与动作收尾；任何步骤未退出都会停止后续流程并保留现场。外部控制服务不会被强制关闭。命令行也可执行 `bash 启动工作台.sh shutdown` 或 `bash 启动工作台.sh restart-all`。这些操作只管理项目服务，不关闭主机或共享的 ADB / XR 系统服务。
- 机械臂服务退出后，在总控按 **A** 重连；详情窗口仍支持 **R**。旧版崩溃控制台可由总控替换，无需手动退出窗口；运行中的服务不会被重启。
- 已退出窗口保留日志；修复问题后可用 `bash 启动工作台.sh restart arm`（或 `teleop`、`web`）重新启动该窗口，运行中的窗口拒绝重启。

已有三窗口工作台再次执行入口会补上总控页，不重启现有服务。需要 tmux（`sudo apt install tmux`）；Oh my tmux 主题可选。原来的独立启动方式仍可使用：

仿真和真机共用 <http://127.0.0.1:18081/>：总控 `M` 切到 MuJoCo、`F` 切回真机。网页、相机以及已经启动的仿真运行实例保留；切出仿真解除输入控制权并保持姿态，不回零、不重建模型。切出真机先退出本工作台遥操并等待停稳，保留机械臂服务的位置保持；外部遥操或未释放的控制权会阻止切换。切回后仍需重新启用，不沿用切换前握住的 Grip。

运行 `bash 启动工作台.sh --mode simulation` 可直接进入仿真；`启动网页仿真.sh` 也是这个统一入口。仿真使用 Quest 输入、arx-televr 原版相对跟随与 IK/增益、MuJoCo 250 Hz 动力学。右手柄完全按 arx-televr 上游：A 按住至少 150 ms 后松开切换模式（首次进入跟随，之后在 FreeDrive / 跟随间切换），A+B 回 Home，Grip 跟随、松开保持，Trigger 控制夹爪。允许遥操或恢复跟踪后，需先松开全部按键、Grip 和 Trigger 才会接收输入。`T / U` 允许仿真遥操，`S / 空格 / O` 暂停保持，`H / P` 仿真回零 / 预抓取。设备未跟踪时等待，不运行自动演示。网页只显示相机、输入和所选机械臂反馈；1920×1080 下相机列按视频比例确定宽度，剩余空间给 3D。

仿真采集全部使用左手柄：左摇杆左推开始任务，X 前往预抓取位并开始记录，再按 X 保存并回预抓取位；Y 短按预抓取、长按 1 秒作废当前录制（不回零，回 Home 用右手 A+B）；左摇杆右推保存并暂停任务，上推撤销上一条，按下长按 1 秒保存并结束任务。左 Grip（或右 Grip 接管）取消仿真预设动作。原始 250 Hz 反馈/指令保存为 schema 2 MCAP，位于数据目录的 `mujoco/`，明确记录 backend=sim。此处是仿真状态记录，实物相机/IMU 仅作参考，不将实物视频配成仿真训练样本。真机入口及其双相机视频采集仍使用现有服务；本阶段先完成原版控制核心的仿真试操，未更换真机控制核心。

`E` 关闭网页及其仿真运行实例；`Z` 完整关闭服务。首次创建 MuJoCo 与正常保存/停车仍需等待，日常模式切换不再等待仿真退出回零。旧运行中的遥操需要正常重启一次，才能使用新的退出后位置保持行为。

```bash
./run.sh --help
./run.sh --profile presets teleop-presets-x5 --help

# USB 反向端口与 CAN 准备，不启动遥操。
./run.sh prepare-connections
./run.sh start-xr-service

./run.sh check-xr
./run.sh check-camera
./run.sh doctor

# 终端 1：机械臂服务。连接真机并进入阻尼待机，持续发布反馈。
./启动机械臂服务.sh
# 启动后显示 Rich 全屏控制台，底部常驻快捷键栏，无需回车：H 回零、P 预抓取、S/空格 停止、I 状态。
# D 切入阻尼（不锁定位置，先支撑机械臂）；S/空格 恢复当前位置保持。动作期间先取消并等待停稳，再按 D。
# B 开启头显发现广播，N 关闭；原生 XR 模块默认关闭广播，切换不重连 USB。见 docs/quest-discovery.md。
# Ctrl+C 取消动作并保留服务；Q 或 Ctrl+D 关闭服务；↑↓ 查看日志，End 返回最新。
# 完整输出保存至配置的 log_dir 下 arm-console-*.log。
# 不需要交互时：./启动机械臂服务.sh --no-interactive
# 如需虚拟机械臂，改用：./run.sh arm-service --backend simulated

# 终端 2：只读三维监看，可以先开并长期保留。
./scripts/viewer/启动仿真.sh

# 终端 3：需要控制时再启动。
./启动遥操.sh                    # 自动唤醒 Quest，再开始遥操，显式 --actuate
# 虚拟服务使用：./run.sh control-arm 或 ./run.sh control-arm --input quest

# 其他终端：只读取服务反馈，不改变控制模式。
./run.sh check-arm
# 虚拟反馈：./run.sh check-arm --virtual

# 采集时以此替换普通真机控制命令：
./启动遥操.sh --enable-session
```

机械臂服务、控制和监看独立运行。没有控制程序时，服务持续反馈，页面显示阻尼待机；控制端退出或失联后，服务清除控制权并请求阻尼，画面继续更新。关闭监看不影响控制；停止服务才会使反馈过期。切换控制方式不会重置机械臂姿态。

**真机服务启动会使能电机并进入阻尼，不是被动 CAN 读取；阻尼不保证位置保持。** 服务不会自动回零。`scripts/viewer/启动仿真.sh` 只启动监看，不启动 SDK；真机服务和虚拟服务由独立命令选择。

`configs/project.toml` 的 `[arm_service]` 集中管理接口、SDK preview、速度/软限位和夹爪参数，`[telemetry]` 管理状态流。presets / collection profile 保留输入与采集设置。主控制端不再提供 `--backend`、`--telemetry` 或 CAN/SDK 配置转发。详见 [机械臂服务与监看](monitoring.md) 和 [插件说明](plugins.md)。

默认 `./run.sh` 选择 `teleop-presets-x5`，连接已启动的真机服务，默认不启用位置控制。`--profile presets` 才加载日常快捷脚本参数。

服务参数优先级：项目默认配置、本机覆盖、命令行。控制参数优先级：控制器默认值、profile、命令行；控制端显式传入的速度覆盖服务默认速度。启动器参数放在命令之前，具体命令参数放在命令之后。

日常比例和速度可直接编辑 `启动遥操.sh` 顶部的 `SCALE_FACTOR=1.0` 和 `MAX_JOINT_SPEED_RAD_S=2.0`。它们覆盖 presets profile，脚本后附加的命令行参数优先级更高。控制端传入的速度优先，未传时才使用 `[arm_service]` 默认速度。修改脚本后重启遥操即可；首次使用这一优先级规则时需先重启服务加载新代码。

```bash
# 仅显示最终命令，不加载硬件 SDK、不执行。
./run.sh --print-command --profile presets teleop-presets-x5 --scale-factor 0.1
# 真机快捷脚本也可覆盖为只读。
./启动遥操.sh --no-actuate
```

## 配置与目录

```text
src/embodied/core/          共享模型、坐标变换、相机、设备锁与存储
src/legacy/robot_control/       机械臂服务/客户端、平滑轨迹、X5 后端与夹爪控制
src/embodied/calibration/wrist/ 手眼标定采样、计划、求解与验证
src/embodied/monitor/    独立运动学仿真、共享 Three.js 场景、FK/IK 与轨迹预览
src/legacy/quest_teleop/        X5 遥操控制器、A/B 采集按键与 Home、采集会话与只读监看
scripts/                 启动器、Quest 工具、CAN/依赖安装、设备检查
configs/project.toml     共享默认配置
configs/local.toml       本机设备和存储路径，不提交
configs/profiles/        绑定特定命令的参数组合
configs/plugins.toml     可替换组件的名称、工厂与能力声明
assets/robots/x5/        当前 X5-2025 hybrid URDF、网格和来源校验
third_party/             外部依赖与可选参考源码，版本由 dependencies.lock.toml 固定
pyproject.toml / uv.lock Python 依赖声明与版本锁
.python-version / .venv/ Python 版本与本机 uv 环境（.venv 不提交）
environments/snapshots/  历史环境盘点
docs/validation/         既有 TXT 诊断记录
outputs/                 日志、构建产物、标定原图/会话/结果，不提交
tests/                   不连接真机的回归检查
```

根目录提供 `启动工作台.sh` 统一入口，并保留 `run.sh`、`启动机械臂服务.sh`、`启动遥操.sh`、`网页控制.sh` 独立入口。其他快捷脚本按用途放入 `scripts/inference/`、`scripts/arm/`、`scripts/datasets/` 和 `scripts/viewer/`，详见 [脚本索引](../scripts/README.md)。连接准备、电量查询、Quest 唤醒分别使用 `./run.sh prepare-connections`、`./run.sh quest-battery`、`./run.sh quest-wake`。环境清单和配置留在根目录的专用文件夹；工具与实现归入 `scripts/` 和 `src/`，只维护当前入口和模块路径。具体见 [目录职责](layout.md) 与 [源码分层](../src/README.md)。

主控制器、预设动作和 X5 反馈/命令桥接由本项目维护；XRoboToolkit 提供通用 Quest/IK/线程层。默认获取、构建与启动均不需要 R5 SDK，见 [X5 控制器说明](x5-controller.md)。

大型文件放在仓库外，路径可由本机配置覆盖：

```text
~/work/models/pi05_base/            现有 LeRobot 格式权重
~/work/data/quest_x5/                共享采集数据集目录
~/work/installers/QuestArmTeleop/   APK/DEB 安装包
```

项目运行、构建和测试不依赖桌面备份。桌面备份保持原位、原样。

## 安装与验证

首次安装见 [安装说明](setup.md)。在项目根目录执行：

```bash
./run.sh fetch-dependencies
uv sync --locked
./run.sh test
```

`uv sync --locked` 会安装锁定的 Python 包并编译 Quest 原生绑定。移动仓库后使用 `./run.sh rebuild-xr-sdk` 清理旧 CMake 路径并重建绑定。主环境不安装 R5、Torch/JAX 或上游其他机器人依赖；本机 π0.5 推理后续使用独立环境。

测试覆盖真实 Placo/Pinocchio 模型、虚拟反馈、模拟 XR、合成标定数据及异常清理，禁止真实 CAN 实例化；不替代真机动作验收。

更多入口见 [命令说明](commands.md)，后续采集和策略边界见 [重构记录](refactor.md)。

独立仿真、手动/自动手眼标定的命令与边界见 [仿真与标定](simulation-calibration.md)。桌面实现的检查结果见 [迁移审查](migration-review.md)。

数据采集、虚拟演练、回放、校验与只读网页见 [采集操作说明](recording.md)。无需设备即可先运行 `./run.sh session demo`，再从另一终端运行 `./run.sh monitor --port 0`。


三维监看页已合并 D435i 腕部 RGB 预览与左右 Quest 手柄数据，保持单页布局。日常 `./启动遥操.sh` 默认开启采集会话（不自动录制），`./scripts/viewer/启动仿真.sh` 打开统一监看。详细数据来源及断流行为见 [监看说明](monitoring.md)。


日常监看固定在 http://127.0.0.1:18081/，重启监看后刷新即可。遥操交互终端新增 `u / 3` 取消模拟佩戴：保留进程、暂停机械臂控制，由 Quest 自行息屏；`w / 1` 恢复模拟佩戴后自动等待新数据，松开 Grip/A/B/右摇杆按键后即可正常操作。


`./scripts/viewer/启动仿真.sh` 现在同时确保独立相机服务运行，不必先启动遥操：模型立即显示，D435i 初始化后显示 RGB；真实关节反馈由机械臂服务提供。未连接机械臂服务时显示“参考姿态”，不会把模型默认姿态当作真实反馈。相机服务由监看与采集共享，关闭页面或遥操后继续运行。


`./启动遥操.sh` 默认配置并确认 USB 数据连接，使用 Rich 显示当前连接方式；未确认 USB 时不启动控制、不自动回退 Wi-Fi。首次需将 Quest 客户端服务器地址设为 `127.0.0.1` 并连接。交互终端 `p` 可刷新连接状态。
