# 环境与设备安装

主环境使用 uv、Python 3.12 和项目内 `.venv`，当前支持 Linux x86_64。`run.sh` 使用 `/usr/bin/python3`（3.11+）解析配置，然后直接启动 `.venv/bin/python`，不受终端中 Conda 激活状态影响。启动不自动安装或升级依赖。

独立 `view-arm` / `scripts/viewer/monitor.sh` 通过 Python 标准库启动 Three.js 本地服务，独立机械臂服务通过锁定的 pyzmq 发布状态。前端模块和材质随仓库保存，无需安装 Node/npm，也不依赖 CDN；浏览器需支持 WebGL 2。场景资产来源与可选浏览器测试见 [共享场景说明](../../src/embodied/applications/monitor/web/README.md)。

## 首次安装

系统需有 Git、C++ 编译器、`adb`、`can-utils`、`iproute2`、libusb/libudev 运行库，以及匹配的 Quest PC Service。安装 [uv](https://docs.astral.sh/uv/getting-started/installation/)（当前项目使用 0.11.x）。系统 Python 用于首次获取源码，不需要先创建 `.venv`。在仓库根目录执行：

```bash
# 仅在首次配置时复制，保留已有相机序列号和本机路径。
cp -n configs/local.example.toml configs/local.toml
# 修改 cameras.wrist_rgb.serial；默认 Python 已指向项目 .venv。
./run.sh fetch-dependencies
./run.sh check-dependencies
uv sync --locked
uv pip check --python .venv/bin/python
./run.sh test
```

`pyproject.toml` 指定 uv 管理的 Python，`.python-version` 固定到 3.12.12；uv 可以自动下载缺失的解释器。Python 包由 `uv.lock` 固定，XR 原生绑定从锁定的本地源码构建，隔离构建依赖（CMake、pybind11、setuptools、wheel）在项目中明确指定。主 X5 SDK 使用 `arx5-interface==0.1.3` 二进制包。无需安装 Conda 或 R5。

## 日常同步与原生绑定修复

拉取项目更新后先检查源码版本，再同步环境。`--locked` 会在依赖声明与锁文件不一致时报错；只有有意调整依赖时才更新锁文件。

```bash
./run.sh check-dependencies
uv sync --locked
./run.sh test

# 仓库移动或 XR 原生源码变化后，重建带绝对路径的绑定：
./run.sh rebuild-xr-sdk
```

重建命令把旧 CMake 构建目录移入 `outputs/build-cache/`，然后通过 uv 禁用缓存并重建 `xrobotoolkit-sdk`。只作用于本项目 `.venv`。更换机器时重新执行首次安装，不复制 `.venv`。不要把本机推理框架安装到遥操环境，环境边界见 [环境说明](../../configs/environments/README.md)。

源码获取只处理缺失目录；已有 commit 不匹配会报错，不会 reset 本地改动。默认获取/检查只需 XRoboToolkit 和 PC-Service-Pybind 两个源码仓库；X5 SDK 使用已安装的 `arx5-interface`。`--all` 额外处理可选官方 X5、模型源码、openpi，以及官方 R5 和 XR 内嵌 R5 fork 参考仓库。两份 R5 均不参与项目 X5 的构建和运行。

迁移前的 Conda 清单归入 `configs/environments/snapshots/conda-teleop/`；`configs/environments/snapshots/teleop-installed.txt` 保留全部包盘点，其中的 `arx_r5_python` 仅是历史记录。原 Conda 环境没有被修改。

2026-09-08 在本机用全新 `.venv` 完成迁移验证：锁文件校验、48 个已安装包的依赖检查、XR 原生绑定构建及重建、62 项离线测试、独立仿真 CLI，以及 D435i 640×480 RGB 连续取帧通过。清除 Conda 环境变量后测试通过，核心模块加载的动态库不含 Conda 或桌面备份路径。没有执行真机运动，也未验证全新操作系统上的安装。测试日志在 `outputs/logs/uv-migration-tests.log`。

## 官方 X5 实验后端

```bash
./run.sh fetch-dependencies --all
uv sync --locked --group sdk-build
./run.sh build-x5-2025-sdk
./run.sh teleop-x5-2025 --help
```

构建目录为 `outputs/build/x5-2025/`；如保留了旧 Conda 的 CMake 缓存，先将该目录移走再重新构建。该闭源 SDK 另外链接 `libkdl_parser.so`、`liborocos-kdl.so.1.5` 及相应依赖，uv 不提供这些系统 KDL/ROS 库，需要按厂商 SDK 的 ABI 配置；ROS 环境按其自身要求设置 `AMENT_PREFIX_PATH`。本机主 `.venv` 没有安装这套可选系统依赖，尚未验证该实验后端的运行。其 `--help` 延迟加载 SDK，无需这些库；主 Stanford X5 遥操、标定和仿真不依赖它。

## Quest 和 CAN

Quest 安装 XRoboToolkit APK；主机安装匹配的 PC Service DEB。安装包目录由 `paths.installer_dir` 记录。首次连接 USB 时，在头显内允许 ADB 调试并勾选“始终允许这台电脑”，此授权不能由电脑代替。

日常执行 `./run.sh` 即可，头显可以之后再开机。VR 窗口会等待 USB 授权和 Android 开机，自动准备主机接收服务、端口转发并打开客户端；拔插后继续检查，失败按 2–30 秒退避重试，按 `R` 立即重试。窗口分别显示各段状态，后台检查不阻塞总控。官方客户端首次连接仍需输入 `127.0.0.1` 并点击 Connect；要免输入、免点击和持续自动重连，需构建安装[自动连接版客户端](quest-client.md)。工作台复用已运行的客户端；安装新版本后先退出旧客户端，避免同时连接。

独立诊断与准备命令：

```bash
./run.sh quest usb status --json        # 只读状态，包括未授权、开机中与连接情况
./run.sh quest usb prepare --launch-client  # 复用接收服务、配置转发、打开客户端
```

Quest 的 XRoboToolkit server IP 设为 `127.0.0.1`。`./scripts/shortcuts/启动遥操.sh` 默认先配置 USB reverse，随后自动执行 `quest-wake start`，唤醒头显、发送模拟佩戴信号并打开 XRoboToolkit 应用，再启动遥操。也可单独使用 `./run.sh quest-wake start`，结束使用 `./run.sh quest-wake stop`。唤醒命令返回不代表 XR 数据已连接；控制端会等待新帧。

未实际佩戴时应先启用模拟佩戴，否则头显休眠会使 PC Service 报 `device missing`，手柄即使有电也会失去跟踪。可用 `./run.sh quest-battery` 查看电量，`adb shell dumpsys power` 查看 `mWakefulness` 和佩戴传感器状态。发生断连时先退出遥操，再唤醒并恢复 Quest 应用连接，最后重新启动遥操。

Quest 控制在输入时间戳开始推进后才发送指令；超过 0.5 秒没有新帧或时间戳回退时停止控制并释放服务控制权。静止手柄的新帧仍然有效，检测不依赖位姿是否变化；恢复连接后需重新启动控制程序，不自动续发断连前的 Grip/预设指令。

```bash
./run.sh can-status
./run.sh can-up --dry-run
./run.sh can-up
```

CAN 路径在 `robot.can_device` 配置。已有接口保持原状态；接口存在但未 UP 时，可明确执行 `sudo ip link set can0 up`。`setup-arm-device` 安装原有 USB-CAN 的 udev 规则，会修改系统配置；新设备应先调整 `scripts/setup/udev/99-arx-r5.rules` 中的序列号。

环境覆盖：`QUEST_TELEOP_CONFIG` 指定本机 TOML，`QUEST_TELEOP_PYTHON` 指定控制解释器，`QUEST_XR_SERVICE_SCRIPT` 指定 PC Service 脚本。

相机序列号使用 `./run.sh check-camera --serial ''` 枚举出的 librealsense 设备序列号。USB 视频节点的标识可能不同，不能直接代替驱动的设备选择值。此命令只枚举设备和采集档位，不启动视频流。

## 独立机械臂服务

日常先在独立终端执行 `./scripts/shortcuts/启动机械臂服务.sh`，再按需启动 `./scripts/viewer/monitor.sh` 和 `./scripts/shortcuts/启动遥操.sh`。仅调试虚拟机械臂时，服务改用 `./run.sh arm-service --backend simulated`，控制用 `./run.sh control-arm`。服务和监看可以在没有控制程序时保持运行。

`scripts/shortcuts/启动机械臂服务.sh` 会在进入交互控制台前自动准备 CAN 接口，必要时在当前终端提示输入 sudo 密码；已有接口保持原状，准备失败则停止启动。`--help` 和虚拟后端跳过准备，可用 `--no-prepare-can` 显式关闭。

真机服务初始化 SDK 时会使能并进入阻尼；监看和 `check-arm` 只读服务，不初始化 SDK。`check-arm --virtual` 读取虚拟服务。服务配置集中在 `[arm_service]`，RPC socket 为 `outputs/runtime/arm.sock`，状态流由 `[telemetry]` 指定。控制端关闭后服务继续反馈，失联控制权会在 0.5 秒后撤销。详细运行边界见 [机械臂服务与监看](monitoring.md)。


旧遥操快捷脚本默认使用 USB：启动脚本用 Rich 展示设备、63901 转发和实际 XR TCP 连接状态；唤醒后最多等待 15 秒确认 USB 连接，再启动机械臂控制端。若仍发现 Wi-Fi 连接或没有 USB 数据连接，会保持未启动/暂停状态，提示在 Quest 客户端设置 `127.0.0.1` 并 Connect，再按 `w` 重试。官方 APK 没有远程填写地址入口；新自动连接版的构建与安装见上文。不会自动回退 Wi-Fi。

`p` 会刷新连接方式提示，也可单独执行 `./run.sh quest-transport status`（只读）。`./run.sh setup-quest-usb` 仍可单独配置 USB 转发。Rich 依赖由 uv 管理；`--help` 不触发设备操作。本次实现没有重连正在运行的真机。

### 夹爪张开端点

`configs/project.toml` 的 `[arm_service].gripper_open_limit_m = 0.078` 限制张开指令，遥操满开目标同步使用此值。位置换算仍采用 `gripper_width = 0.082`、`gripper_open_readout = -3.4`，不改变反馈标尺或 SDK 力矩保护。

此上限根据当前夹爪空载全开时反馈约 0.0789 m、力矩约 -0.98 的受阻现象设置，留出约 0.9 mm 的反馈坐标余量；尚需真机确认无持续顶压，并不等于已完成物理开度标定。修改后需结束遥操并重启机械臂服务，再启动遥操；不要只重启网页。若仍有空载张开力矩警告，应继续检查端点／零点或机械阻力，不提高力矩阈值掩盖问题。

### 遥操速度与响应参数

在 `scripts/shortcuts/启动遥操.sh` 顶部设置当前控制会话的参数：

| 参数 | 脚本默认值 | 含义 |
| --- | --- | --- |
| `MAX_JOINT_SPEED_RAD_S` | 4.0 | 每关节速度上限；0 使用 SDK 配置的各关节速度上限，正值与硬件上限取小值 |
| `MAX_EEF_LINEAR_SPEED_M_S` | 0 | 遥操末端平移目标速度上限，m/s；0 不附加限制 |
| `MAX_EEF_ANGULAR_SPEED_RAD_S` | 0 | 遥操末端旋转目标角速度上限，rad/s；0 不附加限制 |
| `MAX_COMMAND_LEAD_RAD` | 0 | 指令相对反馈允许超前的弧度；0 关闭这层附加限制 |
| `PREVIEW_TIME_S` | 0.02 | SDK 航点的未来时间，单位秒；0 请求即时目标 |
| `JOINT_SOFT_LIMIT_MARGIN_RAD` | 0.05 | 软件限位余量；0 使用模型硬限位 |

两个 `MAX_EEF_*` 参数在遥操端独立限制送入 IK 的平移、旋转目标变化率。启动脚本默认不附加末端目标变化率限制，关节上限请求 4 rad/s（受硬件上限约束），超前量限制关闭，SDK 预览时间 0.02 s。平滑性由服务层在线关节轨迹提供。它们不是实际末端速度的保证，旋转时关节运动也可能带来平移。每周期朝最新手柄目标推进，不排队重放历史输入；松开 Grip 会触发服务层平滑停车、丢弃旧位置目标；重新握持会建立新的姿态参考，规划状态保持连续。预抓取和回零使用关节预设轨迹，不经过这两个末端限制，但同样受脚本中的关节速度、超前量和预览时间设置影响，预设还有独立的速度／加速度轨迹限制，实际速度受各层限制共同约束。这些值是低速验证起点，不代表已验证当前负载下的最佳跟随效果。

关节速度、超前量、预览时间和边界余量随控制权申请传入机械臂服务，覆盖当前会话的服务默认值；下一控制客户端重新申请时使用自己的参数或服务默认值，不继承上一位控制者的覆盖值。速度与硬件边界、SDK 力矩保护仍生效，不通过调大底层 SDK 限值来加速。SDK 配置速度不等于负载下保证能达到的速度。

首次更新代码需退出遥操并重启机械臂服务，再启动遥操；此后仅调整上述脚本参数只需重启遥操。硬件型号、夹爪开度映射和服务循环频率继续由服务配置管理。仿真只验证速度／限位和参数协议，不模拟真实 SDK 插值与伺服延迟。

会话记录请求参数及有效各关节速度，机械臂服务 `describe` 返回 `effective_motion_limits` 与 `effective_joint_speed_limits_rad_s`。SDK 航点使用显式时间戳，Episode 的 `submission.timestamp_argument` 记录实际传入值。

### 遥操启动待机

Quest 遥操启动后只连接并显示数据，不发送机械臂或夹爪命令。先松开右 Grip、Trigger、A/B 和摇杆按键，日常模式无需方向标定，让右摇杆回中并持续下拨 1 秒（持续保持只切换一次，回中后才能再次触发；中途松开、斜推或按下其他按键会取消计时）。终端显示“控制已启用”后再握住 Grip 操作。启用前 A/B、Home 和采集操作不生效；头显暂停／休眠后恢复同样需要重新下拨 1 秒。此门槛约束遥操客户端，不改变独立机械臂服务本身的运行模式。

控制启用后再次下拨 1 秒会返回待机，取消预抓取／Home 及待启动录制，释放机械臂控制权；Quest 数据连接保持，之后可再次下拨 1 秒启用。已有采集任务、计数和正在进行的录制不因切换自动结束，网页分别显示控制状态和采集状态。

### 服务层在线平滑轨迹

使用锁定版本 Ruckig 0.12.2，在 arm-service 的 50 Hz 周期统一生成关节位置、速度和加速度。重规划保留上一周期状态；目标速度、加速度为零，运动途中限制 jerk。原来客户端的 Home／预抓取五次多项式层已移除，二者直接提交终点和运动配置类型，由同一服务规划。

| 启动脚本参数 | 初值 | 单位 |
| --- | --- | --- |
| `MAX_JOINT_SPEED_RAD_S` | 4.0 | rad/s |
| `MAX_JOINT_ACCELERATION_RAD_S2` | 2.0 | rad/s² |
| `MAX_JOINT_JERK_RAD_S3` | 10.0 | rad/s³ |
| `HOME_SPEED_RAD_S` | 0.3 | rad/s |
| `HOME_ACCELERATION_RAD_S2` | 0.5 | rad/s² |
| `HOME_JERK_RAD_S3` | 2.0 | rad/s³ |
| `PRESET_SPEED_RAD_S` | 0.5 | rad/s |
| `PRESET_ACCELERATION_RAD_S2` | 1.0 | rad/s² |
| `PRESET_JERK_RAD_S3` | 4.0 | rad/s³ |

Home／预抓取速度同时受会话关节速度和 SDK 配置上限约束。运动中切换运动类型时先按当前类型平滑制动到零速度、零加速度，再应用新的类型，避免突然降低 jerk 上限而延长制动距离。加速度和 jerk 参数必须是有限正数，没有关闭开关。初值不是厂家认证的负载能力，尚需真机低速验收。Home 完成要求规划结束、位置误差在容差内且反馈速度小于 0.03 rad/s。

松开 Grip（不处于预设动作）时，客户端不再给位置目标，服务切换到零速度／零加速度的制动轨迹，不追赶旧目标。下拨 1 秒切回待机时通过显式 stop RPC 发起同样的制动，服务在规划结束且反馈位置、速度稳定后释放客户端控制权，并继续主动保持停止位置。停车期间拒绝新控制客户端接管，网页显示“平滑停车中”；停稳后才能下拨 1 秒重新启用。待机不自动结束已有录制。

正常待机的主动保持与故障保护不同：客户端失联／租约超时、异常退出、头显暂停和服务关闭继续使用原有阻尼／释放路径，不保证平滑减速。已授权的服务自主停车及后续保持不依赖已退出待机的客户端续约，但仍检查反馈有效性。没有速度反馈稳定时不会声称停稳。

SDK 接收服务规划的位置和速度，不再对这些点做第二次速度或超前量裁剪。可选 `MAX_COMMAND_LEAD_RAD` 改为在规划前限制目标相对反馈的距离；默认 0。SDK 自身插值、关节边界与力矩保护仍存在，因此限制保证针对规划轨迹，并不等于实际机械臂零过冲。规划若无法生成有效轨迹或将越过硬限位，服务进入既有故障保护，不以强行裁剪伪装连续性。

更新后先 `uv sync --locked`，退出遥操并重启机械臂服务，再启动遥操；只重启遥操无法加载新的规划器和协议。网页状态更新需要重启监看进程。后续仅调整脚本参数可重启遥操生效。
