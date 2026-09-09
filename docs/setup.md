# 环境与设备安装

主环境使用 uv、Python 3.12 和项目内 `.venv`，当前支持 Linux x86_64。`run.sh` 使用 `/usr/bin/python3`（3.11+）解析配置，然后直接启动 `.venv/bin/python`，不受终端中 Conda 激活状态影响。启动不自动安装或升级依赖。

独立 `view-arm` / `启动仿真.sh` 通过 Python 标准库启动 Three.js 本地服务，独立机械臂服务通过锁定的 pyzmq 发布状态。前端模块和材质随仓库保存，无需安装 Node/npm，也不依赖 CDN；浏览器需支持 WebGL 2。场景资产来源与可选浏览器测试见 [共享场景说明](../src/robot_simulation/web/README.md)。

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

重建命令把旧 CMake 构建目录移入 `outputs/build-cache/`，然后通过 uv 禁用缓存并重建 `xrobotoolkit-sdk`。只作用于本项目 `.venv`。更换机器时重新执行首次安装，不复制 `.venv`。不要把本机推理框架安装到遥操环境，环境边界见 [环境说明](../environments/README.md)。

源码获取只处理缺失目录；已有 commit 不匹配会报错，不会 reset 本地改动。默认获取/检查只需 XRoboToolkit 和 PC-Service-Pybind 两个源码仓库；X5 SDK 使用已安装的 `arx5-interface`。`--all` 额外处理可选官方 X5、模型源码、openpi，以及官方 R5 和 XR 内嵌 R5 fork 参考仓库。两份 R5 均不参与项目 X5 的构建和运行。

迁移前的 Conda 清单归入 `environments/snapshots/conda-teleop/`；`environments/snapshots/teleop-installed.txt` 保留全部包盘点，其中的 `arx_r5_python` 仅是历史记录。原 Conda 环境没有被修改。

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

Quest 安装 XRoboToolkit APK；主机安装匹配的 PC Service DEB。安装包目录由 `paths.installer_dir` 记录。连接 USB 并允许 ADB 调试：

```bash
adb devices -l
./run.sh setup-quest-usb
./run.sh start-xr-service
```

Quest 的 XRoboToolkit server IP 设为 `127.0.0.1`。`./启动遥操.sh` 默认先配置 USB reverse，随后自动执行 `quest-wake start`，唤醒头显、发送模拟佩戴信号并打开 XRoboToolkit 应用，再启动遥操。也可单独使用 `./run.sh quest-wake start`，结束使用 `./run.sh quest-wake stop`。唤醒命令返回不代表 XR 数据已连接；控制端会等待新帧。

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

日常先在独立终端执行 `./启动机械臂服务.sh`，再按需启动 `./启动仿真.sh` 和 `./启动遥操.sh`。仅调试虚拟机械臂时，服务改用 `./run.sh arm-service --backend simulated`，控制用 `./run.sh control-arm`。服务和监看可以在没有控制程序时保持运行。

真机服务初始化 SDK 时会使能并进入阻尼；监看和 `check-arm` 只读服务，不初始化 SDK。`check-arm --virtual` 读取虚拟服务。服务配置集中在 `[arm_service]`，RPC socket 为 `outputs/runtime/arm.sock`，状态流由 `[telemetry]` 指定。控制端关闭后服务继续反馈，失联控制权会在 0.5 秒后撤销。详细运行边界见 [机械臂服务与监看](monitoring.md)。


日常遥操默认使用 USB：启动脚本用 Rich 展示设备、63901 转发和实际 XR TCP 连接状态；唤醒后最多等待 15 秒确认 USB 连接，再启动机械臂控制端。若仍发现 Wi-Fi 连接或没有 USB 数据连接，会保持未启动/暂停状态，提示在 Quest 客户端设置 `127.0.0.1` 并 Connect，再按 `w` 重试。程序不能通过现有接口替 Quest 修改客户端服务器地址，该设置首次需在头显内完成。不会自动回退 Wi-Fi。

`p` 会刷新连接方式提示，也可单独执行 `./run.sh quest-transport status`（只读）。`./run.sh setup-quest-usb` 仍可单独配置 USB 转发。Rich 依赖由 uv 管理；`--help` 不触发设备操作。本次实现没有重连正在运行的真机。

### 夹爪张开端点

`configs/project.toml` 的 `[arm_service].gripper_open_limit_m = 0.077` 限制张开指令，遥操满开目标同步使用此值。位置换算仍采用 `gripper_width = 0.082`、`gripper_open_readout = -3.4`，不改变反馈标尺或 SDK 力矩保护。

此上限根据当前夹爪空载全开时反馈约 0.0789 m、力矩约 -0.98 的受阻现象设置，留出约 1.9 mm 的反馈坐标余量；尚需真机确认无持续顶压，并不等于已完成物理开度标定。修改后需结束遥操并重启机械臂服务，再启动遥操；不要只重启网页。若仍有空载张开力矩警告，应继续检查端点／零点或机械阻力，不提高力矩阈值掩盖问题。
