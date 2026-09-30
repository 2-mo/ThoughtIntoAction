# ARX TeleVR

基于 PICO 的 ARX X5 遥操作，支持实时关节控制与 MuJoCo 仿真。

独立的 **AC one X5 2025** 控制工程：PICO / XRoboToolkit 输入、原生 VR 路由、六关节 IK、Stanford `set_joint_cmd()` 和 MuJoCo 仿真。只保留 ARX 所需代码；机器人控制、VR 输入、模型与配置分开。

## 目录

```text
config/
  robot.json          # CAN、Home、关节增益、模型路径、夹爪参数
  vr.json             # XR SDK 路径、坐标旋转、Grip 阈值、移动比例
  tool.json           # TCP 与额外负载，修改后重建模型
models/x5/
  x5_2025.source.urdf  # 操作者验证的原始动力学模型
  x5_2025.urdf         # SDK 使用的 URDF，包含指尖 TCP
  x5_2025.xml          # IK / MuJoCo 使用的对应模型
  x5_2025.model.json   # 模型及工具配置的校验值
  meshes/             # 配套网格
arx5_vr/
  real.py             # ARX set_joint_cmd 适配器
  model.py            # FK、IK、关节目标和 Home
  sim.py              # MuJoCo 后端
  ipc/                # VR 与机械臂的命令/反馈边界
  native_config.py    # 独立 VR 进程配置
  worker.py           # 250 Hz 调度、反馈检查、退出
  recording.py        # 电机信息、命令与参数记录
native/
  bin/                # 两个原生 VR 程序
  rollio-source.tar.gz # 所需原生源码、许可证、重建依据
scripts/              # 模型生成、离线检查和打包
```

`run/` 和 `.venv/` 在本机生成，不进入交接包。原生依赖源码压缩保存，解压/重建说明见 [native/README.md](native/README.md)。

## 安装与迁移

运行平台为 **Linux x86_64 / Python 3.10**。原生程序要求 glibc ≥ 2.34、GLIBCXX ≥ 3.4.29。安装 [uv](https://docs.astral.sh/uv/getting-started/installation/)，然后获取仓库并安装：

```bash
git clone https://github.com/GZF123/arx-televr.git
cd arx-televr
./setup.sh
./check.sh --simulation
```

`setup.sh` 按 `uv.lock` 创建本目录虚拟环境，默认安装 ARX SDK。首次安装需要网络；缺少 Python 3.10 时 uv 可下载解释器。仅仿真可用 `./setup.sh sim`。检查使用假硬件，不连接 CAN；`--simulation` 额外执行合成输入、原生路由和 MuJoCo 通路。无需原 Rollio、旧工作目录或 ROS。

真实 VR 另需安装、启动 XRoboToolkit PC Service 与头显应用，确认右手控制器追踪正常。PC Service 不在交接包中。设置 `config/vr.json` 的 `sdk_library`，或按本机路径覆盖：

```bash
cp .env.example .env
# 编辑 .env 中的 XROBOT_SDK_LIBRARY
source .env
./doctor.sh --vr
./start_sim.sh
```

环境变量优先；`.env` 需手动 source。`doctor --vr` 只确认动态库文件存在，不证明头显已连接。

## 启动

| 用途 | 命令 |
| --- | --- |
| 检查依赖、模型和原生程序 | `./doctor.sh` |
| 无头显演示 | `./start_demo.sh --headless --duration 12` |
| VR 控制仿真 | `./start_sim.sh` |
| **VR 控制真机** | `./start_real.sh --headless` |

所有脚本使用自身目录的 `.venv`，支持中文与空格路径。`--help` 显示参数。若手动用 uv 启动，请使用 `uv run --no-sync python ...`，保留已安装的 ARX 可选依赖。普通启动不自动配置 CAN；真机需要操作者已经配置好 `can0`。

**真机启动会使能并回 Home。真机由操作者运行。** 不要同时从两个工程副本控制同一根机械臂。

## 配置

- `config/robot.json`：`park_q` 使用 rad，默认 `[0,0,0,0,0,0]`；以后可改为 `[0,1.2,1.4,0,0,0]`。位置、速度单位为 rad、rad/s。`can_interface` 默认 `can0`。
- `config/vr.json`：`base_yaw_deg` 单位为度；`position_scale` 为 XYZ 相对移动比例；`grip_threshold` 默认为 0.7。控制频率固定 250 Hz，原有模式与超时策略保留。
- `config/tool.json`：TCP 为两指尖中点，相对 link6 的 `[0.145,0,0]` m，旋转为零。这是 SDK 标称尺寸，尚未实物测量。`payload` 仅表示原夹爪之外新增的负载。
- `models/x5/x5_2025.source.urdf`：保留操作者多姿态验证平衡的 link6 有效质量 **0.481 kg**，不是称量值。当前 SDK 使用随附 URDF，不依赖旧虚拟环境里的模型修改。
- 夹爪最大宽度 `0.082 m`，`gripper_open_readout=-3.4`。此前打开夹爪断电重启曾产生负宽度读数，应闭合后上电。

修改 TCP、负载或原始 URDF 后，生成匹配模型：

```bash
.venv/bin/python scripts/build_model.py
./doctor.sh
```

修改工具配置却未重建，校验会拒绝启动。URDF、XML 与工具配置同步校验，避免 SDK 和 IK 使用不同 TCP。

本机配置可独立复制：

```bash
cp config/robot.json config/robot.local.json
cp config/vr.json config/vr.local.json
./start_sim.sh --config config/robot.local.json --vr-config config/vr.local.json
```

机器人模型、工具文件和 SDK 的相对路径均以各自配置文件所在目录为基准。`*.local.json` 不进入交接包。

## 右手按键

| 操作 | 行为 |
| --- | --- |
| A 按住至少 150 ms，然后松开 | Disabled 后进入 FreeDrive；之后在 FreeDrive / CommandFollowing 间切换 |
| A+B | 回配置的 Home，夹爪打开；**不是急停** |
| 跟随模式握住 Grip | 捕获当前参考，开始相对位姿跟随 |
| 松开 Grip，再握紧 | 解除跟随、重新对齐参考 |
| Trigger | 越深越闭合夹爪，独立于 Grip |

启动后等待回 Home 且至少三秒再切换模式。FreeDrive 使用重力补偿。终端 Ctrl+C 让真机进入 SDK 阻尼并退出，不额外回 Home；退出前支撑机械臂。仿真正常退出会回 Home。

## 接口与记录

`real.py` 每 4 ms 调用 `set_joint_cmd(cmd)`、`set_gain(gain)`、`send_recv_once()`。六关节位置、速度与附加前馈力矩分别写入 `cmd.pos()`、`cmd.vel()`、`cmd.torque()`；SDK 自动重力只加一次，附加力矩不要重复加重力。

VR 在独立原生进程中运行，通过 `ipc/RobotBridge` 传递末端目标。自己的 IK 可接在 `model.py` 的 `X5Model.solve_ik()`。数据契约和扩展点见 [接口说明](docs/INTERFACES.md)。

真机自动将反馈、输出目标、SDK 增益及参数快照写入 `run/<本次运行>/`。故障查看 `backend.log` 与 `report.json`；逐 tick 数据为 `tuning.jsonl`。未公开的电流、温度、电压和故障码保留为不可用，不推算填充。

## 交接状态

本次保留既有 ARX 控制行为、VR 链路、默认 Home 与动力学参数。校准平衡和实体键盘是操作者此前验证的结果；本次迁移只做离线检查和仿真，不驱动真机。详细结果见 [验证记录](docs/VALIDATION.md)。

生成可转交压缩包：

```bash
.venv/bin/python scripts/package_release.py
```

输出 `dist/arx-televr.tar.gz` 与 `.sha256`。不包含环境、运行日志、私有配置或缓存。收件方解压后执行 `./setup.sh`。`.gitignore` 同样排除这些本机文件。根目录和模型目录保留原许可证；原生来源记录随源码归档保存。
