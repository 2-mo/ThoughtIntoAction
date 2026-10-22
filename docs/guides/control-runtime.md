# 控制运行时

控制内核负责控制权、命令执行、反馈和记录。机器人驱动、运动学、仿真、输入设备通过 `configs/plugins.json` 注册；当前设备实现包括 ARX X5 真机、MuJoCo 仿真和 Quest 输入。公共运动算法位于 `src/embodied/motion/`，公共协议位于 `src/embodied/contracts/`。

控制环境独立使用 Linux x86_64 / Python 3.10。环境定义在 `configs/environments/control/`，解释器安装到 `outputs/envs/control/bin/python`。与工作台使用不同 Python 环境，源码仍共同放在 `src/`。

## 文件位置

| 内容 | 路径 |
| --- | --- |
| 控制循环、命令网关、IPC、记录 | `src/embodied/control/` |
| ARX 配置加载、真机、仿真、运动学和退出策略 | `src/embodied/devices/robots/arx5/` |
| Quest 配置、输入、原生路由和程序 | `src/embodied/devices/inputs/quest/` |
| 插件注册表 | `configs/plugins.json` |
| 机器人与工具配置 | `configs/robots/arx5/robot.json`、`tool.json` |
| 运动参数与工作空间 | `configs/control/motion.json`、`workspace.json` |
| Quest SDK、坐标和按键配置 | `configs/inputs/quest.json` |
| 配套 URDF、MJCF、网格与模型来源 | `assets/robots/x5/runtime/` |
| 安装、启动、检查、模型生成和发布工具 | `scripts/control/` |
| 每次运行的反馈、参数和日志 | `outputs/runtime/control/<RUN>/` |
| 原 arx-televr 来源记录与许可证 | `docs/provenance/arx-televr/` |

机器人型号与输入设备独立选择。未选择 Quest 输入时，内核不会读取 Quest 配置或加载其 SDK。模型、工具和输入 SDK 的相对路径分别以所属配置文件所在目录为基准。

## 完整仓库入口

以下命令从仓库根目录执行：

```bash
./run.sh control setup
./run.sh control doctor
./run.sh control start --backend sim --input idle --managed
```

这会打开可接受统一 API 命令的仿真内核。没有桌面显示的终端加 `--headless`。完整仓库中的 `docs/reference/robot-primitives.md` 提供操作示例和 `Robot.sim()` 用法。

需要主动执行离线检查时运行 `./run.sh control check`；`--simulation` 额外执行仿真通路。检查入口不连接真机，安装和启动命令各自显式执行。

## 独立控制包入口

发布包保留相同的 `src/`、`configs/`、`assets/` 和 `scripts/` 布局，包含控制运行时及其 ARX/Quest 适配器。解压后在包根目录安装环境：

```bash
./scripts/control/setup.sh sim
./scripts/control/doctor.sh
./scripts/control/start_demo.sh --headless --duration 12
```

需要 ARX SDK 时将安装参数改为 `arx`。安装使用锁定依赖和 Python 3.10；缺少解释器时 uv 可下载。普通运行入口使用 `outputs/envs/control/bin/python`，不会执行依赖安装。`ARX5_PYTHON` 可指定安装时使用的 Python 解释器。

| 操作 | 独立包命令 |
| --- | --- |
| 检查当前机器人模型 | `./scripts/control/doctor.sh` |
| 检查 Quest SDK 配置 | `./scripts/control/doctor.sh --vr` |
| 合成输入控制仿真 | `./scripts/control/start_demo.sh` |
| Quest 控制仿真 | `./scripts/control/start_sim.sh` |
| Quest 控制真机 | `./scripts/control/start_real.sh --headless` |
| 离线检查 | `./scripts/control/check.sh` |

`doctor --vr` 校验输入配置和 SDK 文件，不能证明头显已连接。真实 Quest 输入还需要外部 XRoboToolkit PC Service 与头显应用；SDK 路径可在 `configs/inputs/quest.json` 设置，或通过 `XROBOT_SDK_LIBRARY` 覆盖。

## 真机 SDK 补丁

真机入口要求夹爪上电跨圈补丁版本，SDK wheel 本身不包含此补丁。构建使用仓库维护的固定源码包和补丁头文件，产物放在 `outputs/build/control/arx-gripper/source/python/`。环境移动或更换 Python ABI 后需要重建。

先准备开发头文件，提取方法见 [夹爪上电修复](gripper-startup.md) 的开发头文件部分。默认头文件目录为 `outputs/build/arx-gripper/deps/usr/include`。在完整仓库中执行：

```bash
./run.sh control build-sdk
# 头文件位于其他目录时：
./run.sh control build-sdk --headers /path/to/usr/include
```

独立控制包在包根目录执行以下命令，使用随包的同一份构建脚本：

```bash
UV_PROJECT_ENVIRONMENT="$PWD/outputs/envs/control" uv sync --locked \
  --project configs/environments/control --python 3.10 --extra arx --group sdk-build
outputs/envs/control/bin/python scripts/setup/build_gripper_sdk.py \
  --build-dir outputs/build/control/arx-gripper \
  --headers outputs/build/arx-gripper/deps/usr/include
```

这些命令构建 SDK，不连接 CAN。完整仓库的真机启动器和独立包 `start_real.sh` 都显式加载该产物并要求补丁版本；缺少补丁会拒绝启动。

真机启动会使能机械臂。默认原生 Quest 模式会回到配置的 Home 并打开夹爪；`--managed` 启动先保持测量位置，由取得租约的客户端下发目标。CAN 接口应预先配置，启动器不自动配置网络。退出真机会请求 SDK 阻尼，不额外回 Home，退出前需要支撑机械臂。

## 配置与模型

- `configs/robots/arx5/robot.json` 选择 `profile`，设置关节范围、Home、控制频率、CAN 和模型路径。关节单位为 rad，夹爪宽度为 m。
- `configs/robots/arx5/tool.json` 定义 TCP 和额外负载。模型加载器检查工具与生成模型的一致性。
- `configs/control/motion.json` 是统一内核运动参数的唯一配置入口，含关节速度/加速度/jerk、可选 TCP 约束和到位条件；参见 [字段与生效规则](motion-settings.md)。运行时的实际速度能力由所选驱动报告；`speed=0` 使用逐轴能力，`speed_scale` 对每轴按比例缩放。
- `configs/inputs/quest.json` 定义 SDK 路径、基座旋转、Grip 阈值和位置比例。内置原生 Quest 路由目前要求 250 Hz 机器人配置。

修改 TCP、负载或原始 URDF 后，在根目录显式重建匹配模型：

```bash
outputs/envs/control/bin/python scripts/control/build_model.py
./scripts/control/doctor.sh
```

本机配置可复制为 `robot.local.json` 或 `quest.local.json`，通过 `--config`、`--vr-config` 指定。保留配置与资源之间正确的相对路径；`*.local.json` 不进入发布包。

## 记录与发布

每次运行的 `runtime.json` 记录后端、插件和运行标识，`parameters.json` 记录生效配置与源码身份，`report.json` 记录退出结果。真机自动写入 `tuning.jsonl`；带遥测启动时另有 Tick 数据流。无法取得的硬件字段保留为不可用。

从完整仓库或已有控制包根目录生成发布包：

```bash
python3 scripts/control/package_release.py
```

默认生成 `outputs/releases/embodied-control.tar.gz` 和 `.sha256`；包内 `MANIFEST.sha256` 记录交付文件校验值。包中包含控制源码、插件注册、锁定依赖、模型、原生程序、SDK 源码与构建脚本、来源和许可证；环境、运行记录、本机配置及 `third_party/` 不进入交付包。平台源码继续正常维护，历史来源清单不作为禁止修改源码的运行门槛。
