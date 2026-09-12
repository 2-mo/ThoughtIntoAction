# QuestArmTeleop

Quest Pro 遥操 ARX X5-2025。主入口 `teleop-presets-x5`：右 Grip 跟随、Trigger 控制夹爪、任务开启后 A 预抓取并开始采集、B 保存／长按回 Home、右摇杆左开始／右暂停／上作废上一条／按下长按结束清零；Grip 可取消预设动作。

已整理目录、依赖和启动配置，支持独立运动学仿真、单腕部 D435i 手眼标定，以及主遥操进程内的原始 episode 采集。命令行/脚本为主，网页只读监看。本机 π0.5 与 LeRobot 导出尚未接入；H800 只负责训练。

## 日常启动

环境由 uv 管理，首次安装执行下方的“安装与验证”。日常脚本直接使用项目 `.venv`，无需激活环境，也不会在启动机械臂时自动安装或更新依赖。脚本按自身位置解析路径，可以从任意工作目录调用。

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
# 启动后显示 TUI，底部常驻快捷键栏，无需回车：H 回零、P 预抓取、S/空格 停止、I 状态。
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

`configs/project.toml` 的 `[arm_service]` 集中管理接口、SDK preview、速度/软限位和夹爪参数，`[telemetry]` 管理状态流。presets / collection profile 保留输入与采集设置。主控制端不再提供 `--backend`、`--telemetry` 或 CAN/SDK 配置转发。详见 [机械臂服务与监看](docs/monitoring.md) 和 [插件说明](docs/plugins.md)。

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
src/robot_core/          共享模型、坐标变换、相机、设备锁与存储
src/robot_control/       机械臂服务/客户端、平滑轨迹、X5 后端与夹爪控制
src/handeye_calibration/ 手眼标定采样、计划、求解与验证
src/robot_simulation/    独立运动学仿真、共享 Three.js 场景、FK/IK 与轨迹预览
src/quest_teleop/        X5 遥操控制器、A/B 采集按键与 Home、采集会话与只读监看
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

根目录仅保留 `run.sh`、`启动机械臂服务.sh`、`启动遥操.sh`、`网页控制.sh` 四个日常入口。其他快捷脚本按用途放入 `scripts/inference/`、`scripts/arm/`、`scripts/datasets/` 和 `scripts/viewer/`，详见 [脚本索引](scripts/README.md)。连接准备、电量查询、Quest 唤醒分别使用 `./run.sh prepare-connections`、`./run.sh quest-battery`、`./run.sh quest-wake`。环境清单和配置留在根目录的专用文件夹；工具与实现归入 `scripts/` 和 `src/`，只维护当前入口和模块路径。具体见 [目录职责](docs/layout.md) 与 [源码分层](src/README.md)。

主控制器、预设动作和 X5 反馈/命令桥接由本项目维护；XRoboToolkit 提供通用 Quest/IK/线程层。默认获取、构建与启动均不需要 R5 SDK，见 [X5 控制器说明](docs/x5-controller.md)。

大型文件放在仓库外，路径可由本机配置覆盖：

```text
~/work/models/pi05_base/            现有 LeRobot 格式权重
~/work/datasets/quest_x5/            规划的数据集目录
~/work/installers/QuestArmTeleop/   APK/DEB 安装包
```

项目运行、构建和测试不依赖桌面备份。桌面备份保持原位、原样。

## 安装与验证

首次安装见 [安装说明](docs/setup.md)。在项目根目录执行：

```bash
./run.sh fetch-dependencies
uv sync --locked
./run.sh test
```

`uv sync --locked` 会安装锁定的 Python 包并编译 Quest 原生绑定。移动仓库后使用 `./run.sh rebuild-xr-sdk` 清理旧 CMake 路径并重建绑定。主环境不安装 R5、Torch/JAX 或上游其他机器人依赖；本机 π0.5 推理后续使用独立环境。

测试覆盖真实 Placo/Pinocchio 模型、虚拟反馈、模拟 XR、合成标定数据及异常清理，禁止真实 CAN 实例化；不替代真机动作验收。

更多入口见 [命令说明](docs/commands.md)，后续采集和策略边界见 [重构记录](docs/refactor.md)。

独立仿真、手动/自动手眼标定的命令与边界见 [仿真与标定](docs/simulation-calibration.md)。桌面实现的检查结果见 [迁移审查](docs/migration-review.md)。

数据采集、虚拟演练、回放、校验与只读网页见 [采集操作说明](docs/recording.md)。无需设备即可先运行 `./run.sh session demo`，再从另一终端运行 `./run.sh monitor --port 0`。


三维监看页已合并 D435i 腕部 RGB 预览与左右 Quest 手柄数据，保持单页布局。日常 `./启动遥操.sh` 默认开启采集会话（不自动录制），`./scripts/viewer/启动仿真.sh` 打开统一监看。详细数据来源及断流行为见 [监看说明](docs/monitoring.md)。


日常监看固定在 http://127.0.0.1:8081/，重启监看后刷新即可。遥操交互终端新增 `u / 3` 取消模拟佩戴：保留进程、暂停机械臂控制，由 Quest 自行息屏；`w / 1` 恢复模拟佩戴后自动等待新数据，松开 Grip/A/B/右摇杆按键后即可正常操作。


`./scripts/viewer/启动仿真.sh` 现在同时确保独立相机服务运行，不必先启动遥操：模型立即显示，D435i 初始化后显示 RGB；真实关节反馈由机械臂服务提供。未连接机械臂服务时显示“参考姿态”，不会把模型默认姿态当作真实反馈。相机服务由监看与采集共享，关闭页面或遥操后继续运行。


`./启动遥操.sh` 默认配置并确认 USB 数据连接，使用 Rich 显示当前连接方式；未确认 USB 时不启动控制、不自动回退 Wi-Fi。首次需将 Quest 客户端服务器地址设为 `127.0.0.1` 并连接。交互终端 `p` 可刷新连接状态。
