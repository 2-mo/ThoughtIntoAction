# 命令与控制器

统一从根目录 `run.sh` 启动；启动器直接运行 `src/` 模块或 `scripts/` 工具。共享 Stanford X5 后端位于 `src/robot_control/hardware.py`，关节目标限幅与五次轨迹位于 `src/robot_control/trajectory.py`；调用方直接导入这些模块，Quest 控制流程与 A/B/Grip 行为不变。共享组件与独立标定、仿真的位置见 [源码分层](../src/README.md)。

主控制器与预设状态机位于 `src/quest_teleop/controller.py` 和 `presets.py`；真机参数解析在 `src/quest_teleop/cli.py`，虚拟控制参数在 `src/robot_control/cli.py`，监看参数在 `src/robot_simulation/cli.py`。全部项目 X5 入口使用自己的 `X5TeleopBase`，不再导入 R5 控制器或 SDK，见 [X5 控制器说明](x5-controller.md)。

| 命令 | 行为 |
| --- | --- |
| `teleop-presets-x5` | 项目 X5 主控制器，通用 XR Grip/IK、A/B 采集按键与 Home，通过机械臂服务控制，默认不启用位置控制 |
| `teleop-official-x5` | 服务客户端，无 A/B 采集按键与 Home动作，默认不启用位置控制 |
| `arm-service` | 独立设备通信、状态发布与控制权管理；默认真机阻尼待机，`--backend simulated` 用于虚拟服务 |
| `view-arm` | 独立三维监看，只订阅状态，不创建输入或机械臂后端 |
| `control-arm` | 虚拟机械臂控制，终端 FK/IK/回放或 `--input quest`；连接独立虚拟服务，不创建后端或网页 |
| `handeye` | 单 D435i 眼在手上标定；`auto` 默认离线，真机运动需 `--actuate` |
| `session` / `record` | 查询当前会话、合成演练、遥操原始数据录制开始/结束 |
| `dataset` | 原始数据查看、检查、离线回放及结果标注 |
| `doctor` / `monitor` | 设备/环境只读诊断、本机只读网页监看 |
| `teleop` | 历史自定义 XR 校验/离合控制器；原有默认行为会执行动作 |
| `teleop-dry-run` | 历史控制器的只读/阻尼入口 |
| `teleop-x5-2025` | ARX 官方 type=2 SDK 实验入口，位置执行仍被代码禁止 |
| `test-joint` | Stanford SDK 关节检查，默认只读，`--actuate` 才做小幅动作测试 |
| `test-joint-2025` | 官方 SDK 实验性关节检查，默认只读，不属于主遥操路径 |
| `prepare-connections` | 准备 Quest USB 与 CAN 连接，不启动遥操 |
| `quest-battery` | 查询 Quest 电量 |
| `quest-wake` | Quest 唤醒与佩戴状态模拟 |
| `check-xr` / `visualize-xr` | XR 检查与浏览器可视化 |
| `check-arm` | 只读服务反馈，不申请控制权；`--virtual` 读取虚拟服务 |
| `fetch-dependencies` / `check-dependencies` | 使用系统 Python 获取/检查锁定源码，创建 `.venv` 前也可用 |
| `test` / `check-all` | `test` 在主环境运行 pytest；`check-all` 依次运行 control、platform、motion、phone、rebot、browser 各组离线检查，缺少环境的组显示 SKIP；`--only`、`--simulation`、`--print` |
| `rebuild-xr-sdk` | 保存旧 CMake 缓存并用 uv 重建 Quest 绑定，适用于移动仓库后的修复 |
| `plugins` | 查看可替换后端、输入、场景和相机的注册信息，不初始化设备 |

环境同步直接在仓库根目录执行 `uv sync --locked`。日常命令使用 `.venv/bin/python`，不隐式同步或安装依赖。

`official` 是沿用的命令名：当前已使用项目自己的 X5 控制器，继续复用 XRoboToolkit 通用控制流程；主硬件后端仍为 `arx5-interface==0.1.3`。只有 `*-2025` 实验入口使用 ARX 官方闭源后端。

主入口、官方基线和统一仿真共用 `assets/robots/x5/X5_2025_hybrid.urdf`。历史 `teleop` 为保留旧行为，仍使用已安装 SDK 的自带模型，不作为新默认入口。

```bash
./run.sh --profile presets teleop-presets-x5 --help
./run.sh --profile presets teleop-presets-x5
./run.sh --profile presets teleop-presets-x5 --actuate
```

主入口保持既有夹爪接触后限力保持逻辑。Python 默认速度为 `0.2 rad/s`；`presets` profile 保留用户原命令的 `0.5 rad/s`、`0.5` 位移缩放与 `0.03 s` preview。

ARX 官方 SDK 曾出现非指令关节漂移，`teleop-x5-2025 --actuate` 仍禁止，重构未解除限制。

历史自定义控制器的 CSV 日志迁到 `outputs/logs/`，可由 `paths.log_dir` 配置。主 `teleop-presets-x5` 增加 `--enable-session` 后会输出会话诊断日志，并接受独立终端的原始 episode 录制命令；见 [采集说明](recording.md)。未开启时保留原来的终端诊断。既有 TXT 记录保存在 `docs/validation/`。

源码位置见 [目录职责](layout.md)。虚拟控制由 `robot_control.cli` 运行；选择位姿输入时组合主 presets 控制器。真机与虚拟控制均通过 `robot_control.client` 连接服务，由服务通过 `robot_core.telemetry` 发布状态；独立监看在 `robot_simulation/cli.py` 和 `scene.py`；旧的继承式虚拟控制器和命令别名已移除。历史自定义控制器位于 `src/quest_teleop/legacy/`，提供独立实现。插件组合和接口见 [插件说明](plugins.md)。
