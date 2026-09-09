# 项目 X5 遥操控制器

当前 `teleop-presets-x5` 使用本项目维护的 X5 控制器。A/B 采集按键与 Home和录制会话在 `src/quest_teleop/`；设备通信、待机反馈与命令提交在 `src/robot_control/service.py`；默认运行不导入上游 R5 控制器或 `arx_r5_python`。

```text
run.sh teleop-presets-x5 → quest_teleop.cli.presets_main
  → quest_teleop.presets.PresetSingleX5Controller
  → quest_teleop.controller.X5TeleopController
  → quest_teleop.base_controller.X5TeleopBase
  → XRoboToolkit 通用 HardwareTeleopController / BaseTeleopController

控制端 arm 插件：service → ArmClient → 独立 arm-service
服务端 arm 插件：stanford → OfficialX5HardwareAdapter；simulated → SimulatedX5
input 插件：quest → XrClient；idle → 离线未激活输入
服务端 LocalPublisher → 持续状态发布
独立 view-arm 进程：订阅状态 → Three.js Scene
```

`base_controller.py` 维护六关节到 Placo 的映射、反馈更新、Grip 激活时的关节目标发送、Trigger 夹爪目标和日志接口。项目自行创建选中的输入/后端/状态发布器，复用上游 IK 方法；不调用会硬编码创建 XrClient 的上游构造函数。IK 与命令提交使用同一状态锁，主线程检查服务反馈，场景在独立监看进程刷新。组件初始化失败或运行退出时释放已创建资源，不包含上游 R5 的自动回零代码。

`controller.py` 维护 X5 服务接入、采集会话、指令来源记录和退出时的阻尼清理。`presets.py` 维护 A 预抓取后开始采集、B 保存／长按回 Home、右摇杆左开始／右暂停／上作废上一条／按下长按结束清零、Grip 取消预设并重建跟随参考。关节限速、0.05 rad 指令超前限制、SDK preview、夹爪限力保持均继续由原共享后端处理。

参数解析在 `src/quest_teleop/cli.py`，启动器直接运行该模块。Python 调用方直接使用 `X5TeleopController`，旧文件转发和类名别名已移除。无预设基线的会话元数据类名为 `X5TeleopController`，主入口记录 `PresetSingleX5Controller`，控制参数和数据 schema 不变。

`control-arm --input quest` 组合主 presets 控制器与虚拟后端，真机继续用现有 presets 入口。控制端只连接服务，不创建设备后端、状态发布器或三维场景；`启动仿真.sh` / `view-arm` 独立订阅与渲染，控制端退出或重启不会关闭监看。旧 `simulate` 命令已移除。历史 `teleop` / `teleop-dry-run` 保留独立实现，官方 X5 实验入口选用 `arx2025` 插件。官方 SDK 的真机位置执行禁用仍有效。主采集入口继续用 `--enable-session` 共享单 D435i；相机按配置中的 `driver` 加载，历史 `--enable-camera` 保留其已有采集实现。

默认依赖获取/检查仅要求 XRoboToolkit 与 PC-Service-Pybind 源码；`uv sync --locked` 安装主环境并构建 XR 服务绑定，移动仓库后使用 `./run.sh rebuild-xr-sdk` 修复绑定。启动器不再添加 R5 的动态库目录。两份 R5 源码都保留为 `required=false` 的参考，只有 `fetch-dependencies --all` / `check-dependencies --all` 才处理它们；新的 `.venv` 不安装 R5。

配置中沿用的 `/dev/arx_r5` 是 USB-CAN 适配器的 udev 别名，与 `/dev/arxcan0` 指向同一个设备；它不是 R5 SDK 依赖，本次未更换 CAN 接口或设备绑定。

## 来源与许可

复用逻辑来自 [XRoboToolkit-Teleop-Sample-Python](https://github.com/XR-Robotics/XRoboToolkit-Teleop-Sample-Python)，固定 commit `79e5cb8a56e3455515ce1b476e993c764ec58739` 的 `xrobotoolkit_teleop/hardware/arx_r5_teleop_controller.py`。迁入关节映射、反馈/命令与兼容日志方法，重写 X5 初始化配置；不迁入 R5 SDK 导入、R5 模型、示例 CAN/相机配置或自动回零行为。

Copyright (c) 2025 XR Robotics，原 MIT 许可保存在 [LICENSE.XRoboToolkit](../src/quest_teleop/LICENSE.XRoboToolkit)。上游源码和锁定版本保持原样；项目对迁入代码的修改在主仓库维护。

## 验证边界

离线检查覆盖 R5 导入被阻止时的 X5 命令帮助、真实 Placo/Pinocchio 加虚拟反馈、A/长按 B/Grip 和 Trigger、录制来源与退出阻尼。新增插件替换、仿真拒绝硬件/真实录制、输入和显示故障清理，以及共享场景服务退出验证。进程加载库列表用于确认没有间接加载 R5 动态库；依赖检查使用可选仓库缺失的临时目录验证。

本次不执行真机运动。现场遥操与自动标定验收仍使用既有步骤。

主入口硬件参数已集中到 `[arm_service]`，不再转发 `--backend`、`--telemetry`、CAN 或 SDK preview。手眼标定共享该服务，控制退出只释放控制权，设备与状态发布继续运行。详见 [机械臂服务与监看](monitoring.md)。
