# 运行配置

配置在根目录集中管理，供遥操、标定、仿真、采集和工具共同使用。

| 文件 | 职责 |
| --- | --- |
| `project.toml` | 可提交的共享默认值：模型、路径、相机规格、采集参数等 |
| `local.example.toml` | 本机配置示例；相机序列号需替换为实际值 |
| `local.toml` | 本机设备序列号、数据/模型路径等覆盖，不提交 |
| `profiles/*.toml` | 某个命令的参数组合，如日常 presets、collection、virtual-arm |
| `plugins.toml` | 受信任的插件工厂及能力声明；实现仍放在各自的 src 包 |

`robot_core.project.load_config()` 先读取 `project.toml`，再叠加选定的本机配置。选定顺序为启动器 `--config`、`QUEST_TELEOP_CONFIG`、默认 `local.toml`；显式指定的文件缺失时报错。相对路径从项目根目录解析。

共享默认解释器为 `.venv/bin/python`，由根目录 `uv sync --locked` 创建。本机配置通常无需指定 Python；如需诊断其他环境，显式使用 `QUEST_TELEOP_PYTHON` 或 `[environment].teleop_python`。解释器缺失会提示同步环境，不自动回退到 Conda 或系统 Python 执行控制器。

profile 的命令参数排在用户额外参数之前，后者可以覆盖前者。profile 不允许设置 `actuate`，真机动作需入口或命令显式启用。配置读取和 `--print-command` 不初始化设备。

`virtual-arm` profile 配置虚拟控制客户端（`control-arm`）的输入与轨迹参数，初始 `q` 在 `arm-service --backend simulated --q ...` 设置。真机遥操使用 `presets` / `collection` profile。三维监看单独运行 `view-arm` / `启动仿真.sh`。

`[arm_service]` 集中定义服务后端、RPC socket、频率、速度/软限位、SDK preview 和夹爪参数。`[robot]` 保留 CAN 与 URDF。服务端先读取本机覆盖，再应用 `arm-service` 的命令行参数。控制端显式传入的速度覆盖服务默认速度；未传速度时才使用服务默认值。CAN、SDK preview 等其他硬件参数仍由服务配置管理。

`[telemetry].endpoint` 定义状态发布通道，默认 `tcp://127.0.0.1:5567`；服务持续发布，可有多个监看。更换控制程序不会重启服务或中断状态流。服务控制接口默认为权限 0600 的 `outputs/runtime/arm.sock`，控制端与手眼标定读取同一配置。详见 [服务与监看说明](../docs/monitoring.md)。

目前使用单 X5 和单腕部 D435i，沿用上述结构。标定原图、会话与结果在 `outputs/calibration/`，不放入配置目录；未知外参和未完成标定状态保持原样。
