# 工具脚本

日常从根目录 `run.sh` 调用，命令见 [命令说明](../docs/commands.md)。

| 位置 | 职责 |
| --- | --- |
| `launch.py` | 解析配置/profile、选择 Python 环境、调用源码模块或工具 |
| `check.py` | 运行离线测试 |
| `setup/` | 依赖安装、CAN、udev 和连接准备 |
| `quest/` | Quest 服务、USB、唤醒、电量与输入检查 |
| `diagnostics/` | 相机检查、机械臂反馈检查与关节跟踪测试 |

控制器、仿真和网页实现位于 `src/`；脚本通过 `robot_core.project` 使用共享配置。`scripts/` 不加入正常运行的 `PYTHONPATH`，`src/` 不反向导入脚本。
