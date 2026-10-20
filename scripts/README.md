# 工具脚本

日常从根目录 `run.sh` 调用，命令见 [命令说明](../docs/commands.md)。

| 位置 | 职责 |
| --- | --- |
| `workspace/` | tmux 统一工作台启动、窗口状态与有序退出 |
| `launch.py` | 解析配置/profile、选择 Python 环境、调用源码模块或工具 |
| `check.py` | 运行离线测试 |
| `setup/` | 依赖安装、CAN、udev 和连接准备 |
| `quest/` | Quest 服务、USB、唤醒、电量与输入检查 |
| `inference/` | Pi05 原版/微调推理启动及独立停止 |
| `arm/` | 独立机械臂动作（回零） |
| `datasets/` | 数据导出和转换环境准备 |
| `viewer/` | 带自定义参数的网页监看入口 |
| `diagnostics/` | 相机检查、机械臂反馈检查与关节跟踪测试 |

控制器、仿真和网页实现位于 `src/`；脚本通过 `embodied.core.project` 使用共享配置。`scripts/` 不加入正常运行的 `PYTHONPATH`，`src/` 不反向导入脚本。

## 快捷入口

根目录提供统一入口 `启动工作台.sh`，以及独立入口 `run.sh`、`启动机械臂服务.sh`、`启动遥操.sh`、`网页控制.sh`。

运行 `bash 启动工作台.sh` 创建或进入同一 tmux 会话，默认打开 Rich 总控页（窗口 4），在页面内启停、重连、查看详情；`status` 查看窗口，`stop` 有序退出工作台控制台（网页和相机继续运行），`restart arm|teleop|web` 重开已退出窗口。总控 `Z` / 命令行 `shutdown` 完整关闭服务并退出，总控 `R` / 命令行 `restart-all` 有序完整重启。机械臂与遥操的参数仍编辑原脚本。
网页日常启停使用 `./网页控制.sh`；下表保留按用途分类的快捷操作，路径均相对项目根目录，原有参数保持不变。

| 用途 | 当前路径 |
| --- | --- |
| Pi05 真机推理 | `scripts/inference/启动Pi05真机.sh` |
| 微调 Pi05 真机推理 | `scripts/inference/启动微调Pi05真机.sh` |
| 停止 Pi05 | `scripts/inference/停止Pi05.sh` |
| 独立回零 | `scripts/arm/机械臂回零.sh` |
| 导出训练数据 | `scripts/datasets/导出训练数据.sh` |
| 网页监看 / 自定义端口 | `scripts/viewer/启动仿真.sh` |
| 同一工作台 / 网页选择仿真（18081） | `bash 启动工作台.sh --mode simulation`；总控 `M / F` 切模式，`T / S` 遥操待机 / 暂停保持 |

例如：`./scripts/viewer/启动仿真.sh --port 8082 --no-camera`。
脚本可从其他工作目录用绝对路径运行；根目录不再保留同名转发脚本。
