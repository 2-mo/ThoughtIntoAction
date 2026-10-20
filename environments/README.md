# 环境边界

遥操、采集、手眼标定和仿真共享根目录的 uv 环境：

| 文件/目录 | 用途 |
| --- | --- |
| `../pyproject.toml` | 直接依赖、数值库 ABI 约束、原生构建依赖 |
| `../uv.lock` | Python 运行依赖和可选构建工具的解析版本 |
| `../.python-version` | uv 管理的 CPython 3.12.12 |
| `../.venv/` | 本机安装结果，不提交、不复制到另一台机器 |
| `snapshots/conda-teleop/` | 迁移前 Conda 基础包和直接 pip 包清单，仅作历史参考 |
| `snapshots/teleop-installed.txt` | 迁移前全部可见 Python 分发包盘点，不是安装清单 |

在根目录运行 `uv sync --locked`。使用 `run.sh` 启动功能，启动器直接加载项目 `src/` 和锁定的 XRoboToolkit 源码。这里是从 checkout 运行的应用，`tool.uv.package=false`；所用 XR/IK/可视化模块的运行依赖明确列在主项目中，上游源码由 `dependencies.lock.toml` 单独固定。上游多机器人 Python 分发包不安装，PC Service 的原生 Python 绑定由 uv 从源码构建。

保留 NumPy 2.3.5、Pinocchio 3.7.0、Placo 0.9.3、Meshcat 0.3.2、OpenCV 5.0.0.93、pyrealsense2 2.58.4.10922 和 arx5-interface 0.1.3。旧环境的 `eiquadprog 1.3.2` 要求 Boost 1.90，与 Pinocchio 使用的 Boost 1.87 冲突；新环境固定兼容的 `eiquadprog 1.2.9`，并锁定 cmeel/URDFDOM 等原生依赖，避免只满足宽泛版本约束却加载到不匹配的 C++ 库。

默认环境不安装 R5、Torch/JAX、其他机器人驱动或完整 ROS。编译器、USB/CAN 工具、Quest PC Service、系统驱动仍属于系统安装项，不能仅靠 `uv.lock` 重建。独立官方 ARX 实验后端另需兼容的 KDL/ROS 系统库，见 [安装说明](../docs/guides/setup.md)。

原 `xrobot312` Conda 环境未修改，保留作临时回退；默认启动配置不再指向它。不在同一个环境中交替使用 Conda/pip 和 uv 改写依赖。

本机 π0.5 推理使用独立环境，H800 只负责训练。当前参考 openpi 要求 NumPy < 2，与遥操环境冲突；现有权重是 LeRobot 格式，需先确定实际推理实现，再建立 `inference/` 和 `training-h800/` 各自的依赖声明与锁文件。本次没有安装或接通策略后端。训练与推理需匹配模型实现、数据 schema 和归一化配置，GPU 构建按各自硬件选择；权重和数据路径继续放在本机配置中。
