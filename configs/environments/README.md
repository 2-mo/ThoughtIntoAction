# 环境边界

平台应用、采集与标定使用根目录 uv 环境；控制 worker 单独使用 Python 3.10。环境声明与安装结果分开：

| 文件/目录 | 用途 |
| --- | --- |
| `../../pyproject.toml` | 直接依赖、数值库 ABI 约束、原生构建依赖 |
| `../../uv.lock` | Python 运行依赖和可选构建工具的解析版本 |
| `../../.python-version` | uv 管理的 CPython 3.12.12 |
| `../../.venv/` | 本机安装结果，不提交、不复制到另一台机器 |
| `control/pyproject.toml`、`control/uv.lock` | 控制环境声明与锁版本，Python 3.10 |
| `../../outputs/envs/control/` | 控制环境安装结果，运行 `./run.sh kernel setup` 创建 |
| `snapshots/conda-teleop/` | 迁移前 Conda 基础包和直接 pip 包清单，仅作历史参考 |
| `snapshots/teleop-installed.txt` | 迁移前全部可见 Python 分发包盘点，不是安装清单 |

在仓库根目录运行 `uv sync --locked`，按需运行 `./run.sh kernel setup` 准备控制环境。使用 `run.sh` 启动功能，启动器加载项目 `src/`，正式外部依赖通过锁版本安装；不从 `third_party/` 导入 Python 模块或动态库。需要维护的 Quest 辅助代码位于所属设备模块的 `_vendor/`，来源与许可证随代码保存。受管构建/安装工作区位于 `outputs/dependencies/`，依赖身份由 `dependencies.lock.toml` 固定。

保留 NumPy 2.3.5、Pinocchio 3.7.0、Placo 0.9.3、Meshcat 0.3.2、OpenCV 5.0.0.93、pyrealsense2 2.58.4.10922 和 arx5-interface 0.1.3。旧环境的 `eiquadprog 1.3.2` 要求 Boost 1.90，与 Pinocchio 使用的 Boost 1.87 冲突；新环境固定兼容的 `eiquadprog 1.2.9`，并锁定 cmeel/URDFDOM 等原生依赖，避免只满足宽泛版本约束却加载到不匹配的 C++ 库。

默认环境不安装 R5、Torch/JAX、其他机器人驱动或完整 ROS。编译器、USB/CAN 工具、Quest PC Service、系统驱动仍属于系统安装项，不能仅靠 `uv.lock` 重建。独立官方 ARX 实验后端另需兼容的 KDL/ROS 系统库，见 [安装说明](../../docs/guides/setup.md)。

原 `xrobot312` Conda 环境未修改，保留作临时回退；默认启动配置不再指向它。不在同一个环境中交替使用 Conda/pip 和 uv 改写依赖。

π0.5 与 GR00T 的训练/推理使用各自环境，避免与控制环境的 NumPy、Torch/JAX 和原生库互相覆盖。OpenPI 与 GR00T 的受管源码工作区在 `outputs/dependencies/`，解释器安装在 `outputs/envs/`；参考 checkout 不用作安装目录。训练与推理必须匹配模型实现、数据 schema 和归一化配置，GPU 构建按各自硬件选择；权重和数据路径继续放在本机配置中。
