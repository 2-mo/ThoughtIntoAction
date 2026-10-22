# 项目目录职责

根目录放项目说明、源码版本锁、日常启动入口，以及按用途组织的目录。

```text
QuestArmTeleop/
├── README.md
├── .gitignore
├── pyproject.toml            Python 依赖声明与原生构建依赖
├── uv.lock                   Python 包版本锁
├── .python-version           uv 管理的 Python 版本
├── .venv/                    本机运行环境，不提交
├── dependencies.lock.toml    外部源码版本锁
├── run.sh                    统一命令入口
├── 启动机械臂服务.sh         独立设备通信，默认真机阻尼待机
├── 启动遥操.sh               唤醒 Quest 并启动 presets 遥操
├── 网页控制.sh               网页启停、状态与相机开关
├── src/                      按能力组织的 Python 实现
├── scripts/                  安装、设备准备、诊断和 Quest 工具
├── configs/                  共享默认、本机覆盖和命令档位
├── configs/environments/snapshots/   历史环境盘点
├── assets/                   当前模型与自有静态资产
├── third_party/              固定版本的外部源码和可选参考
├── docs/                     使用说明、迁移记录和历史验证
├── tests/                    不执行真机运动的回归测试
├── outputs/                  日志、标定、构建等生成文件，不提交
└── .cache/                   本机工作备份与缓存，不提交
```

`configs/` 描述如何运行。当前 Python 环境由根目录 `pyproject.toml`、`uv.lock` 和 `.python-version` 定义，实际安装在 `.venv/`；`configs/environments/` 保存环境边界说明和历史快照，将来的推理/训练环境也在此独立声明。原 Conda 环境保留在仓库外作临时回退，不是默认运行依赖。模型权重、数据集、安装包仍使用本机配置中的仓库外目录。

`run.sh` 直接调用 `src/` 模块或 `scripts/` 文件。普通命令的 `PYTHONPATH` 只有本项目 `src/` 和固定版本 XRoboToolkit；官方 X5 实验命令按需补充其 SDK 路径。项目维护当前入口和模块路径，不设旧接口兼容层。`myTeleop/`、旧根目录工具转发脚本与 `simulate-r5` 别名均已移除。

配置和公共路径统一由 `src/embodied/core/project.py` 读取，只依赖 Python 标准库。各包直接导入这个模块；旧配置读取模块和路径导出模块已移除。采集源码指纹覆盖 `src/` 和 `scripts/` 中的 Python 文件。

插件注册表在 `configs/plugins.toml`，加载器在 `robot_core/plugins.py`。实现仍归入所属能力包：机械臂后端在 `robot_control/backends/`，位姿输入在 `quest_teleop/inputs.py`，状态发布在 `robot_core/telemetry.py`，独立监看在 `robot_simulation/`。设备通信与控制客户端在 `robot_control/service.py` 和 `client.py`；控制进程不创建后端或场景，见 [插件说明](../reference/plugins.md)。

新增功能放置原则：可复用实现归入相应 `src` 包；安装、设备准备和单次诊断入口归入 `scripts`；三维监看归入 `robot_simulation`，虚拟控制 CLI 归入 `robot_control`，选择位姿输入后延迟加载 `quest_teleop` 的控制器；网页跟随所属功能包保存。

详细包职责见 [源码分层](../../src/README.md)，日常操作见 [命令说明](../reference/commands.md)，配置和环境分别见 [配置说明](../../configs/README.md) 与 [环境说明](../../configs/environments/README.md)。
