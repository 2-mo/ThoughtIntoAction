# 模块化具身智能平台

一个仓库覆盖遥操、运动控制、采集、导出、训练和策略执行。公共操作接口、运动算法、数据流程与设备驱动各自维护，机械臂、相机和控制来源按契约接入；当前贯通的机械臂是 ARX X5-2025，搭配 Quest / iPhone、RealSense 与 MuJoCo，训练和策略包括 π0.5、GR00T。其他机械臂仍需模型与后端适配，具体边界见[完整架构](docs/architecture/overview.md)。

## 从这里开始

| 要做什么 | 入口 |
| --- | --- |
| 安装环境、第一次仿真 | [入门](docs/getting-started.md) · [安装](docs/guides/setup.md) |
| 打开仿真窗口，学习关节 / XYZ / 夹爪控制 | `./run.sh examples sim_control` · [仿真与真机接口](src/examples/README.md) |
| 日常遥操、采集和管理服务 | `./run.sh` 打开工作台 · [日常操作](docs/guides/operation.md) |
| 找命令或参数 | `./run.sh help` · `./run.sh help data` · `./run.sh search 标定` |
| 了解平台、演示给别人看 | `./run.sh web site` · [宣传站](apps/site/README.md) |
| 理解代码、找实现 | [架构概览](docs/architecture/overview.md) · [源码导航](src/README.md) |
| 学习如何写运动与采集程序 | `./run.sh examples` · [Python 示例](src/examples/README.md) · [Robot 接口](docs/reference/robot-primitives.md) |
| 查看后续工作 | [工程计划](docs/plans/README.md) |

## 安装

```bash
./run.sh setup deps        # 准备锁版本的正式外部依赖（outputs/dependencies）
uv sync --locked           # 平台环境（Python 3.12）
./run.sh kernel setup      # 内核环境（Python 3.10）
./run.sh check all         # 全部离线检查
```

详细安装见 [docs/guides/setup.md](docs/guides/setup.md)。

## 使用

**日常只需要 `./run.sh`**：打开统一工作台（tmux）。总控窗口列出当前可用的按键——启动真机或仿真内核，之后录制、检查、导出、网页控制台、手眼标定、策略都是一个键；每个操作都会把实际命令输入“命令”窗口，可以看到、修改、重复执行。

工作台固定打开 **1 内核、2 命令、3 总控、4 网页、5 VR、6 相机、7 宣传** 七个终端窗口，用 `Ctrl+B` 后按数字切换。默认打开带采集面板的监看页（<http://127.0.0.1:18081/>），只读连接统一内核，并启动宣传网站。VR 未连接、机械臂未通电或相机未就绪时，对应窗口保留状态与操作提示，设备接入不会自动使能机械臂。网页启动失败会显示日志，按 `R` 重试。再次运行 `./run.sh` 会补齐缺少的窗口，保留已有进程。

通过 SSH 打开工作台时，监看页自动绑定 SSH 连接使用的服务器地址，**以窗口 4 显示的链接为准**；客户端的 `127.0.0.1` 不指向远程机器。也可在 `configs/local.toml` 的 `[viewer_service]` 中指定 `host`，或手动启动时使用 `--host <服务器 IPv4>`。

`./run.sh help`（或 `-h`、`--help`）显示精简任务导航；`./run.sh help data` 或 `./run.sh data --help` 只看一组，`./run.sh search <关键词>` 搜索中文说明和命令名，`./run.sh help --all` 展开完整目录。帮助在终端使用 Rich 排版，输出到管道或未安装 Rich 时自动使用纯文本。

终端首页按任务列出常用入口；没有 tmux 时显示可复制的命令。`./run.sh <功能>` 也可查看一组命令，原有的单词命令仍可直接使用。

| 要做的事 | 命令 |
| --- | --- |
| 仿真里用 Quest 遥操并采集 | `./run.sh sim start`，网页 <http://127.0.0.1:18081/> |
| 真机遥操并采集（当前日常链路） | `./run.sh real start` |
| 只启动内核 | `./run.sh kernel start --backend sim --input quest` |
| 手机接入运行中的内核 | `./run.sh phone run --run <RUN>` |
| 点动、MoveJ / MoveL、示教点 | `./run.sh motion run --run <RUN> status` |
| 写第一个关节 / XYZ 动作 | `./run.sh examples movej`、`./run.sh examples movel`（自动使用仿真） |
| 查看、回放、标注数据 | `./run.sh data review` |
| 导出 LeRobot 数据集 | `./run.sh data export --source <目录>` |
| π0.5 / GR00T 推理 | `./run.sh policy pi05-sft`、`./run.sh policy groot` |
| 手眼标定（经内核，每个相机一个结果） | `./run.sh calib handeye capture <相机> --auto`，见 [手眼标定](docs/guides/calibration.md) |
| 夹爪、负载标定（旧链路） | `./run.sh calib gripper`、`calib payload` |
| 相机服务与检查 | `./run.sh camera service`、`camera check` |
| 新版监看与采集面板（工作台默认） | `./run.sh web monitor --kernel --no-camera --no-connections`；只读连接统一内核 |
| 浏览器控制台（接入运行中的内核） | `./run.sh web console`；负载 / 夹爪标定与方块任务暂用 `web legacy` |
| 宣传网站（本地预览） | `./run.sh web site`，或直接打开 `apps/site/index.html` |

`<RUN>` 是内核启动时打印的 `outputs/runtime/control/<目录>`。

## 目录

```text
src/              embodied 平台代码、examples 教学示例与 legacy 旧真机链路
apps/             用户界面：console 浏览器控制台、ios 手机 App、site 宣传站
assets/           模型、场景及本地 references 参考素材
configs/          平台配置、插件目录、设备参数与 environments 环境声明
docs/             指南、架构、examples 示例、plans 工程计划与 history 历史记录
scripts/          按用途组织的工具；shortcuts 快捷入口、training 训练适配
tests/            按功能组织的验证
third_party/      只读参考项目与历史迁移索引，不作为运行依赖
outputs/          数据、运行记录、安装环境、依赖工作区与构建产物
```

根目录只保留 `run.sh` 作为启动入口。原来的中文脚本移动到 [scripts/shortcuts/](scripts/shortcuts/)，文件名和参数保留；例如 `bash scripts/shortcuts/启动遥操.sh`。重构待办移至 [docs/plans/](docs/plans/README.md)。

`src/embodied/control/` 维护公共控制运行时：网关、动作调度、周期执行与只读观测。硬件实现按设备归属：ARX 的 profile、真机/仿真后端、运动学与 SDK 资源在 `devices/robots/arx5/`；Quest 原生输入与路由在 `devices/inputs/quest/`。公共运动算法仍在 `motion/`，不为每台机械臂复制一份。具体职责见 [源码导航](src/README.md)。

插件统一登记在 `configs/plugins.json`，实现只有被选中时才加载。机器人参数在 `configs/robots/`，输入参数在 `configs/inputs/`，控制策略参数在 `configs/control/`。控制环境定义在 `configs/environments/control/`，安装结果在 `outputs/envs/control/`，运行记录在 `outputs/runtime/control/`；Python 版本隔离不再要求根目录保留一个独立控制工程。

`third_party/` 仅用于阅读、比较和来源核对。正式依赖按锁文件安装；需要维护的上游片段随所属模块保存来源、版本和许可证，构建工作区在 `outputs/dependencies/`。参见 [外部参考边界](third_party/README.md) 与 [PhyAgentOS 参考取舍](docs/architecture/phyagent-reference.md)。

本机采集数据、标定结果和原始归档集中在 `outputs/{datasets,calibrations,archives}/`。原 `environments/` 移到 [configs/environments/](configs/environments/README.md)，原 `integrations/openpi/` 移到 `scripts/training/openpi/`，历史 rebot 示例在 `docs/examples/`；统一内核教学示例在 `src/examples/`。依赖版本仍由根目录 `pyproject.toml`、`uv.lock` 和 `dependencies.lock.toml` 管理。

## 手柄

右手柄按 arx-televr 上游：A 按住至少 150 ms 后松开切换模式（首次进入跟随，之后在 FreeDrive 与跟随间切换），A+B 回 Home，Grip 跟随，Trigger 控制夹爪。左手柄负责采集：X 录制 / 保存，Y 预抓取（长按作废当前录制），左摇杆管理任务，左 Grip 取消预设动作。以上为仿真工作台的按键；真机日常链路的按键见 [日常操作](docs/guides/operation.md)。

## 安全

- 安全控制统一在内核：关节限位、桌面平面、速度与反馈检查对所有经 gateway 的来源生效，客户端不各自实现。
- 内核真机原生 Quest 模式启动会使能电机、回 Home 并打开夹爪；managed 模式先保持启动时的位置与夹爪。退出均进入 SDK 阻尼，退出前先支撑机械臂。
- 旧机械臂服务启动时进入阻尼待机，不自动回零；阻尼不保证位置保持。

## 现状

- 仿真：遥操、采集、导出、手机、动作、标定适配均经过内核，`./run.sh check all` 的 MuJoCo 端到端检查覆盖这些链路。
- 真机：日常仍走 `src/legacy/` 中的旧机械臂服务；内核真机入口 `./run.sh kernel real` 已具备，按 [真机验收清单](docs/guides/real-arm-acceptance.md) 验收后切换（[内核计划](docs/plans/01-control-kernel.md) K-08、K-09）。
- 推进计划见 [docs/plans/](docs/plans/README.md)；日常操作细节见 [docs/guides/operation.md](docs/guides/operation.md)；命令说明见 [docs/reference/commands.md](docs/reference/commands.md)。
