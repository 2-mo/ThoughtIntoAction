# 知行有灵 · Thought into Action

模块化具身智能平台。一个仓库覆盖遥操、运动控制、采集、导出、训练和策略执行。公共操作接口、运动算法、数据流程与设备驱动各自维护，机械臂、相机和控制来源按契约接入。设备实现包括 ARX X5-2025 与七轴无夹爪 Reference7 教学仿真，输入支持 Quest / iPhone，相机支持 RealSense，训练和策略包括 π0.5、GR00T。Reference7 验证不同轴数的插件接入，不是另一款真机；具体边界见[完整架构](docs/architecture/overview.md)。

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
| 编写分阶段任务、重复仿真测试 | `./run.sh examples approach_retreat`、`./run.sh examples sim_task` |
| 验证七轴无夹爪模型 | `./run.sh examples reference7` |
| 用反馈评估速度与跟踪质量 | `./run.sh examples benchmark --degrees 10 --speed 0` |
| 连续路径、任务与相机一起录制 | `./run.sh examples move_path`、`./run.sh examples record_task` |
| 查看后续工作 | [工程计划](docs/plans/README.md) |

## 安装

```bash
git clone https://github.com/2-mo/ThoughtIntoAction.git
cd ThoughtIntoAction
./run.sh setup deps        # 准备锁版本的正式外部依赖（outputs/dependencies）
uv sync --locked           # 平台环境（Python 3.12）
./run.sh kernel setup      # 内核环境（Python 3.10）
./run.sh check all         # 全部离线检查
```

详细安装见 [docs/guides/setup.md](docs/guides/setup.md)。

## 使用

**日常只需要 `./run.sh`**：打开统一工作台（tmux）。总控窗口列出当前可用的按键——启动真机或仿真内核，之后录制、检查、导出、网页控制台、手眼标定、策略都是一个键；每个操作都会把实际命令输入“命令”窗口，可以看到、修改、重复执行。

工作台固定打开 **1 总控、2 命令、3 robot、4 网页、5 VR、6 相机、7 宣传、8 手机** 八个终端窗口，用 `Ctrl+B` 后按数字切换。默认打开带采集面板的监看页（<http://127.0.0.1:18081/>），读取统一内核反馈，并启动宣传网站、独立共享相机及手机接收服务；相机和手机输入不依赖 robot 启动。相机窗口按 `R` 可重试启动，已有共享服务直接复用，关闭相机窗口不会中断其他采集或标定对共享服务的使用。VR 未连接、机械臂未通电或相机未就绪时，对应窗口保留状态与操作提示，设备接入不会自动使能机械臂。网页启动失败会显示日志，按 `R` 重试。会话名为 `emb`，可用 `tmux attach -t emb` 返回。再次运行 `./run.sh` 会补齐缺少的窗口，并在界面版本变化时重载总控、等待页和网页服务，保留运行中的内核与命令。旧会话自动改名并调整窗口顺序。若名称已被其他工作区占用，用 `EMBODIED_BENCH_SESSION=其他名称 ./run.sh` 指定会话。

**手机遥操在窗口 8 或网页操作**：RobotTeleop 填窗口显示的电脑地址、UDP 5005。接收服务持续等待，手机晚打开也可连接；两处共享在线、ARKit 追踪、输入帧率、数据年龄、Clutch 和控制权状态。正常追踪且松开 Clutch 后，在手机点“开始遥操”、按 `T` 或网页“启用手机遥操”接管当前 robot；按住手机底部控制区跟随，上滑开夹爪、下滑闭夹爪，松开保持。默认 1:1 相对控制，追踪受限 / 丢失时整张卡片高亮。手机“恢复”清除手机停止锁定或重新连接，之后再点开始。`S` / 空格或“停止遥操”释放控制权，继续监看。刷新网页和终端不重新接管。数据年龄是最新有效收包距今，不是端到端延迟。App 更新和签名信任步骤见 [iOS 安装说明](apps/ios/README.md)。

**真机测试也只从 `./run.sh` 开始**：在交互终端打开工作台时，空闲的 robot 窗口会直接准备无 Quest 的真机启动流程，只需在该窗口确认一次，就绪后 `Ctrl+B → 1` 返回总控。已有内核或启动确认会原样保留；后台 `./run.sh bench start --detach` 和非交互调用不会准备真机启动。取消确认后可在总控按 `5` 使用仿真，按 `1 / 2 / 3` 另选真机配置。启动失败时原始错误保留在 robot 窗口，按回车返回总控；修复后按 `3` 重试，不会自动反复启动。按 `M` 读取状态与能力，`B` 填入关节 1 的 10° 往返测速，`X` 填入基座 Z 方向 +2 cm 的 MoveL，`T` 填入 Z +2 cm 再 Y +2 cm 的圆角路径。运行目录与后端自动填写，动作在窗口 2 按回车后开始；测试中 `Ctrl+C` 取消动作，完成后返回总控。测速原始数据与报告保存在 `outputs/benchmarks/`。网页自动跟随内核；本机桌面在网页就绪后请求打开浏览器，也可按 `W` 打开，SSH 使用网页窗口显示的地址。

总控按“图标 / 设备 / 状态”分列，使用高亮青色的单色符号：`◇` robot、`◉` VR、`▣` 相机、`↔` USB-CAN、`▤` 服务，不依赖彩色 emoji；名称与彩色状态独立对齐，对应设备窗口提供详情。用彩色圆点和文字提示：黄色表示等待启动确认、连接、反馈或许可，绿色表示有效反馈 / 跟踪就绪 / 图像正常，红色表示启动失败、读取失败或数据过期，灰色表示当前配置未启用。USB-CAN 可见不等于机械臂上线，相机 USB 可见不等于正在出图；仿真反馈会明确标注。更换相机后，在 `configs/local.toml` 按腕部、外部位置更新序列号，避免服务仍然查找旧设备。

**robot 窗口直接按 `P` 前往预抓取位、`H` 回零位，无需回车；空格 / `S` 取消本面板动作并保持。** 按键使用统一内核的 MoveJ，所有默认运动参数集中在 [`configs/control/motion.json`](configs/control/motion.json)，`speed=0` 使用后端每轴速度上限；加速度、jerk 和到位判定也在此配置，短行程的实际速度可能达不到速度上限。运动中不排队重复按键，界面显示关节位置、实时速度、目标误差、规划时长、限制来源与实际到位 / 失败结果。按 `D` 提示扶住机械臂，5 秒内再按 `D` 退出真机内核进入 SDK 阻尼；恢复控制需在总控重新启动 robot。预抓取使用 ARX 已有公共位姿，夹爪宽度保持。启动确认完成后自动显示统一状态面板；原内核进程和终端保留在同一窗口，`Ctrl+B z` 展开查看原始日志，再按一次恢复面板。终端日志保存在 `outputs/logs/bench/robot-pane-*.log`，控制组件日志位于面板显示的运行目录。刷新界面不会重启电机；关闭面板时取消它自己尚未完成的动作，内核继续运行。

VR 可以晚于工作台开机。窗口 5 持续检查主机接收服务、USB 授权、头显开机、端口转发、客户端和 TCP 连接；开机并授权后自动补齐转发、打开客户端，拔插后继续恢复，失败按 2–30 秒退避重试，`R` 可立即重试。检查和准备在后台进行，不阻塞总控按键。首次 USB 调试仍需在头显勾选“始终允许这台电脑”；电脑不能自行批准。官方 Quest 客户端仍需手动填 `127.0.0.1` 并 Connect；免输入、免点击需要一次性安装[自动连接版客户端](docs/guides/quest-client.md)。连接状态与 robot 是否接入 Quest 分开显示，准备 VR 不会切换现有内核或取得机械臂控制权。

**重启在总控按 `R`**：`1` 重启网页、宣传站和终端状态界面，保留 robot、共享相机与命令窗口；`2` 沿用本次启动配置重启 robot，真机在命令窗口确认一次；`3` 重启共享相机服务并重新读取配置，录制中会拒绝。`R` 或 `Esc` 返回菜单。也可使用 `./run.sh bench restart --target interfaces`；`--target` 还可选择 `robot` 或 `camera`；robot 必须由当前工作台启动并有启动命令记录，不会猜测外部内核的配置。

新版监看页提供“实测速度 · 质量”和“任务 / 技能复核”：查看反馈速度、周期、采集心跳、帧计数和错误，自动更新任务阶段。模型与相机来自当前运行描述，可显示七轴无夹爪模型。采集数据按机器人关节与夹爪契约保存，可选择外部相机作为参考；程序任务保留 scripted 来源并生成待复核标注，见[采集与复核](docs/guides/recording.md)。

关节速度图保留原始反馈淡线，并叠加时间常数为 120 ms 的平滑趋势线；瞬时数值、指令和采集数据不滤波。曲线连续滚动，悬停可看最近 5 秒网页已收到样本的原始峰值，超范围提示也依据原始数据。网页抽样用于监看，控制周期和完整测速请使用总控的测速报告。

通过 SSH 打开工作台时，监看页自动绑定 SSH 连接使用的服务器地址，**以窗口 4 显示的链接为准**；客户端的 `127.0.0.1` 不指向远程机器。SSH 或无桌面环境下，内核使用 `--headless`，网页提供三维反馈，不依赖 tmux 中可能失效的 X11 显示连接；桌面 MuJoCo 视图可在有效的桌面会话中用 `./run.sh kernel viewer --run <RUN>` 单独附加。也可在 `configs/local.toml` 的 `[viewer_service]` 中指定 `host`，或手动启动时使用 `--host <服务器 IPv4>`。

`./run.sh help`（或 `-h`、`--help`）显示精简任务导航；`./run.sh help data` 或 `./run.sh data --help` 只看一组，`./run.sh search <关键词>` 搜索中文说明和命令名，`./run.sh help --all` 展开完整目录。帮助在终端使用 Rich 排版，输出到管道或未安装 Rich 时自动使用纯文本。

终端首页按任务列出常用入口；没有 tmux 时显示可复制的命令。`./run.sh <功能>` 也可查看一组命令，原有的单词命令仍可直接使用。

| 要做的事 | 命令 |
| --- | --- |
| 仿真里用 Quest 遥操并采集 | `./run.sh sim start`，网页 <http://127.0.0.1:18081/> |
| 真机遥操并采集（当前日常链路） | `./run.sh real start` |
| 只启动内核 | `./run.sh kernel start --backend sim --input quest` |
| 手机接入运行中的内核 | 工作台窗口 `8` 按 `T`，或网页“启用手机遥操” |
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
outputs/          数据、运行记录、安装环境、依赖工作区与构建产物
```

根目录只保留 `run.sh` 作为启动入口。原来的中文脚本移动到 [scripts/shortcuts/](scripts/shortcuts/)，文件名和参数保留；例如 `bash scripts/shortcuts/启动遥操.sh`。重构待办移至 [docs/plans/](docs/plans/README.md)。

`src/embodied/control/` 维护公共控制运行时：网关、动作调度、周期执行与只读观测。硬件实现按设备归属：ARX 的 profile、真机/仿真后端、运动学与 SDK 资源在 `devices/robots/arx5/`；Quest 原生输入与路由在 `devices/inputs/quest/`。公共运动算法仍在 `motion/`，不为每台机械臂复制一份。具体职责见 [源码导航](src/README.md)。

插件统一登记在 `configs/plugins.json`，实现只有被选中时才加载。机器人参数在 `configs/robots/`，输入参数在 `configs/inputs/`，控制策略参数在 `configs/control/`。控制环境定义在 `configs/environments/control/`，安装结果在 `outputs/envs/control/`，运行记录在 `outputs/runtime/control/`；Python 版本隔离不再要求根目录保留一个独立控制工程。

`third_party/` 是 Git 忽略的可选本地参考目录，仅用于阅读、比较和来源核对；来源与迁移记录保存在 `docs/provenance/`。正式依赖按锁文件安装；需要维护的上游片段随所属模块保存来源、版本和许可证，构建工作区在 `outputs/dependencies/`。参见 [外部参考边界](docs/provenance/README.md) 与 [PhyAgentOS 参考取舍](docs/architecture/phyagent-reference.md)。

本机采集数据、标定结果和原始归档集中在 `outputs/{datasets,calibrations,archives}/`。原 `environments/` 移到 [configs/environments/](configs/environments/README.md)，原 `integrations/openpi/` 移到 `scripts/training/openpi/`，历史 rebot 示例在 `docs/examples/`；统一内核教学示例在 `src/examples/`。依赖版本仍由根目录 `pyproject.toml`、`uv.lock` 和 `dependencies.lock.toml` 管理。

提交约定：文档、网页、代码示例、报告、日志、截图及提交说明均不包含本机显卡的具体型号、硬件架构、UUID、序列号、驱动信息或显存统计。软件依赖与运行参数按需记录；原始诊断和实验输出放在 Git 忽略的 `outputs/` 下，分享前检查并去除硬件信息。

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
