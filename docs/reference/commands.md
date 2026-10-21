# 命令

所有功能从仓库根目录的 `./run.sh` 进入，可在任意目录调用：

```bash
./run.sh                       # 列出全部功能与命令
./run.sh <功能>                # 只看一组
./run.sh <功能> <命令> [参数]  # 运行
```

`<功能> <命令>` 是下表的分组写法，每一条都对应一个现有的启动器命令、脚本或组件，参数原样传递。原有的单词命令（如 `arm-service`、`view-arm`）仍可直接使用，见文末“旧链路命令”。

## 命令目录

<!-- catalog:start -->

由 `python -m embodied.workstation.catalog` 从 `src/embodied/workstation/catalog.py` 生成，勿手改。

### sim · MuJoCo 仿真工作台（Quest 遥操 + arx 内核 + 采集 + 网页）

| 命令 | 说明 |
| --- | --- |
| `./run.sh sim start` | 启动并进入工作台；--detach 后台启动 |
| `./run.sh sim status` | 工作台各窗口状态 |
| `./run.sh sim stop` | 有序退出遥操与内核，保留网页和相机 |
| `./run.sh sim shutdown` | 关闭工作台启动的全部服务 |

### real · 真机工作台（当前日常链路：机械臂服务 + 遥操 + 网页）

| 命令 | 说明 |
| --- | --- |
| `./run.sh real start` | 启动并进入工作台；首次需要在窗口 1 输入 sudo 密码 |
| `./run.sh real status` | 工作台各窗口状态 |
| `./run.sh real stop` | 有序退出遥操与机械臂服务 |
| `./run.sh real shutdown` | 关闭工作台启动的全部服务 |

### kernel · arx-televr 控制内核（control/，唯一 SDK 所有者）

| 命令 | 说明 |
| --- | --- |
| `./run.sh kernel start` | 启动内核：--backend sim|real，--input quest|demo，--managed 接受多来源 |
| `./run.sh kernel real` | 真机 · 上游操控：准备 Quest USB 与 CAN 后启动（先确认安全，待验收） |
| `./run.sh kernel viewer` | 附加只读 MuJoCo 窗口 |
| `./run.sh kernel observe` | 读取运行中内核的遥测：--run <RUN> |
| `./run.sh kernel verify` | 核对上游文件与本地补丁台账 |
| `./run.sh kernel check` | 上游离线检查；--simulation 加跑 MuJoCo 链路 |
| `./run.sh kernel doctor` | 检查依赖与模型，不连接设备 |
| `./run.sh kernel clean` | 清理旧运行目录 control/run：[--keep 20] [--yes]；运行中的内核不受影响 |
| `./run.sh kernel setup` | 创建 Python 3.10 内核环境 |
| `./run.sh kernel build-sdk` | 编译带夹爪修正的 SDK |
| `./run.sh kernel profiles` | 工作台组合 profile：plugins | validate | start | status | stop |

### arm · 机械臂（真机）

| 命令 | 说明 |
| --- | --- |
| `./run.sh arm service` | 旧机械臂服务（准备 CAN、阻尼待机、交互控制台） |
| `./run.sh arm check` | 只读服务反馈；--virtual 读取虚拟服务 |
| `./run.sh arm home` | 回零 |
| `./run.sh arm can-up` | 配置 CAN 接口 |
| `./run.sh arm can-status` | CAN 接口状态 |
| `./run.sh arm setup-device` | 安装 USB-CAN udev 规则 |
| `./run.sh arm joint-test` | 关节检查，默认只读；--actuate 小幅动作 |
| `./run.sh arm state` | 直连 SDK 读取状态（需独占 CAN） |
| `./run.sh arm gripper-position` | 夹爪位置测试 |
| `./run.sh arm gripper-torque` | 夹爪力矩测试 |

### quest · Quest 头显与遥操

| 命令 | 说明 |
| --- | --- |
| `./run.sh quest teleop` | 唤醒 Quest 并启动遥操（启动遥操.sh） |
| `./run.sh quest check` | XR 数据检查 |
| `./run.sh quest service` | 启动 XRoboToolkit PC Service |
| `./run.sh quest usb` | 配置 USB 反向端口：prepare | status |
| `./run.sh quest discovery` | 头显发现广播：on | off | status |
| `./run.sh quest battery` | 头显电量 |
| `./run.sh quest wake` | 唤醒与模拟佩戴 |
| `./run.sh quest visualize` | 浏览器查看手柄位姿 |
| `./run.sh quest calibrate` | 手柄与工具方向标定 |

### phone · iPhone 遥操

| 命令 | 说明 |
| --- | --- |
| `./run.sh phone run` | 接入运行中的内核：--run <RUN> |
| `./run.sh phone tools` | 手机协议工具集 |
| `./run.sh phone monitor` | 查看手机位姿流 |
| `./run.sh phone simulate` | 模拟手机位姿流 |
| `./run.sh phone loopback` | 本机 UDP 回环 |
| `./run.sh phone network` | 网络连通与丢包 |
| `./run.sh phone latency` | 延迟报告 |

### motion · 动作（内核客户端）

| 命令 | 说明 |
| --- | --- |
| `./run.sh motion run` | --run <RUN> status | movej | movel | jog | gripper | teach | return | home | stop |
| `./run.sh motion console` | 浏览器动作编辑台 |

### collect · 数据采集

| 命令 | 说明 |
| --- | --- |
| `./run.sh collect run` | 从运行中的内核采集：--run <RUN> --task <任务> [--cameras wrist_rgb,exterior_rgb]（默认存 MP4） |
| `./run.sh collect session` | 旧链路采集会话：status | demo | ... |
| `./run.sh collect record` | 旧链路录制开始 / 结束 |
| `./run.sh collect groot` | GR00T 双相机采集 |

### data · 数据查看、回放与导出

| 命令 | 说明 |
| --- | --- |
| `./run.sh data review` | inspect | validate | label | replay；旧格式与内核 episode 都支持（replay 仅旧格式） |
| `./run.sh data runtime` | 内核 episode（schema 2）检查 |
| `./run.sh data export` | 导出 LeRobot v2.1（首次自动安装导出环境） |
| `./run.sh data export-setup` | 安装导出环境 |

### policy · 策略推理（真机）

| 命令 | 说明 |
| --- | --- |
| `./run.sh policy pi05` | π0.5 零样本 |
| `./run.sh policy kernel` | π0.5 经 arx 内核执行：--run <RUN> --prompt <任务> [--policy sft|zeroshot]（动作由内核统一限位） |
| `./run.sh policy pi05-sft` | π0.5 微调模型（PyTorch，旧机械臂服务） |
| `./run.sh policy pi05-sft-jax` | π0.5 微调模型（JAX） |
| `./run.sh policy groot` | GR00T 推理：--run <RUN> --task <任务> 经 arx 内核；不带 --run 为旧直连交互 |
| `./run.sh policy pregrasp` | 前往预抓取位 |
| `./run.sh policy stop` | 停止并恢复 |
| `./run.sh policy setup-pi05` | 安装零样本推理环境 |
| `./run.sh policy setup-sft` | 安装微调推理环境（PyTorch） |
| `./run.sh policy setup-sft-jax` | 安装微调推理环境（JAX） |

### calib · 标定

| 命令 | 说明 |
| --- | --- |
| `./run.sh calib handeye` | 手眼标定（经内核）：marker | capture <相机> [--auto] | solve <会话> | status |
| `./run.sh calib gripper` | 夹爪零点 |
| `./run.sh calib payload` | 负载辨识 |

### camera · 相机

| 命令 | 说明 |
| --- | --- |
| `./run.sh camera service` | 共享相机服务 |
| `./run.sh camera check` | 相机检查 |
| `./run.sh camera rgbd` | D405 / D455 RGB-D 与 WebRTC |
| `./run.sh camera d405` | D405 配置基准 |
| `./run.sh camera permissions` | 修复相机设备权限 |

### web · 网页

| 命令 | 说明 |
| --- | --- |
| `./run.sh web monitor` | 18081 监看页（含共享相机） |
| `./run.sh web control` | 监看页启停：start | stop | status | camera-on | camera-off |
| `./run.sh web console` | apps/console 控制台，接入运行中的内核：[--run <RUN>] |
| `./run.sh web legacy` | 旧控制台（直连 CAN；负载 / 夹爪标定与方块任务，内核化前使用） |
| `./run.sh web backend` | 控制台后端 |
| `./run.sh web vision` | 控制台手势识别服务（8765） |
| `./run.sh web build` | 构建控制台前端 |
| `./run.sh web preview` | 控制台前端预览 |
| `./run.sh web lan` | 局域网 HTTPS 发布 |
| `./run.sh web tailnet` | Tailscale 发布 |
| `./run.sh web site` | 本地预览宣传网站（site/） |

### task · 任务程序

| 命令 | 说明 |
| --- | --- |
| `./run.sh task cube` | 方块抓取演示（旧链路，直连 CAN） |

### check · 离线检查

| 命令 | 说明 |
| --- | --- |
| `./run.sh check all` | 全部检查组：--only <组> |
| `./run.sh check test` | 主环境 pytest |
| `./run.sh check deps` | 核对锁定的源码依赖 |
| `./run.sh check doctor` | 旧链路设备与环境诊断 |
| `./run.sh check ports` | 端口与 socket：登记、占用进程、冲突 |

### setup · 安装与设备准备

| 命令 | 说明 |
| --- | --- |
| `./run.sh setup deps` | 获取锁定的源码依赖 |
| `./run.sh setup env` | 创建组件环境：analysis | web | phone | rebot | frontend | groot |
| `./run.sh setup connections` | 准备 Quest USB 与 CAN |
| `./run.sh setup xr-sdk` | 重建 Quest 绑定 |

<!-- catalog:end -->

## 启动器选项

启动器选项写在功能或命令之前：

| 选项 | 作用 |
| --- | --- |
| `--print-command` | 只打印解析后的命令与环境，不执行，例如 `./run.sh --print-command arm check` |
| `--profile <名称>` | 加载 `configs/profiles/<名称>.toml` 中的参数组合；profile 不能开启实际动作，`--actuate` 必须显式给出 |
| `--config <文件>` | 使用另一个本机配置覆盖 `configs/local.toml` |

参数优先级：

- 服务参数：项目默认配置 → `configs/local.toml` → 命令行。
- 控制参数：控制器默认值 → profile → 命令行；控制端显式传入的速度覆盖服务默认速度。

环境由 `uv sync --locked` 同步；日常命令使用 `.venv/bin/python`，不会隐式安装或更新依赖。组件命令（`./run.sh components list`）使用各自声明的环境，首次运行前用 `./run.sh setup env <名称>` 创建。

## 旧链路命令

以下单词命令属于当前真机日常链路（`src/legacy/`），内核真机验收后随旧链路删除。

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
| `test` / `check-all` | `test` 在主环境运行 pytest；`check-all` 依次运行 control、platform、motion、phone、rebot、policy（OpenPI 变换）、export（LeRobot 导出与回归摘要）、browser、sim（MuJoCo 端到端）、groot（GR00T 导出）各组离线检查，端到端证据写入 `outputs/checks/<时间>/`，缺少环境的组显示 SKIP；`--only`、`--simulation`、`--print` |
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

历史自定义控制器的 CSV 日志迁到 `outputs/logs/`，可由 `paths.log_dir` 配置。主 `teleop-presets-x5` 增加 `--enable-session` 后会输出会话诊断日志，并接受独立终端的原始 episode 录制命令；见 [采集说明](../guides/recording.md)。未开启时保留原来的终端诊断。既有 TXT 记录保存在 `docs/history/validation/`。

源码位置见 [目录职责](../history/layout.md)。虚拟控制由 `legacy.robot_control.cli` 运行；选择位姿输入时组合主 presets 控制器。真机与虚拟控制均通过 `legacy.robot_control.client` 连接服务，由服务通过 `embodied.core.telemetry` 发布状态；独立监看在 `robot_simulation/cli.py` 和 `scene.py`；旧的继承式虚拟控制器和命令别名已移除。历史自定义控制器位于 `src/legacy/quest_teleop/legacy/`，提供独立实现。插件组合和接口见 [插件说明](plugins.md)。
