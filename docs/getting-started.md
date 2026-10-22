# 入门

平台运行在 Ubuntu x86_64。Python 环境由 [uv](https://docs.astral.sh/uv/) 管理，不需要激活；所有命令从仓库根目录的 `./run.sh` 进入，可以从任意目录调用。

## 1. 安装

```bash
./run.sh setup deps        # 获取 dependencies.lock.toml 锁定的源码依赖
uv sync --locked           # 平台环境：Python 3.12，安装到 .venv/
./run.sh kernel setup      # 控制内核环境：Python 3.10，安装到 outputs/envs/control/
./run.sh check all         # 离线检查，不连接任何设备
```

按需再装：

| 用途 | 命令 |
| --- | --- |
| 带夹爪修正的 ARX SDK（真机内核） | `./run.sh kernel build-sdk` |
| 标定、规划、网页后端 | `./run.sh setup env analysis`、`./run.sh setup env web` |
| 手机工具 | `./run.sh setup env phone` |
| 浏览器控制台前端 | `./run.sh setup env frontend` |
| LeRobot 导出 | `./run.sh data export-setup` |
| π0.5 推理 | `./run.sh policy setup-pi05`、`policy setup-sft` |

驱动、udev 规则、相机与 Quest 的设备准备见 [安装](guides/setup.md)。

## 2. 统一工作台

```bash
./run.sh
```

打开一个 tmux 会话：窗口 1 内核、2 命令、3 总控、4 网页（`Ctrl+B` 再按数字切换，`Ctrl+B D` 离开，全部继续运行）。总控里直接按键：

| 键 | 没有内核时 | 内核运行时 |
| --- | --- | --- |
| 1 – 5 | 启动内核：真机 Quest 直控 / 真机 Quest + 平台客户端 / 真机无 Quest / 仿真 Quest / 仿真无输入 | — |
| V O | — | MuJoCo 视图；遥测一帧 |
| C K E | — | 采集 20 s（双相机）；检查最新 episode；导出 LeRobot |
| W M | — | 网页控制台；动作命令 |
| H J | — | 手眼标定：腕部 / 固定相机 |
| G | — | π0.5 策略 |
| S | — | 停止内核（按两次；真机进入阻尼，先扶住机械臂） |
| P Q Z | 端口；离开；关闭工作台 | 同左 |

需要输入任务名或较长的操作（采集、导出、标定、策略）只把命令输入命令窗口，确认后回车；其余直接执行。网页、动作、标定、策略需要带命令网关的内核（2、3、5）。

没有 tmux 时，`./run.sh` 显示常用任务和环境提示，也可以直接运行分组命令。总控按 `?` 查看操作帮助。查找命令无需进入工作台：

```bash
./run.sh help data             # 只看数据功能
./run.sh search 标定           # 搜索中文说明和命令名
./run.sh help                  # 完整命令目录
```

## 3. 第一次仿真（18081 网页工作台）

不需要机械臂和相机。

```bash
./run.sh sim start
```

打开 <http://127.0.0.1:18081/>。戴上 Quest 后：

1. 松开所有按键、Grip 和 Trigger，输入开始被接收。
2. 右手 A 按住至少 150 ms 后松开，进入跟随；握住 Grip，机械臂跟随手柄移动；Trigger 控制夹爪。
3. 左摇杆左推开始任务，左手 X 前往预抓取位并开始录制，再按 X 保存。

没有头显时，可以用演示输入单独运行内核：

```bash
./run.sh kernel start --backend sim --input demo --headless --duration 12
```

浏览器控制台（`apps/console`）接入运行中的内核，页面上点“连接”；关节、末端 XYZ、回参考位、夹爪和手势遥操都经内核执行，关闭页面只释放控制，内核保持当前位置：

```bash
./run.sh web console        # http://127.0.0.1:8000
```

## 4. 第一次真机

经 arx 内核（验收中）：`./run.sh` 打开统一工作台，总控按 1（Quest 直控）或 2 / 3（带命令网关）。启动会使能并回 Home，先扶好机械臂；验收步骤见 [真机验收](guides/real-arm-acceptance.md)。

旧链路（验收前的日常方式，`src/legacy/`）：

```bash
./run.sh real start
```

窗口 1 是机械臂服务（首次需要输入 sudo 密码配置 CAN），窗口 2 是遥操，窗口 3 是网页。机械臂服务启动后进入阻尼待机，不会自动回零；阻尼不保证保持位置，先支撑机械臂。总控按键见 [日常操作](guides/operation.md)。

## 5. 从数据到策略

```text
遥操采集 ──▶ episode（MCAP + 视频）──▶ LeRobot v2.1 ──▶ H800 训练 ──▶ checkpoint ──▶ 机械臂执行
 sim / real      data review            data export        scripts/training/openpi    policy pi05-sft
```

```bash
./run.sh data review --help                # 查看、检查、标注、回放
./run.sh data export --source <episodes>   # 导出到新的 LeRobot 数据集目录
./run.sh policy pi05-sft --help            # 用微调模型控制机械臂
```

导出会在 `meta/quest_export.json` 记录每条数据的控制栈、相机集合和时间缺口，推理时据此提示相机是否一致。细节见 [训练数据导出](guides/openpi-dataset.md)。

## 6. 接下来

- 全部命令：`./run.sh help`；按关键词查找：`./run.sh search <关键词>`
- 宣传与演示：`./run.sh web site`，用浏览器打开终端打印的地址
- 架构：[架构概览](architecture/overview.md)
- 推进计划：[docs/plans/](plans/README.md)
