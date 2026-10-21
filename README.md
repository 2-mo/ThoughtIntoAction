# 具身智能平台（ARX X5）

一个仓库覆盖遥操、采集、导出、训练和策略执行：Quest / iPhone 遥操 ARX X5-2025，采集双相机数据，导出 LeRobot / GR00T 数据集，在 H800 上训练，再回到机械臂执行 π0.5 或 GR00T 策略。MuJoCo 仿真与真机使用同一个控制内核。

## 组成

```text
control/          控制内核：arx-televr（上游 GZF123/arx-televr），250 Hz worker、IK、MuJoCo / 真机后端，
                  唯一的机械臂 SDK 所有者；gateway 统一处理控制权、限位与工作空间
src/embodied/     平台：输入、动作、相机、采集、数据导出、策略、标定、监看、控制台、工作台
src/legacy/       已被内核取代的旧实现（当前真机日常链路），真机验收后删除
apps/console      浏览器控制台（Vite + TypeScript）；apps/ios 为手机遥操 App
configs/          平台配置；机器人参数在 control/config/
todo/             平台重构计划与冗余清单
```

源码结构见 [src/README.md](src/README.md)。

## 开始

```bash
./run.sh setup deps        # 获取锁定的源码依赖
uv sync --locked           # 平台环境（Python 3.12）
./run.sh kernel setup      # 内核环境（Python 3.10）
./run.sh check all         # 全部离线检查
```

详细安装见 [docs/guides/setup.md](docs/guides/setup.md)。

## 使用

所有功能都从 `./run.sh <功能> <命令>` 进入；`./run.sh` 列出全部命令，`./run.sh <功能>` 只看一组。

| 要做的事 | 命令 |
| --- | --- |
| 仿真里用 Quest 遥操并采集 | `./run.sh sim start`，网页 <http://127.0.0.1:18081/> |
| 真机遥操并采集（当前日常链路） | `./run.sh real start` |
| 只启动内核 | `./run.sh kernel start --backend sim --input quest` |
| 手机接入运行中的内核 | `./run.sh phone run --run <RUN>` |
| 点动、MoveJ / MoveL、示教点 | `./run.sh motion run --run <RUN> status` |
| 查看、回放、标注数据 | `./run.sh data review` |
| 导出 LeRobot 数据集 | `./run.sh data export --source <目录>` |
| π0.5 / GR00T 推理 | `./run.sh policy pi05-sft`、`./run.sh policy groot` |
| 手眼、夹爪、负载标定 | `./run.sh calib wrist`、`calib handeye`、`calib gripper`、`calib payload` |
| 相机服务与检查 | `./run.sh camera service`、`camera check` |
| 浏览器控制台（接入运行中的内核） | `./run.sh web console`；负载 / 夹爪标定与方块任务暂用 `web legacy` |
| 宣传网站（本地预览） | `./run.sh web site`，或直接打开 `site/index.html` |

`<RUN>` 是内核启动时打印的 `control/run/<目录>`。

## 手柄

右手柄按 arx-televr 上游：A 按住至少 150 ms 后松开切换模式（首次进入跟随，之后在 FreeDrive 与跟随间切换），A+B 回 Home，Grip 跟随，Trigger 控制夹爪。左手柄负责采集：X 录制 / 保存，Y 预抓取（长按作废当前录制），左摇杆管理任务，左 Grip 取消预设动作。以上为仿真工作台的按键；真机日常链路的按键见 [日常操作](docs/guides/operation.md)。

## 安全

- 安全控制统一在内核：关节限位、桌面平面、速度与反馈检查对所有经 gateway 的来源生效，客户端不各自实现。
- 内核真机启动会使能电机并回 Home；退出进入 SDK 阻尼，退出前先支撑机械臂。
- 旧机械臂服务启动时进入阻尼待机，不自动回零；阻尼不保证位置保持。

## 现状

- 仿真：遥操、采集、导出、手机、动作、标定适配均经过内核，`./run.sh check all` 的 MuJoCo 端到端检查覆盖这些链路。
- 真机：日常仍走 `src/legacy/` 中的旧机械臂服务；内核真机入口 `./run.sh kernel real` 已具备，按 [真机验收清单](docs/guides/real-arm-acceptance.md) 验收后切换（`todo/01-control-kernel.md` K-08、K-09）。
- 推进计划见 [todo/](todo/README.md)；日常操作细节见 [docs/guides/operation.md](docs/guides/operation.md)；命令说明见 [docs/reference/commands.md](docs/reference/commands.md)。
