# 知行有灵 · Thought into Action

模块化具身智能平台：一个控制内核 + 一个网页控制台，覆盖设备接入、遥操、数据采集和远程策略执行。
机械臂支持 ARX X5（单臂 / 双臂）与 RealMan RM65（单臂 / 双臂），相机为 RealSense，策略服务是远程 π0.5。

本文件只讲**日常要做的事**；实现细节、协议和排障都在分支文档里，索引见 [docs/README.md](docs/README.md)。

## 快速开始

首次安装：

```bash
./run.sh setup deps        # 锁定的源码依赖
uv sync --locked           # 平台环境（Python 3.12）
./run.sh kernel setup      # 内核环境（Python 3.10）
```

启动控制台：

```bash
./run.sh web console --host <本机IP>
```

浏览器打开 `http://<本机IP>:8000`。四个页面就是全部日常操作，下面的内核、遥操、采集、策略都在网页里完成。

## 四个页面

| 页面 | 你要做的事 |
| --- | --- |
| **总览** | 「接入配置」里选机械臂（ARX，单臂 / 双臂）、相机和相机↔机械臂绑定，保存后按提示重启；「系统电源」里启动 / 停止控制内核。首次使用先在这里配置并启动内核。 |
| **控制台** | 查看关节与三维画面、点动、MoveJ / MoveL、预抓取与回参考位；Quest 或手机遥操也在这里接管。 |
| **数据采集** | 填任务描述 → 开始采集 → 停止并保存；顶部进度按当前任务统计，已录列表可回看时长、删除。 |
| **策略** | 顶部选模型卡片（可一键挂载 / 卸载），点卡片自动填入 prompt 和服务端地址，然后「运行策略」；旁边有准备位 / 回参考位。 |

## 支持的设备

| | 说明 |
| --- | --- |
| 机械臂 | ARX X5（单臂 / 双臂）、RealMan RM65（单臂 / 双臂） |
| 相机 | RealSense：右腕 / 左腕 / 外部固定，可在接入配置里改序列号和绑定 |
| 输入 | Quest 头显、iPhone、网页控制台 |

ARX 在总览页「接入配置」里改单臂 / 双臂、相机与绑定；RealMan 目前用工作台 profile 启动
（`./run.sh realman quest` / `quest-dual`，见 [docs/realman/README.md](docs/realman/README.md)），
尚未出现在接入配置页。两者共用同一套内核环境，切换不需要重装环境。

## 数据与日志

| 位置 | 内容 |
| --- | --- |
| `outputs/datasets/` | 采集的 episode（每条含 mcap、三路视频与元数据） |
| `outputs/policy_remote/` | 每次策略运行的 `commands.jsonl` 与 `summary.json` |
| `outputs/logs/models/` | 模型挂载日志 |
| `outputs/runtime/control/` | 内核运行目录（`<RUN>`） |

## 安全

- 真机启动会使能电机并保持当前位置；停止或退出进入阻尼，**退出前先支撑机械臂**。
- 规划类动作（回参考位、准备位、MoveJ / MoveL）受桌面平面约束；流式的关节遥操与策略执行不做桌面平面检查，与原生遥操一致。
- 关节限位、夹爪宽度和每拍步长始终由内核统一裁剪，客户端不各自实现。

## 文档

| 先读 | |
| --- | --- |
| [文档索引](docs/README.md) | 全部指南、参考与计划的入口 |
| [控制台运行手册](docs/guides/console-runbook.md) | 启动顺序、网页点击流程与排障 |
| [接入配置](docs/guides/device-setup.md) | 机械臂 / 相机配置、厂商接入与重启语义 |
| [采集](docs/guides/recording.md) | 采集按键、episode 结构与校验 |
| [远程 π0.5 策略](docs/guides/policy-remote.md) | 服务端命令、协议、策略页与模型卡片 |
| [RealMan RM65](docs/realman/README.md) | 驱动、真机接入、仿真与复测 |
| [架构概览](docs/architecture/overview.md) · [内核协议](docs/reference/kernel-protocol.md) | 分层、进程与遥测协议 |

## 目录

```text
src/      平台代码：控制内核、设备驱动、数据与策略
apps/     用户界面：console 控制台、ios 手机 App、site 宣传站
configs/  插件登记、设备参数、工作台 profile、模型登记表
docs/     指南、架构、参考与计划
scripts/  启动、标定、数据与训练工具
tests/    按功能组织的验证
outputs/  数据、运行记录、日志与安装环境（不进 Git）
```

根目录只保留 `run.sh` 作为启动入口；所有子命令都能用 `./run.sh help` 或 `./run.sh help <功能>` 查到。

提交约定：文档、截图、日志与提交说明不包含本机显卡型号、序列号等硬件信息；原始诊断输出放在 Git 忽略的 `outputs/` 下。
