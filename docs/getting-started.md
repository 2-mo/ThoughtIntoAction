# 入门

平台运行在 Ubuntu x86_64。Python 环境由 [uv](https://docs.astral.sh/uv/) 管理，不需要激活；所有命令从仓库根目录的 `./run.sh` 进入，可以从任意目录调用。

## 1. 安装

```bash
./run.sh setup deps        # 获取 dependencies.lock.toml 锁定的源码依赖
uv sync --locked           # 平台环境：Python 3.12，安装到 .venv/
./run.sh kernel setup      # 控制内核环境：Python 3.10，安装到 control/.venv/
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

## 2. 第一次仿真

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

## 3. 第一次真机

真机日常链路仍是旧机械臂服务（`src/legacy/`），内核真机入口待验收后切换。

```bash
./run.sh real start
```

窗口 1 是机械臂服务（首次需要输入 sudo 密码配置 CAN），窗口 2 是遥操，窗口 3 是网页。机械臂服务启动后进入阻尼待机，不会自动回零；阻尼不保证保持位置，先支撑机械臂。总控按键见 [日常操作](guides/operation.md)。

## 4. 从数据到策略

```text
遥操采集 ──▶ episode（MCAP + 视频）──▶ LeRobot v2.1 ──▶ H800 训练 ──▶ checkpoint ──▶ 机械臂执行
 sim / real      data review            data export        integrations/openpi    policy pi05-sft
```

```bash
./run.sh data review --help                # 查看、检查、标注、回放
./run.sh data export --source <episodes>   # 导出到新的 LeRobot 数据集目录
./run.sh policy pi05-sft --help            # 用微调模型控制机械臂
```

导出会在 `meta/quest_export.json` 记录每条数据的控制栈、相机集合和时间缺口，推理时据此提示相机是否一致。细节见 [训练数据导出](guides/openpi-dataset.md)。

## 5. 接下来

- 全部命令：`./run.sh`
- 架构：[架构概览](architecture/overview.md)
- 推进计划：[todo/](../todo/README.md)
