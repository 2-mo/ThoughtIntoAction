# 控制内核：arx-televr

`control/` 是平台唯一控制内核，操控行为以上游 arx-televr 为准。审查时状态：

- 上游文件、执行位与哈希冻结在 `control/UPSTREAM_FILES.json`；13 个局部补丁登记在 `control/LOCAL_PATCHES.json`。
- 本地新增 `control/adapters/`（15 个文件）、`config/components.json`、`vendor/arx5-sdk/`（夹爪补丁 SDK）和 4 个检查脚本。
- 仿真已使用此内核：`启动网页仿真.sh` → workbench simulation → `./run.sh control start --backend sim`。
- **真机日常仍走旧链路**：`启动机械臂服务.sh` → `robot_control.service`，`启动遥操.sh` → `quest_teleop`（placo IK）→ `ArmClient`。`control/run/` 下 64 次运行记录全部是 `backend=sim`。

## 与上游保持同步

- [ ] **K-01** 上游同步流程 `P1`
  - 现状：`src/workstation/control.py` 与 `control/scripts/package_release.py` 读取 `UPSTREAM_FILES.json` / `LOCAL_PATCHES.json`，未核对 `./run.sh control verify` 是否完整比对全部上游文件。
  - 做法：
    - 确认 verify 会检查非 adapters 文件的哈希，必须等于上游值或补丁值。
    - 在 `control/README.md` 末尾写明同步步骤：获取上游新提交 → 对比 `UPSTREAM_FILES.json` → 重放补丁 → 更新三份记录 → `control verify`。
  - 完成：任意改动一个上游文件而不登记补丁，verify 失败。

- [ ] **K-02** 复核 13 个局部补丁 `P2`
  - 做法：逐项判断保留、移到 `adapters/`，或整理后提交给上游。目标是让 `arx5_vr/` 的补丁只剩确实无法在外围完成的部分。
  - 涉及：`app.py`、`runtime.py`、`worker.py`、`real.py`、`sim.py`、`recording.py`、`model.py`、`types.py`、`ipc/__init__.py`、`pyproject.toml`、`uv.lock`、`scripts/{package_release,self_test}.py`。

- [ ] **K-03** 内核对 `src/` 的反向依赖 `P1`
  - 现状：
    - `arx5_vr/app.py:95` 和 `worker.py:48` 通过 `sys.path` 引入 `src/component_api`。
    - `adapters/workbench.py:15` 引入旧遥操包 `quest_teleop.arming.DownHoldGate`。
    - `adapters/commands.py:262,271` 引入 `operations.vendor.arx_motion.profiles`。
    - `control/scripts/check_workbench_teleop.py` 引入 `runtime_client`。
  - 做法：内核只依赖一个兼容 Python 3.10 的小型共享包（注册表、设备锁、运动 profile、协议常量）；`DownHoldGate` 移入 `adapters/` 或共享包，不再依赖 `quest_teleop`。
  - 完成：G-04 的 `control/` → `src/` 导入白名单只剩共享包。

## 工作台适配与上游操控一致

- [ ] **K-04** 按键映射 `P1` `待定`
  - 现状：
    - 上游（`control/README.md` 的右手按键表）：A 按住 ≥150 ms 后松开，切换 Disabled → FreeDrive ↔ CommandFollowing；A+B 回 Home 并打开夹爪；Grip 跟随，松开再握重新对齐；Trigger 控制夹爪。
    - 工作台适配（`adapters/workbench.py`，提示文字在 `arx5_vr/app.py:215`）：过滤 coordinator 的按键输入，右摇杆下拨 1 s 启用；Grip / Trigger 不变；A/B 留给采集和预设。
    - 旧链路：A 预抓取并开始采集，B 保存，长按 B 回 Home，右摇杆管理任务。
  - 做法：定一份按键表。控制相关按键（模式、跟随、夹爪、Home）语义与上游一致；采集键另行分配，不与控制键冲突。按键表放一处，终端提示、网页和文档都引用它。
  - 选项：
    1. 恢复上游 A / A+B 语义，采集改用左手柄或摇杆；
    2. 保留工作台的启用门，记为外围适配，并写明与上游的差异。

- [ ] **K-05** 工作台适配改写原生配置文本 `P2`
  - 现状：`adapters/workbench.py` 中 `QuestDevice.commands` 用字符串替换追加左手柄通道（把 `PICO Right Controller` 替换成 `Quest Left Controller`）；`Routing.commands` 替换 topic 后写回 coordinator 配置。
  - 做法：在生成原生配置时（`arx5_vr/native_config.py` 之后）用结构化数据生成，不对文本做替换。

## 真机切换

- [ ] **K-06** 旧链路与上游的行为差异 `P0` `待定`
  - 默认采用上游，逐行决定是否需要外围适配：

    | 场景 | 旧链路（当前真机） | 上游 arx-televr |
    | --- | --- | --- |
    | 控制端失联 | 0.5 s 后释放并进入 SDK 阻尼，不保位（`robot_control/service.py:40,88-90`） | 租约过期后位置保持（`adapters/commands.py:170-171,206-210`） |
    | 启动 | 阻尼待机，不回零（`service.py:399`） | 使能并回 `park_q`（默认全零，`app.py:153`，`model.py:162-171`） |
    | 退出 | 阻尼 | SDK 阻尼，不回 Home（`worker.py:209-212`） |
    | 运动整形 | Ruckig 限加速度 / jerk（`启动遥操.sh:23-24`：12 / 120） | 每 tick 关节步长 0.0873 rad，加 SDK 限速 |
    | 桌面平面 | 有（`robot_core/safety.py`，`project.toml:70-74`） | 无 |
    | 夹爪 | 上限 0.078 m，力保持 0.6 / 0.3 Nm（`project.toml:60-67`，`hardware.py:338-393`） | 上限 0.082 m，夹爪力矩 0（`real.py:427`） |
    | IK | placo，控制 link6 | MuJoCo 阻尼最小二乘，控制 `tcp_link` |

  - 建议：
    - 原生 Quest 路径完全按上游。
    - 桌面 / 工作空间约束作为 gateway 的外围检查，只作用于非原生来源（手机、策略、操作）；与 C-04 一起实施。
    - 启动回 Home 是上游行为，需在操作说明中写明；`park_q` 可在 `robot.json` 配置。
  - 完成：每行有决定并记录在此。

- [ ] **K-07** 统一真机入口的夹爪补丁 `P0`
  - 现状：`real.py:161` 只在环境变量 `QUEST_ARX_GRIPPER_TURNS=1` 时启用本机夹爪修正。该变量只由 `src/workstation/control.py:54` 设置，`control/start_real.sh:11` 不设置；`start_real.sh` 也不像 `workstation/control.py:40-49` 那样清理 `PYTHONPATH`、`LD_LIBRARY_PATH` 和 `CONDA_*`。
  - 做法：真机只允许经 `./run.sh control start` 启动；或者 `start_real.sh` 改为调用它。缺少补丁 SDK 时拒绝启动。
  - 完成：两种方式启动真机时补丁状态与环境相同。

- [ ] **K-08** 真机验收 `P1` `阶段 3`
  - 内容：
    - 上游 README 的启动、模式切换、Grip 跟随、A+B Home、Trigger、Ctrl+C 阻尼；
    - 工作台 managed 模式与租约；
    - 复用 `outputs/migration/native-faults-*` 的故障场景；
    - 记录 `control/run/<RUN>/report.json` 与 `tuning.jsonl`。
  - 由操作者执行。同时补做阶段 0–2 中标注"未真机验证"的条目。
  - 依赖：K-04、K-06、K-07、R-01。

- [ ] **K-09** 日常真机入口切换到内核 `P1`
  - 做法：
    - 阶段 2：基于内核的真机入口与真机采集（D-09）先写好，用假 SDK 或 MuJoCo 验证，不作为默认入口。
    - 阶段 3：K-08 通过后，`启动工作台.sh` 真机模式、`启动机械臂服务.sh`、`启动遥操.sh` 默认改用内核。
  - 依赖：默认切换依赖 K-08。

- [ ] **K-10** 退役旧控制内核 `P2` `阶段 3`
  - 范围：
    - `src/robot_control/{service,client,hardware,online_motion,motion,trajectory,console,startup,cli}.py` 和 `backends/`；
    - `quest_teleop` 的控制部分；
    - XRoboToolkit placo IK 依赖。
    - `motor_feedback.py` 如仍需要，作为只读诊断保留。
  - 依赖：K-09，以及 C-02 中依赖 `ArmClient` 的客户端全部迁移。见 `redundancy.md` X-17。

## 内核运行管理

- [ ] **K-11** 界面不负责内核生命周期 `P1`
  - 现状：仿真时由 viewer 进程启动内核（`src/robot_simulation/arx_preview.py:125-128` 执行 `control start --managed`），并轮询 `runtime.json`、`workbench-request.json` 等文件。
  - 做法：由工作台或 CLI 启动内核；viewer 只订阅状态。

- [ ] **K-12** `control/run/` 清理 `P3`
  - 现状：审查时 67 个运行目录、213 MB，没有清理机制。
  - 做法：保留最近 N 次，或按天数清理。
