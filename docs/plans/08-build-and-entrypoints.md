# 包结构、入口与环境

- [ ] **B-01** 包化 `P2`
  - 现状：
    - 根目录、`control/`、`configs/environments/arx_motion` 三个 uv 项目都设 `package = false`。
    - 45 个文件使用 `sys.path` 修改，64 个文件各自计算仓库根目录。
    - `scripts/launch.py:107-187` 按命令改写 `PYTHONPATH` / `LD_LIBRARY_PATH`；只有在 argv 中检测到 `--backend arx2025` 时才追加官方 SDK 路径（`:150-154`）。
  - 做法：
    - 平台（Python 3.12）改用 uv workspace，成员包 `package = true`、src 布局、可编辑安装。
    - `control/` 保持独立 Python 3.10 项目，与上游一致。
    - 共享契约包同时兼容 3.10 与 3.12（K-03）。
  - 完成：G-04 的 `sys.path` 白名单清空，`launch.py` 不再注入路径。

- [ ] **B-02** 统一命令入口 `P2`
  - 现状：约 10 种启动机制。
    - `scripts/launch.py`：49 个子命令，分布在 5 张手写表中（`:20-71`）。
    - `./run.sh components run`：52 个命令，覆盖 7 个环境（`configs/components/commands.json`）。
    - 另有 `./run.sh workbench`、`./run.sh control`、`scripts/workspace/tmux_workspace.py`。
    - `control/start_{sim,real,demo}.sh`。
    - 60 多个 `.sh`，其中 `src/arx_tools/` 10 个、`src/infer_after_sft/` 7 个、`src/zero-shot/` 3 个、`docs/imported/robot-teleop/` 若干。
    - 中文快捷脚本已集中到 `scripts/shortcuts/`，仿真入口已补齐执行权限；根目录统一从 `run.sh` 进入。
  - 做法：
    - 一棵命令树，注册为 `[project.scripts]`。
    - 中文快捷脚本保留在 `scripts/shortcuts/`；文档优先提供 `./run.sh <功能> <命令>`，不再增加根目录别名。
    - `src/` 下的 shell 脚本移出。
    - `control/start_*.sh` 保留上游文件，日常入口经统一 CLI（K-07）。

- [ ] **B-03** 合并注册表 `P2`
  - 现状：
    - `component_api.Registry`（87 行）合并三份数据：`configs/plugins.json`（10 项）、`configs/plugins.json`（22 项，静默 `.update()`）和 `configs/components/migrated.json`（33 项，重复时报错）。
    - `src/robot_core/plugins.py` 只是 `sys.modules` 别名。
    - `src/robot_teleop/registry.py`（285 行）是另一套手写注册表。
    - `migrated.json` 中 trajectory、path、collision、policy、kinematics、servo_algorithm 这几类在运行时没有调用者，只被测试引用。
  - 做法：一个注册表、一份数据文件（按 runtime 分节）；删除没有运行时调用者的登记；`robot_teleop/registry.py` 随 X-15 退役。

- [ ] **B-04** 环境收敛 `P2`
  - 现状：约 11 个 Python 环境：
    - 根 3.12、`control` 3.10；
    - `configs/environments/arx_motion` 的 `.venv` 与 `.venv-web`、`phone`（3.12）、`rebot`（3.10）；
    - Isaac-GR00T；
    - `outputs/envs/{pi05-zero-shot,pi05-sft,pi05-jax,lerobot-export}`；
    - conda 快照。
    
    版本互不一致：numpy 1.26.4 / 2.2.6 / 2.3.5，pin 3.7.0 / 4.1.0，ruckig 0.12.2 / 0.19.4，OpenCV 5.0 / contrib 4.12。
  - 做法：保留控制（3.10，上游）、平台（3.12），以及每个 GPU 策略后端和导出各一个独立环境。`arx_motion`、`phone` 并入平台；`rebot` 在接入前保持独立（C-10）。

- [ ] **B-05** 拆分巨型文件 `P3`
  - 范围：
    - `src/arx5_backend/kernel/control.py`（6002 行）
    - `src/hand_eye_calibration/collector.py`（5315 行）
    - `src/arx5_control/controller.py`（3996 行）
  - 做法：在各自的使用者迁到内核后再拆分（C-05、C-06）；能整体退役的直接退役。

- [ ] **B-06** 工作台状态来源 `P3`
  - 现状：tmux 工作台用 pane 选项和抓取的 pane 文本作为状态（`scripts/workspace/tmux_workspace.py:246-249`）；仿真 / 真机模式写入 txt 文件（`src/robot_simulation/mode.py:7-9`）。
  - 做法：由各进程报告自身状态，工作台只汇总。
