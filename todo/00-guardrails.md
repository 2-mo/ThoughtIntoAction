# 安全网

重构前先能回到基线，并能用一条命令确认离线行为没有变化。

- [x] **G-01** 基线标签 `P0`
  - 做法：在当前 `main` 打 tag，之后重构在独立分支进行，按 `AGENTS.md` 用 `--no-ff` 合并。
  - 完成：附注 tag `pre-platform-refactor` 指向 `172982f`（审查基线）；重构分支为 `refactor/platform`。tag 只在本地，未推送。

- [x] **G-02** 根目录测试配置 `P1`
  - 原状：没有根 pytest 配置、`conftest.py` 或测试标记。`./run.sh test` 只用 unittest 运行 `tests/` 顶层，不包含 `tests/migrated/`；根 `.venv` 没有 pytest。
  - 已做：
    - 根 `pyproject.toml` 增加 `test` 依赖组（`pytest==8.4.1`，与 analysis 环境相同）并设为 uv 默认组，`uv sync --locked` 不会移除它。
    - 增加 `[tool.pytest.ini_options]`：`testpaths = ["tests"]`，递归时跳过 `migrated`（各自环境运行），`--strict-markers`，标记 `hardware`、`sim`。
    - `tests/conftest.py` 默认跳过 `hardware` 标记，`--run-hardware` 时运行。
    - `scripts/check.py` 改为 pytest；`./run.sh test` 用法不变，环境仍由 `launch.py` 设置。
  - 结果：
    - `./run.sh test --collect-only` 收集到 366 项，无错误。其中包括原 unittest 的全部 362 项（已逐项比对），以及 `tests/golden/` 中 4 项此前未被运行的 pytest 测试。
    - 全量运行：359 通过、5 跳过、2 失败。2 项失败在基线上同样存在（见 G-07）。
    - `components run test.motion` / `test.phone` 在新配置下仍分别收集到 65 / 263 项。
  - 未做：共享 fixture（`arm_service_fixture.py`、`mcap_fixture.py`）在 prepend 导入模式下工作正常，暂不移入 conftest。`unit`、`browser` 标记暂不登记（浏览器测试是 node 运行的 `.mjs`）。

- [x] **G-03** 按环境分组的离线测试命令 `P1`
  - 原状：测试分散在 `tests/`、`tests/migrated/{arx_motion,phone}`、`src/zero-shot/test_*.py`、`src/infer_after_sft/test_*.py`、`control/scripts/check_*.py`、`scripts/check*.py`、`tests/*.mjs` 和 `apps/console/tests/`，分别依赖不同环境。
  - 已做：`./run.sh check-all`（`scripts/check_all.py`）依次运行以下各组，每组重新进入 `run.sh`，环境与单独运行时相同；缺少环境的组显示 SKIP。支持 `--only`、`--simulation`（加跑 12 s MuJoCo 链路）和 `--print`。

    | 组 | 命令 |
    | --- | --- |
    | control | `control verify` + `control check` |
    | platform | `test` |
    | motion | `components run test.motion` |
    | phone | `components run test.phone` |
    | rebot | `components run test.rebot` |

  - 基线结果：control、motion（65）、phone（263）、rebot 通过；platform 为 359 通过、5 跳过，失败见 G-07。
  - 未纳入：`src/zero-shot`、`src/infer_after_sft` 的 unittest（依赖 `outputs/envs/` 下的推理环境，随 P-07 迁入 `tests/`）、浏览器 `.mjs`、`apps/console` Playwright。

- [x] **G-04** 收敛检查（棘轮） `P1`
  - 已做：`tests/test_convergence.py` 扫描 `src/`、`control/`（跳过 `.venv`、`run`、`build`、`native`、`vendor`、`models`）、`scripts/`、`apps/console/src`、`integrations/`、`examples/`。名单在 `tests/convergence_baseline.json`：新增违规时失败；已清理的条目没有从名单删除时也失败，所以名单只能缩小。
  - 基线（`172982f` 之后）：

    | 检查 | 文件数 | 目标 |
    | --- | --- | --- |
    | `sdk_controller`：`Arx5JointController(` | 9 | 只剩 `control/arx5_vr/real.py`（C-02） |
    | `sys_path`：`sys.path.insert/append(` | 40 | 0（B-01） |
    | `can0_literal`：代码中的 `"can0"` / `=can0` | 22 | 0；上游默认值 `control/arx5_vr/types.py` 豁免（R-02） |
    | `home_path`：`/home/<user>/` | 5 | 0（R-05） |
    | `control_imports_src`：`control/` 导入 `src/` 的包 | 7 | 只剩共享包（K-03） |

  - 验证：基线通过；人为新增一处违规、添加一条过期条目，两种情况都会失败。

- [ ] **G-05** 删除前的引用检查 `P2`
  - 做法：小脚本，输入模块或目录，列出 `src/`、`control/`、`scripts/`、`configs/`、`apps/` 中的运行时引用，排除 `tests/`、`docs/` 和 `third_party/migration/manifest.json`。`redundancy.md` 中的删除都先运行它。

- [ ] **G-07** 基线上失败的两项测试 `P1`
  - 现状：在 `172982f` 上用 unittest 同样失败，与测试运行器无关。
    - `tests/test_package_boundaries.py::test_monitor_never_imports_a_controller_or_device_sdk`：两个原因。
      - 绑定默认端口 18081，本机工作台运行时会冲突（`OSError`）。测试应传入 `--port 0`。
      - 端口空闲时暴露真正的边界违规：`robot_simulation.cli` → `arx_preview.py:43` → `session.py:6` 导入 `quest_teleop.collection`（`Press`、`StickAction`），监看进程因此依赖旧遥操包。由 `172982f` 引入。按键解析应移到采集或 L0，不留在 `quest_teleop`。
    - 监看边界：已修复。测试改用 `--mode real --port 0`，并单独检查仿真模块可导入；`session.py` 不再导入 `quest_teleop`（K-04）。
    - `tests/test_plugins.py::test_control_inputs_use_same_service_and_release_only_the_lease`：已修复（2b39eff）。`172982f` 把遥操退出改为平滑停止后保持，服务停稳后才释放租约，测试却立即断言。改为最多等待 3 s；实测停稳后释放，此场景下不存在停止后卡住的问题。
  - 浏览器测试：已修复。`tests/browser_*.mjs` 测的是日常 18081 监看页，此前没有任何入口运行它们，因此多处期望已过时：`browser_scene` 未指定模式（继承工作台持久化的仿真模式），且仍按 66822fc 之前的 1440/1366/1024 视口和已移入详情面板的元素断言，并期望遥操退出后为阻尼（172982f 起为保持）；`browser_telemetry` 力矩图阈值 24 px 早于 66822fc 布局（实测真实页面 22.25 px，改为不塌缩的 20 px），且只去掉 `sensors.js` 的第一条 import。已加入 `check-all` 的 `browser` 组，7 项全部通过。页面随 U-01 退役时一并删除这些测试。
  - 偶发失败：`tests/test_camera_lifecycle.py::test_real_graceful_shutdown_of_dummy_socket_owner` 在一次 `check-all` 全量运行中失败（"相机服务尚未退出"），单独运行 3 次都通过，属于时序不稳定。
  - 完成：两项通过，或注明属于旧内核、随 K-10 退役；偶发项稳定通过。

- [ ] **G-06** CI 或本地 pre-commit `P2`
  - 现状：没有 `.github`、pre-commit、ruff、mypy 或 pyright 配置（只有 `environments/phone/pyproject.toml` 有）。
  - 做法：先接入 ruff（少量规则）、G-03 离线测试、`control verify` 和来源校验。类型检查在包化（B-01）后接入。
