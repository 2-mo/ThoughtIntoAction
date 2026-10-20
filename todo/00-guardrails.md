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

- [ ] **G-03** 按环境分组的离线测试命令 `P1`
  - 现状：测试分散在 `tests/`、`tests/migrated/{arx_motion,phone}`、`src/zero-shot/test_*.py`、`src/infer_after_sft/test_*.py`、`control/scripts/check_*.py`、`scripts/check*.py`、`tests/*.mjs` 和 `apps/console/tests/`，分别依赖不同环境。
  - 做法：一个入口按环境运行：控制（Python 3.10，`./run.sh control verify`）、平台（Python 3.12）、手机、前端。各组可单独运行。
  - 完成：新入口列出各组结果；`AGENTS.md` 的最小验证仍可只运行单组。
  - 依赖：G-02。

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
    - `tests/test_package_boundaries.py::test_monitor_never_imports_a_controller_or_device_sdk`：`robot_simulation.cli.main([])` 绑定默认端口 18081，本机已有服务在监听，导致 `OSError`。测试依赖本机环境，应传入 `--port 0`。
    - `tests/test_plugins.py::test_control_inputs_use_same_service_and_release_only_the_lease`（`:101`）：`--input probe` 结束后服务仍保留 owner，日志为 "X5 stop-and-hold requested"。测试期望释放，实际是停止后保持；需确认是 `172982f` 改变了停止语义而测试未更新，还是 `release_after_stop` 期间 owner 无法清除（`src/robot_control/service.py:89,247`）。
  - 完成：两项通过，或注明属于旧内核、随 K-10 退役。

- [ ] **G-06** CI 或本地 pre-commit `P2`
  - 现状：没有 `.github`、pre-commit、ruff、mypy 或 pyright 配置（只有 `environments/phone/pyproject.toml` 有）。
  - 做法：先接入 ruff（少量规则）、G-03 离线测试、`control verify` 和来源校验。类型检查在包化（B-01）后接入。
