# 安全网

重构前先能回到基线，并能用一条命令确认离线行为没有变化。

- [x] **G-01** 基线标签 `P0`
  - 做法：在当前 `main` 打 tag，之后重构在独立分支进行，按 `AGENTS.md` 用 `--no-ff` 合并。
  - 完成：附注 tag `pre-platform-refactor` 指向 `172982f`（审查基线）；重构分支为 `refactor/platform`。tag 只在本地，未推送。

- [ ] **G-02** 根目录测试配置 `P1`
  - 现状：没有根 pytest 配置、`conftest.py` 或测试标记。`./run.sh test` 只运行 `tests/` 顶层 unittest（`scripts/check.py`），不包含 `tests/migrated/`。根 `.venv` 没有 pytest。在 arx_motion 环境中 collect 整个 `tests/` 有 21–22 个错误：缺依赖、`arm_service_fixture` / `mcap_fixture` 同级导入失败，以及两个同名 `test_golden_vectors.py` 冲突。
  - 做法：
    - 在根 `pyproject.toml` 增加 `[tool.pytest.ini_options]`，设 `--strict-markers`，标记 `unit`、`sim`、`hardware`、`browser`。
    - `conftest.py` 默认跳过 `hardware`，共享 fixture 放入 conftest。
    - pytest 加入根 `test` 依赖组。
  - 完成：`.venv/bin/python -m pytest --collect-only -q` 无错误。
  - 依赖：X-02 可同时处理同名测试冲突。

- [ ] **G-03** 按环境分组的离线测试命令 `P1`
  - 现状：测试分散在 `tests/`、`tests/migrated/{arx_motion,phone}`、`src/zero-shot/test_*.py`、`src/infer_after_sft/test_*.py`、`control/scripts/check_*.py`、`scripts/check*.py`、`tests/*.mjs` 和 `apps/console/tests/`，分别依赖不同环境。
  - 做法：一个入口按环境运行：控制（Python 3.10，`./run.sh control verify`）、平台（Python 3.12）、手机、前端。各组可单独运行。
  - 完成：新入口列出各组结果；`AGENTS.md` 的最小验证仍可只运行单组。
  - 依赖：G-02。

- [ ] **G-04** 收敛检查（棘轮） `P1`
  - 做法：在 `tests/test_package_boundaries.py` 现有机制上增加白名单检查，名单只允许缩小：
    - `Arx5JointController(` 构造位置（目标只剩 `control/arx5_vr/real.py`，见 C-02）；
    - `sys.path.insert` / `sys.path.append`（审查时 45 个文件）；
    - 字面量 `"can0"`、`/home/`；
    - `control/` 对 `src/` 的导入（目标只剩共享契约包，见 K-03）。
  - 完成：新增违规时检查失败；每次清理同步缩小白名单。

- [ ] **G-05** 删除前的引用检查 `P2`
  - 做法：小脚本，输入模块或目录，列出 `src/`、`control/`、`scripts/`、`configs/`、`apps/` 中的运行时引用，排除 `tests/`、`docs/` 和 `third_party/migration/manifest.json`。`redundancy.md` 中的删除都先运行它。

- [ ] **G-06** CI 或本地 pre-commit `P2`
  - 现状：没有 `.github`、pre-commit、ruff、mypy 或 pyright 配置（只有 `environments/phone/pyproject.toml` 有）。
  - 做法：先接入 ruff（少量规则）、G-03 离线测试、`control verify` 和来源校验。类型检查在包化（B-01）后接入。
