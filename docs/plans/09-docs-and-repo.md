# 文档与仓库

- [x] **O-01** 修正 README 与代码不符之处 `P1`
  - 已做（2026-10-21）：README 重写后，三处事实在移出的 `docs/guides/operation.md`、`consolidated-components.md` 中修正（真机仍走旧链路、`MAX_JOINT_SPEED_RAD_S=0`、归档目录未纳入 git）。
  - 现状：
    - README 第 5 行称"默认 Quest 遥操继续使用 arx 原生链路"，`docs/consolidated-components.md:5` 说法相同。实际真机日常仍走旧链路，README 第 50 行"未更换真机控制核心"才是准确的。
    - README 第 101 行写 `MAX_JOINT_SPEED_RAD_S=2.0`，脚本中为 0。
    - README 还指向本机未跟踪的 `outputs/archives/migration/originals/`。
  - 做法：先修正这些事实，再做 O-02。

- [x] **O-02** 重写 README `P2`
  - 已做：README 改为平台概览与命令表，原操作说明移到 `docs/guides/operation.md`。
  - 现状：17 KB，日常启动从第 28 行开始，安装在第 147 行；缺少新人入门说明。
  - 做法：README 只保留一页：平台是什么、安装、第一次仿真、真机启动、文档链接。操作细节移到 guides。

- [x] **O-03** 重组 `docs/` `P2`
  - 已做（2026-10-21）：`docs/{getting-started.md, README.md, guides/, reference/, architecture/, history/}`；迁移记录、`imported/`、`validation/` 移入 `history/`；新增入门与架构概览；`tests/test_docs.py` 检查文档相对链接。测试按环境分为 `tests/{motion,phone,browser}`；`scripts/` 中文文件名改为 ASCII，删除 4 个与命令目录重复的包装脚本。
  - 做法：
    - 分为 getting-started / guides / architecture / reference / history。
    - 迁移方案与实施记录（`arx-runtime-redesign.md`、`arx-runtime-implementation.md`、`control-inputs-and-motion.md`、`plugin-migration-inventory.md`、`migration-review.md`、`refactor.md`、`validation/*.txt`）移入 history。
    - `docs/imported/`（85 个文件，含 `.vscode`、Makefile 和 shell 脚本）移入 history，或直接删除，git 历史保留原文。
  - 依赖：本目录的待办完成后，history 中记录完成情况。

- [x] **O-04** 过时的结构说明 `P2`
  - 已做：`src/README.md` 按新结构重写，`docs/layout.md` 移入 history。
  - 现状：`docs/layout.md` 和 `src/README.md` 没有提到 `control/`、`apps/`、`configs/environments/`。`docs/layout.md` 称配置只由 `robot_core/project.py` 读取，但 `src/arx5_paths.py` 被 17 个文件引用。
  - 做法：随 B-01 重写，或合并进 architecture。

- [ ] **O-05** 资产去重 `P2`
  - 现状：49 份重复文件，约 50 MB。
    - X5 网格各 4 份：`assets/arx_motion/weights/browser/geometry/x5`、`assets/phone/dashboard/models/arx5`、`assets/robots/x5/meshes/X5`、`assets/robots/x5/runtime/meshes`。
    - 木纹纹理 2 份：`assets/arx_motion/textures`、`src/robot_simulation/web/textures`。
    - three.js 3 份（U-03）。
  - 做法：网格统一引用 `assets/robots/x5/runtime/meshes`（R-03）；其余随使用者合并删除。

- [ ] **O-06** 大文件存放 `P3` `待定`
  - 现状：
    - 已跟踪工作树约 300 MB。其中 `src/embodied/devices/inputs/quest/native/bin/rollio-*` 约 32 MB ELF（上游同样跟踪），`src/embodied/devices/robots/arx5/vendor/arx5-sdk/source.tar` 28 MB。
    - `assets/arx_motion/mediapipe` 约 84 MB，`assets/robots/rebot` 网格约 92 MB。
  - 选项：Git LFS，或 release 附件加 `dependencies.lock.toml` 的 sha256 获取。`src/embodied/devices/inputs/quest/native/` 保持与上游一致。

- [ ] **O-07** git 维护 `P3`
  - 现状：`.git` 122 MB，2482 个松散对象，从未执行过 gc。
  - 做法：`git gc`，只压缩，不改写历史；需用户确认后执行。

- [ ] **O-08** 未跟踪目录 `P2` `待定`
  - 已做（2026-10-22）：本地参考截图移至 `assets/references/`，平台计划移至 `docs/plans/`；本机归档、标定和数据分别归并至 `outputs/archives/`（28 GB）、`outputs/calibrations/`、`outputs/datasets/`。已有环境、构建和日志仍保留在 `outputs/` 的原位置；本机产物与 `outputs/runtime/control/` 继续由 `.gitignore` 忽略。
  - 待定：`outputs/archives/migration/originals/` 何时移出工作区由用户决定；本轮只迁移目录，保留完整内容与历史元数据。

- [ ] **O-09** 文件命名 `P3`
  - 已做（2026-10-22）：五个中文快捷脚本集中至 `scripts/shortcuts/`，根目录仅保留 `run.sh`；统一命令继续可用，遥操参数与控制行为保持不变。
  - 做法：代码、模块与 `src/` 下的路径使用 ASCII，包括 `src/infer_after_sft/*.sh`；中文快捷文件名仅保留在 `scripts/shortcuts/`。
  - 统一包名：`reBotArm_control_py`、`zero-shot`，以及 `handeye_calibration` / `hand_eye_calibration`。
