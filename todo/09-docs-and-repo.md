# 文档与仓库

- [ ] **O-01** 修正 README 与代码不符之处 `P1`
  - 现状：
    - README 第 5 行称"默认 Quest 遥操继续使用 arx 原生链路"，`docs/consolidated-components.md:5` 说法相同。实际真机日常仍走旧链路，README 第 50 行"未更换真机控制核心"才是准确的。
    - README 第 101 行写 `MAX_JOINT_SPEED_RAD_S=2.0`，脚本中为 0。
    - README 还指向本机未跟踪的 `archives/migration/originals/`。
  - 做法：先修正这些事实，再做 O-02。

- [ ] **O-02** 重写 README `P2`
  - 现状：17 KB，日常启动从第 28 行开始，安装在第 147 行；缺少新人入门说明。
  - 做法：README 只保留一页：平台是什么、安装、第一次仿真、真机启动、文档链接。操作细节移到 guides。

- [ ] **O-03** 重组 `docs/` `P2`
  - 做法：
    - 分为 getting-started / guides / architecture / reference / history。
    - 迁移方案与实施记录（`arx-runtime-redesign.md`、`arx-runtime-implementation.md`、`control-inputs-and-motion.md`、`plugin-migration-inventory.md`、`migration-review.md`、`refactor.md`、`validation/*.txt`）移入 history。
    - `docs/imported/`（85 个文件，含 `.vscode`、Makefile 和 shell 脚本）移入 history，或直接删除，git 历史保留原文。
  - 依赖：本目录的待办完成后，history 中记录完成情况。

- [ ] **O-04** 过时的结构说明 `P2`
  - 现状：`docs/layout.md` 和 `src/README.md` 没有提到 `control/`、`apps/`、`environments/`。`docs/layout.md` 称配置只由 `robot_core/project.py` 读取，但 `src/arx5_paths.py` 被 17 个文件引用。
  - 做法：随 B-01 重写，或合并进 architecture。

- [ ] **O-05** 资产去重 `P2`
  - 现状：49 份重复文件，约 50 MB。
    - X5 网格各 4 份：`assets/arx_motion/weights/browser/geometry/x5`、`assets/phone/dashboard/models/arx5`、`assets/robots/x5/meshes/X5`、`control/models/x5/meshes`。
    - 木纹纹理 2 份：`assets/arx_motion/textures`、`src/robot_simulation/web/textures`。
    - three.js 3 份（U-03）。
  - 做法：网格统一引用 `control/models/x5/meshes`（R-03）；其余随使用者合并删除。

- [ ] **O-06** 大文件存放 `P3` `待定`
  - 现状：
    - 已跟踪工作树约 300 MB。其中 `control/native/bin/rollio-*` 约 32 MB ELF（上游同样跟踪），`control/vendor/arx5-sdk/source.tar` 28 MB。
    - `assets/arx_motion/mediapipe` 约 84 MB，`assets/robots/rebot` 网格约 92 MB。
  - 选项：Git LFS，或 release 附件加 `dependencies.lock.toml` 的 sha256 获取。`control/native/` 保持与上游一致。

- [ ] **O-07** git 维护 `P3`
  - 现状：`.git` 122 MB，2482 个松散对象，从未执行过 gc。
  - 做法：`git gc`，只压缩，不改写历史；需用户确认后执行。

- [ ] **O-08** 未跟踪目录 `P2` `待定`
  - 现状：`参考图像/` 未被 `.gitignore` 覆盖。`archives/`（28 GB）、`outputs/`（14 GB）、`calibrations/`、`control/run/` 已被忽略。
  - 做法：决定 `参考图像/` 是提交还是忽略；`archives/migration/originals/` 何时移出工作区由用户决定。

- [ ] **O-09** 文件命名 `P3`
  - 做法：根目录中文入口脚本保留，方便操作者；代码、模块与 `src/` 下的路径使用 ASCII，包括 `src/infer_after_sft/*.sh`、`scripts/**/*.sh`。
  - 统一包名：`reBotArm_control_py`、`zero-shot`，以及 `handeye_calibration` / `hand_eye_calibration`。
