# 数据层

目标：一种原始 episode 格式，由内核 tick 和相机服务写入；一个读取库，旧格式永久保留只读适配；导出器按插件提供；原始数据从不改写。

## 审查时可写出的格式

| 格式 | 写入 | 读取 | 状态 / 动作 |
| --- | --- | --- | --- |
| A. Quest episode schema 1（MCAP `quest_teleop.*.v1`，MP4 / PNG，多相机） | `episode_io/recording.py` ← `quest_teleop/runtime/session.py` | `episode_io/mcap_io.py`、`quest_teleop/{datasets,export_lerobot,replay}.py`、`zero-shot/compute_stats.py` | 测量关节 + 夹爪，240 Hz；action 为 `requested_pos`（见 D-02） |
| B. 内核 episode schema 2（`quest_runtime.*.v2`，PNG，仅腕部） | `episode_io/runtime_episode.py` ← `collection/capture.py` | `dataset_tools/{runtime_v2,exporters}.py` | 反馈 q + 宽度；action 为 `sent_q` + 夹爪目标，250 Hz |
| C. GR00T 原始 episode（`schema_version 1`，无 `kind`） | `isaac_groot_data_collection/{raw_episode,recording}.py` ← 网页内核 | `isaac_groot_data_collection/{validator,exporter}.py` | 只有测量值，100 Hz，不记录命令 |
| D. LeRobot v2.1（OpenPI）+ `robot/*.npz` | `quest_teleop/export_lerobot.py` ← `scripts/datasets/convert.sh` | `integrations/openpi/run.py` | 7 维，30 Hz；两种 sidecar 布局（`multirate.py:20`、`exporters.py:57`） |
| E. LeRobot v2.1（GR00T，手写 parquet） | `isaac_groot_data_collection/exporter.py` | Isaac-GR00T | 16 维；action 为插值后的下一帧测量值 |
| F. 任务包 zip v2 | `quest_teleop/task_package.py` | 包内转换工具 | — |
| G. 诊断日志 `tuning.jsonl`、会话 `states.jsonl` | `control/arx5_vr/worker.py`、`session.py` | — | 非训练数据 |
| H. 旧 JSONL episode（只读） | 已不能写入 | `zero-shot/infer.py:16-36` | — |

## P0

- [ ] **D-01** 导出拼接时间轴 `P1`
  - 决定（2026-10-21）：不切分。缺口作为异常（视同现实世界的干扰）记入导出元数据与质量报告，记录位置、时长和原因（反馈过旧、缺相机帧、owner / phase / mode 排除），便于以后按原因筛选。实施并入 D-02。
  - 现状：
    - `quest_teleop/export_lerobot.py:272` 调用 `dataset.add_frame` 时不传时间戳，LeRobot 按 index / fps 生成。
    - episode 中间被剔除的帧会造成断点，包括反馈过旧（`multirate.py:11-12`）、缺外部帧（`export_lerobot.py:146-151`），以及 owner / phase / mode 排除（`observations/runtime_alignment.py:224-241`）。动作块可能跨越被删除的 Home 或 FreeDrive 段。
    - GR00T 导出同样重新生成时间（`exporter.py:321`），并折叠静止帧（`exporter.py:640-668`）。
  - 做法：在缺口处切成独立片段，或写入真实时间与断点；作为导出布局新版本，旧导出保持可复现。
  - 完成：每个导出 episode 的缺口都有记录。

- [ ] **D-02** 导出写入 action 定义与来源 `P0`
  - 现状：
    - schema 1 的 `requested_pos` 在服务路径是 Ruckig 每周期的插值点（`robot_control/service.py:153` → `hardware.py:236-247`），在直连路径是原始 IK 目标（`hardware.py:257`），两者使用同一个 schema 标识。
    - `docs/recording.md:94` 称它为"最新输入目标"；manifest 的 `action_definition`（`session.py:376`）描述的却是 submission / sdk_cmd。
  - 做法：每次导出写 `meta/x5_export.json`，记录 action 定义、控制栈（旧服务 / arx 内核 / 直连）、采样频率、相机集合、夹爪单位与来源、片段边界。
  - 完成：P-01、P-05 的推理检查可以读取这些字段。

## 一致性

- [ ] **D-03** 夹爪语义 `P1`
  - 现状：OpenPI 导出使用请求宽度（力保持之前，≤ 0.082 m，`export_lerobot.py:64`）；GR00T 导出使用下一帧测量宽度（`exporter.py:167`）；`arx5_control` 按 SDK 出厂宽度 0.088 换算。同一个电机位置在不同数据集里对应不同宽度。
  - 做法：在 D-02 元数据中标明；不同来源的数据不能混合训练。

- [ ] **D-04** schema 标识冲突 `P1`
  - 现状：Quest episode、GR00T 原始 episode 和 tuning 日志都写 `schema_version: 1`；`dataset_tools/exporters.py:71` 只按数字选择适配器。
  - 做法：改用带命名空间的标识，如 `x5.episode/3`；旧数据按 `kind` 和文件结构识别。

## 结构

- [ ] **D-05** 导出回归样本 `P1`
  - 做法：用现有各格式的小样本建立 golden 测试，确保后续重构导出结果不变。必须在 D-06 – D-08 之前完成。

- [ ] **D-06** 单一 MCAP 写入与读取 `P2`
  - 现状：两套写入器（`episode_io/mcap_io.RecordWriter`、`episode_io/runtime_episode.RuntimeEpisodeWriter`，各自处理时钟偏移和 schema 注册）；两套读取器（`mcap_io.read_topics`、`dataset_tools/runtime_v2.load_episode`）；两个重叠的 schema 1 校验（`datasets.validate_episode`、`export_lerobot.load_episode`）。
  - 做法：`episode_io` 拆为 schema / writer / reader / alignment / quality；`open_episode()` 返回统一 `Episode`，旧格式由只读适配提供。

- [ ] **D-07** 单一对齐器 `P2`
  - 现状：至少 5 套：`multirate.camera_rows`、`runtime_alignment.align`、`datasets.nearest_indices`、`quality.nearest_skew`、GR00T 的 `np.interp`（`exporter.py:125`，会使用未来数据）。
  - 做法：以 `observations/runtime_alignment.py` 为基础，统一为因果对齐，并输出片段 ID。

- [ ] **D-08** 导出器插件化 `P2`
  - 做法：`dataset_tools/exporters/{openpi_lerobot,groot_lerobot}` 接收 `Episode`；3 份 GR00T modality 配置改为由 spec 生成（X-25）。

- [ ] **D-09** 真机采集切换到内核 `P1`
  - 现状：仿真采集已从内核写入 schema 2（`robot_simulation/session.py`）；真机采集仍走旧 `TeleopSession`。
  - 做法：随 K-09 阶段 2 一起写好并在仿真中验证；阶段 3 真机验收后作为默认。

- [ ] **D-10** 拆分 TeleopSession `P2`
  - 现状：`quest_teleop/runtime/session.py` 同时管理机器人快照、相机、任务、录制、回放和打包。
  - 做法：拆为采集、文件读写、只读回放，设备只通过客户端访问。

- [ ] **D-11** schema 2 工具补齐 `P2`
  - 现状：回放、标注和质量检查只支持 schema 1（`quest_teleop/datasets.py:21`）。
  - 依赖：D-06。

- [ ] **D-12** 新原始格式 schema 3 `P2`
  - 做法：由内核写入，topics 包括 `/robot/feedback`、`/robot/command`（实际发送值）、`/robot/intent`（操作者目标）和 `/camera/<id>`。A、B、C 三种格式的读取器永久保留。
  - 依赖：D-05、D-06；在仿真内核上先完成。
