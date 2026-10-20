# 策略运行时

目标是一个策略运行时，由以下部分组成：

- spec：观测与动作定义，从导出元数据写入 checkpoint；
- 观测构建器；
- 动作块执行器：负责延迟跳步、过期拒绝、分布外状态拦截、单步限幅和失败预算；
- sink：通过 `ControlSession` 把动作交给内核；
- GPU 推理服务：独立环境，机械臂侧不导入 torch / jax。

审查时有 3 套推理 runner：

| runner | 模型 | 机械臂接口 |
| --- | --- | --- |
| `src/zero-shot/`（`rollout.py`、`real.py`） | LeRobot PI05 base | `robot_control.client.ArmClient` → 旧机械臂服务 |
| `src/infer_after_sft/`（PyTorch / JAX） | openpi `215abfb` | 同上，经 `sys.path` 复用 zero-shot 的 `control_loop`（`run.py:10,17-19`） |
| `src/gr_zeroshot/`（实为微调 GR00T） | 远程 ZMQ 服务（`policy.py:55-82`） | 直接使用 `arx5_control.ArmController`（`runtime.py:605`），100 Hz |

## P0

- [ ] **P-01** 校验相机集合 `P0`
  - 现状：
    - 导出包含外部相机时，openpi 训练配置自动启用它（`integrations/openpi/run.py:33`）。`configs/local.toml:40` 配置了 `wrist_rgb` 和 `exterior_rgb`。
    - SFT 推理只打开腕部相机（`src/infer_after_sft/run.py:85`）；`integrations/openpi/x5_policy.py:16-18` 将 base 图像填零、mask 设为 False。
    - `src/infer_after_sft/checkpoint.py` 不检查训练时用了哪些相机。
  - 做法：checkpoint 记录相机集合（来自 D-02）；推理启动时比对，不一致则拒绝运行，或按实际训练配置打开相机。
  - 依赖：D-02。

- [ ] **P-02** `--preview` 不移动机械臂 `P0`
  - 现状：`src/infer_after_sft/run.py:76` 在判断 `args.preview` 之前就执行 `move_pregrasp(arm, open_gripper=True)`。
  - 做法：preview 只做观测与预测，不发送运动；需要预抓取的流程使用单独参数。

- [ ] **P-03** GR00T 运行时保护 `P0`
  - 现状：
    - 反馈速度保护被设为浮点最大值（`src/gr_zeroshot/runtime.py:612`），默认值为 0.60 rad/s（`arx5_control/controller.py:99`）。
    - 被拒绝的动作块和 worker 错误被忽略（`runtime.py:744,773`），没有失败计数或时间限制。
    - 实时延迟按 0 传入（`runtime.py:755`），过期步不会被跳过；不检查观测年龄。
    - 任何异常都会自动回 Home（`runtime.py:826`）。
  - 做法：恢复速度保护；增加失败预算、观测年龄检查和延迟跳步；异常时保持当前位置，回 Home 改由操作者确认。

## 一致性

- [ ] **P-04** 执行参数与采集一致 `P1`
  - 现状：采集时加速度 12 rad/s²、jerk 120 rad/s³（`启动遥操.sh:23-24`）；SFT 推理为 1 / 4（`infer_after_sft/run.py:79`）；zero-shot 为 0.5 / 2（`zero-shot/rollout.py:229`），并在测量位置附近每步限幅 ±0.08 rad（`rollout.py:78`）。
  - 做法：内核切换后，以内核执行参数为准，与数据的 action 定义对应，并写入 spec。

- [ ] **P-05** 区分不同 action 定义训练的 checkpoint `P1`
  - 现状：schema 1 与 schema 2 的导出都使用配置名 `pi05_quest_x5`（`integrations/openpi/quest_x5.py:38-43`）和资产 ID `local/quest_x5`。schema 2 的 action 是内核 250 Hz 的 `sent_q`（`control/adapters/ticks.py:219-221`），推理却通过 240 Hz 的旧 Ruckig 服务执行（`zero-shot/rollout.py:141-142,182`）。
  - 做法：spec 带上 action 定义与控制栈；推理时与实际执行路径比对。
  - 依赖：D-02。

## 结构

- [ ] **P-06** 统一策略运行时 `P1`
  - 做法：
    - 新建 policy 包，包含 spec / observation / executor / client / sinks。
    - 现有 `ArmClient` runner 的保护（采集前制动、5 s 推理超时、图像与状态时间差 > 60 ms 或图像 > 200 ms 拒绝、观测 > 1.5 s 拒绝、偏离观测 > 0.2 rad 拒绝、分布外状态拦截，见 `zero-shot/rollout.py:58,114,124,134,137`）迁入 executor 作为默认值。
    - JAX 的 horizon 与模型配置从 checkpoint 读取，不在代码中假设（`infer_after_sft/checkpoint.py:76-81`）。
  - 依赖：C-01。先在 MuJoCo 内核上执行验证，真机执行放到阶段 3。

- [ ] **P-07** 整理目录 `P2`
  - 现状：`src/zero-shot`（连字符目录，不能作为包导入）和 `src/infer_after_sft`（含中文 `.sh`）的 `test_*.py` 放在 `src/` 中。
  - 做法：并入 policy 包，测试移到 `tests/`，shell 入口并入统一 CLI。

- [ ] **P-08** GR00T 改为内核客户端 `P1`
  - 做法：`gr_zeroshot` 改名（如 `policy_groot`），通过 P-06 的 sink 执行，不再直接使用 `ArmController`。
  - 依赖：P-03、P-06。
