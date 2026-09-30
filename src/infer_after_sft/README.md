# 使用微调后的 π0.5 控制 X5

默认权重：`/home/r2/work/checkpoints/2500`。这是 OpenPI PyTorch 训练到第 2500 步的 checkpoint，不是 JAX 参数目录，也不是 LeRobot 的 `from_pretrained` 包。

## 启动

本机环境已经安装。项目根目录运行：

```bash
bash src/infer_after_sft/run.sh
# 分类快捷入口，等价：
bash scripts/inference/启动微调Pi05真机.sh
```

模型加载一次 → 机械臂到共用预抓取位 → 夹爪逐步张开至约 78 mm 并确认到位 → 连续读取相机/状态并执行关节和夹爪动作。默认任务是把蓝色方块放到其后方、远离机械臂的白色区域，默认运行 60 秒；Ctrl+C 停止并保持，不自动回零。预抓取移动途中保留当前夹爪开度，到位后以约 30 mm/s 张开，随后由策略决定夹爪目标。每次启动只在首次预测前复位；连续重规划使用实时姿态，不会每个动作块都回预抓取位。

```bash
# 指定时长或任务
bash src/infer_after_sft/run.sh --seconds 120 --prompt 'Pick up the blue cube and place it in the white area.'

# 移动到预抓取位并张开夹爪后预测，不执行模型动作
bash src/infer_after_sft/run.sh --preview

# 指定另一份同格式 checkpoint（自动读取其中的配置与统计）
bash src/infer_after_sft/run.sh --checkpoint /path/to/checkpoint/5000
```

`--preview` 和正常连续入口都会先移动到预抓取位并张开夹爪；`--preview` 只跳过模型动作执行。

## 卡住时停止和回零

另开终端：

```bash
bash src/infer_after_sft/停止.sh
bash src/infer_after_sft/回零.sh
bash src/infer_after_sft/预抓取.sh
```

分别等价于 `scripts/inference/停止Pi05.sh` 和 `scripts/arm/机械臂回零.sh`。独立停止程序已经识别本目录的 `run.py`、`worker.py`，不依赖模型加载或 GPU 响应，不结束机械臂服务。

`预抓取.sh` 同样先退出策略进程，再到共用预抓取位并逐步张开夹爪至约 78 mm，最后保持。回零保持当前夹爪开度。两个定位入口复用采集的 ArmClient → 服务 Ruckig → 硬件控制链，并直接读取 `启动遥操.sh` 中的数字运动参数（不执行遥操脚本、不连接头显）。当前回零速度上限为前段 2.0、后段 0.75 rad/s，预抓取为 2.0 rad/s，预览时间 0.05 s，关闭目标超前裁剪；桌面保护和关节边界由服务继续执行。Ctrl+C 停止并保持。这里的独立定位参数不改变模型执行速度。

## 与训练一致的推理路径

- 从 `metadata.pt` 读取 `pi05_x5_pytorch` 配置：32 维模型动作、15 步 horizon、200 个文本 token、bfloat16、离散状态输入，不启用编译。
- `metadata.pt` 采用限制性读取器，只接收基础配置和一个忽略的训练 filter 占位符，不导入 checkpoint 指定的任意 Python 类；不加载 `optimizer.pt`。
- 使用 OpenPI `PI0Pytorch` 和 `safetensors.load_model(..., strict=True)`，严格检查权重，保留 tied-weight 关系；不允许失败后返回随机模型。
- 使用 checkpoint 自带的 `assets/local/x5_blue_cube_to_white/norm_stats.json`；启动时快照统计并校验 worker 使用的哈希相同，不生成或替换成 zero-shot 的实验统计。
- 用户已确认训练沿用了本项目 `integrations/openpi/x5_policy.py` 适配器，因此直接复用它：单腕 RGB → `left_wrist_0_rgb`，缺失相机显式 mask；6 个关节增量相对观测状态，夹爪是绝对米。
- 输入顺序与训练一致：X5 适配 → delta 变换（推理无 actions 时为空操作）→ quantile 归一化 → 图像 resize/pad → **7 维状态进入 tokenizer** → 状态补到 32 维。
- 输出由 OpenPI 做反归一化、6 关节增量恢复为绝对目标，最后取 7 维。没有套用旧 LeRobot 基座的动作解码。

checkpoint 未记录服务器 OpenPI 源码 revision；本地固定 OpenPI `215abfb217dbac7d5f1273282331b9b1866c0479`，已通过权重严格加载和训练变换往返检查。其所需的 Transformers 4.53.2 替换文件只安装到独立 SFT 环境。

## 控制与日志

复用 `src/zero-shot/rollout.py` 的设备控制循环和服务客户端，推理后端独立。每轮预测 15 步，默认执行前 10 步（30 Hz）后读取新观测，夹爪跟随预测。可用 `--chunk-steps 1..15` 调整。

速度默认 2.0 rad/s，可用 `--speed` 调整，最大 2.0；模型执行加速度上限为 1.0 rad/s²，jerk 上限为 4.0 rad/s³。这组提速参数待真机验证，启动时打印并保存到 `motion.json`。夹爪目标变化率默认 0.03 m/s，可用 `--gripper-speed` 调整，最大 0.05。关节软限位、目标超前限制、桌面保护、反馈/图像新鲜度检查和独占控制租约继续生效。其他控制者占用时不抢占。控制/推理异常时停止保持；硬件服务故障不能由模型程序保证恢复。

状态统计阈值可用 `--state-quantile-limit 1.1` 调整，微调入口默认已从共享入口的 0.5 放宽到 1.1。超出量定义为 `max(max(abs(2*(state-q01)/(q99-q01+1e-6)-1)-1, 0))`；1.1 约允许在统计区间两端各延伸区间宽度的 55%。该 checkpoint 夹爪统计范围为 26.6～77.9 mm，夹爪完全闭合的超出量约 1.038，因此原阈值会拦住正常闭合状态。放宽只改变执行前的统计检查，不修改训练归一化或动作解码，也不代表模型在统计范围外的效果已验证。非有限值仍拒绝执行。每轮打印超出量，`prediction.json` 记录各维超出量，`rollout.json` 记录阈值。

结果在 `outputs/infer_after_sft/<时间>/`：

- `checkpoint.json`、`norm_stats.json`：本轮的配置与统计快照。
- `preview/` 或 `cycle_NNNN/`：图像、带时间戳的观测、`prediction.json`、`prediction.npz`。
- NPZ 中 `actions_x5` 是反归一化后的 **15×7 绝对关节角 rad / 夹爪宽度 m**，不是最终限幅命令。
- 连续运行的 `commands.jsonl` 是实际发送给服务的关节/夹爪目标，`rollout.json` 记录结束原因和最终状态。到时结束不等于任务成功，`task_success` 不自动置为 true。

## 环境与验证

```bash
bash src/infer_after_sft/setup.sh
PYTHONPATH=src/infer_after_sft outputs/envs/pi05-sft/bin/python -m unittest discover -s src/infer_after_sft -p 'test_*.py' -v
```

环境在 `outputs/envs/pi05-sft`，不改遥操和旧 zero-shot 环境。使用 CUDA 12.8 PyTorch 支持 RTX 5090；JAX 仅作为 OpenPI 的 CPU 辅助依赖。首次安装需要网络，正常推理使用本地权重、统计和已校验的公开 PaliGemma 分词器。`--help` 不加载模型、不连接硬件。

已验证 checkpoint 严格加载、真实相机观测得到 15×7 输出（首轮约 384 ms），以及 4 项 metadata/变换检查和共享控制循环的 3 项模拟服务检查。完整真机抓放效果尚未验证。

## JAX 权重推理

```bash
bash src/infer_after_sft/setup_jax.sh
bash src/infer_after_sft/run_jax.sh
# 指定权重目录或 step 目录
bash src/infer_after_sft/run_jax.sh --checkpoint ~/work/checkpoints/blue_cube_jax_v1/15000
```

默认使用 `~/work/checkpoints/blue_cube_jax_v1` 下数字最大的、含 `params/_METADATA` 的 step，目前为 15000。原生恢复 Orbax `params`，严格校验模型参数树和形状，读取该 step 自带的 X5 统计；不加载 `train_state`。JAX/CUDA 运行在独立环境 `outputs/envs/pi05-jax`，不修改 PyTorch 环境。

这个 Orbax checkpoint 未保存训练配置，因此使用项目 X5 适配器的 PI05 配置：32 维模型动作、15 步 horizon、200 token、6 个关节增量和绝对夹爪宽度。如果训练时改了 horizon，使用 `--jax-horizon N`；horizon 不能从权重形状可靠推断，必须与训练一致。每块执行步数仍由 `--chunk-steps` 控制，默认 10。

首次 JIT 编译用虚拟观测预热，在连接机械臂之前完成（最多等待 600 秒）；之后每轮耗时包含 JAX 同步和输出转换。正常入口会到预抓取位、张开夹爪再执行；`--preview` 也会定位，但不执行模型预测。停止和回零继续使用本目录原有脚本。

本机已验证：5090 上恢复 15000 步权重、完成 JIT 预热，使用已保存的真实观测连续推理两次均输出有限的 15×7 动作，耗时约 63 ms、62 ms（包含同步；仅两次样本，不是完整基准）。结果在 `outputs/diagnostics/jax-verify-20260911-161112/`。尚未验证此 JAX checkpoint 的完整真机抓放效果。


### 预抓取失败与停止状态

模型入口的预抓取现在也读取 `启动遥操.sh` 的采集运动参数，与 `预抓取.sh` 一致；模型执行仍使用其独立限速。启动记录 `pregrasp_motion.json`。服务存在时，在加载模型前先检查控制权，`stopping` 或其他控制者占用会直接退出，状态快照保存在 `service_before.json`。控制失败时尽量保存 `service_error.json`。

`实际关节与服务规划偏差超过 0.15 rad` 是硬件跟踪偏差，不是 JAX 推理超时。如果停止轨迹已完成但实际位置仍与停止目标差距较大，服务会保持 `stopping`，此时不能宣称已经保持或释放。不要直接重跑推理；先检查是否受阻及硬件反馈。需要重启服务时，应先支撑机械臂，避免关闭控制后失去支撑；退出原机械臂服务，再运行 `bash 启动机械臂服务.sh`。软件配置修正不能证明硬件跟踪问题已消失。

`停止.sh` 现在支持异常 `stopping` 恢复：仅针对已失联的 PI05 控制会话，在规划刹车完成、最近 0.5 秒反馈速度低于 0.03 rad/s 且位置变化低于 0.01 rad 时，将保持目标设为实际位置并释放原控制权，全程不切换到 damping。保留原偏差的终端报告。该功能需要新版机械臂服务；旧进程必须重启一次才能使用，脚本遇到旧版会明确报错，不会伪报已停止。仍在运动、其他控制者或不新鲜的反馈均不能用此路径恢复。
