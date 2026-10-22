# π0.5 base 的 X5 zero-shot 离线探测

读取 `/home/r2/work/models/pi05_base` 的 LeRobot safetensors 权重，用真实腕部 RGB、6 关节角和夹爪宽度生成动作块。无训练。`run.sh` 为离线探测；`real.sh` 提供预抓取定位、现场推理及显式选择的单步实验。

## 一键连续真机运行

在项目根目录执行：

```bash
bash scripts/inference/start_pi05_zeroshot.sh
```

默认任务：把蓝色方块放到它后方、远离机械臂的白色区域。加载模型一次后自动连接机械臂服务、到达共用预抓取位，然后连续运行 **60 秒**。每轮拍摄新图像，预测 50 步、执行前 10 步（30 Hz），再拍摄重规划。关节和夹爪都由预测驱动；Ctrl+C 停止并保持，不回零。模型约一分钟加载完成后才开始定位和连续运行计时。

```bash
bash scripts/inference/start_pi05_zeroshot.sh --seconds 120
# 可选：任务、关节速度上限和每次执行步数
bash scripts/inference/start_pi05_zeroshot.sh --prompt 'Pick up the blue cube and place it in the white area.' --speed 0.3 --chunk-steps 10
```

默认关节速度请求上限 0.3 rad/s、加速度 0.5 rad/s²、jerk 2 rad/s³，目标相对实际关节最大超前 0.08 rad。输出先裁剪到带 0.05 rad 余量的关节范围，再限制超前量。夹爪目标限定为设备允许宽度，最大变化率 0.03 m/s；实际接触后的力矩保持由已有机械臂服务处理。模型推理等待期间服务减速，客户端持续更新控制租约，不重复播放旧动作块。

原有桌面保护、单控制者互斥和反馈检查继续生效。预测明显超出统计覆盖（归一化范围外 >0.5）、服务异常或动作观测过期时结束本轮并保持。运行时间最大 300 秒，每轮执行 1..15 步；这是真机实验限制，不是任务完成判定。`--help` 不连接设备。

日志位于 `outputs/zero-shot/rollout-<时间>/`：逐轮图像/状态/完整预测，`commands.jsonl` 的真实发送目标与裁剪信息，以及 `rollout.json` 的结束原因。`task_success` 保持 null，是否完成抓放由现场判断。统计缺失时自动从原始采集计算；机械臂服务不存在时自动启动，已有其他控制者时拒绝抢占。

这份入口已通过模拟服务的多轮控制、夹爪变化、异常停止与到时保持测试，并检查同一真实 PI05 进程可连续响应多个观测。完整真机抓放成功率尚未验证。

## 安装和运行

在项目根目录执行：

```bash
bash src/embodied/policy/pi05_zeroshot/setup.sh
bash src/embodied/policy/pi05_zeroshot/run.sh \
  --episode ~/work/data/quest_x5/.raw/X5_遥操采集_20260911-035453-87d1b692 \
  --frames 0 30 60 \
  --prompt 'Pick up the blue cube and place it in the white area.'
```

环境放在 `outputs/envs/pi05-zero-shot`，与遥操和旧版 LeRobot 导出环境隔离。首次安装需要网络，下载 CUDA PyTorch、固定 revision 的 LeRobot，以及 OpenPI 使用的公开 PaliGemma SentencePiece 文件。权重从本地读取，分词器缓存后推理不需要网络。默认 CUDA + bfloat16、10 次去噪；可通过 `--device cpu --dtype float32` 选择 CPU。`PI05_PYTHON` 可指定已有兼容环境。

也可使用单张腕部图片：

```bash
bash src/embodied/policy/pi05_zeroshot/run.sh --image /path/to/wrist.png \
  --state 0 1.5 1.45 -1.52 0 0 0.078 \
  --prompt 'Pick up the blue cube and place it in the white area.'
```

`--state` 顺序为 joint1…joint6（rad）和实际夹爪宽度（m）。`--frames` 为`episode.mcap` 中 RGB 索引主题从 0 开始的消息序号。图像只匹配其接收时间之前最新状态，超过 100 ms 则拒绝，不用未来状态。输入图像由 BGR 转 RGB，模型自行保持比例缩放与填边。

## 结果与解读

默认每次创建 `outputs/zero-shot/<时间>/`，也可 `--output /path/to/new-directory`，拒绝覆盖已有目录。

- `report.json`：参数、输入来源/时间戳、加载时间、每个动作块耗时、GPU 峰值分配量、输出范围、状态被裁剪的维度。
- `sample_NNN.png`：对应输入图像。
- `sample_NNN.npz`：`actions_model_space`（50×32）、`state_physical`（7）、`state_model_input`（32）。用 `numpy.load(..., allow_pickle=False)` 读取。

每个样本重置相同随机种子（默认 0），便于固定图片比较不同 prompt；首个推理耗时包括 GPU 预热成本。支持 `--seed`、`--steps` 和 `--dtype`。报告中的前 7 维只是观察窗口，没有声明为 X5 的 7 个动作。

**当前基座文件未提供 X5 的训练统计和动作语义。离线生成数值不代表能完成抓取，也不能据此报告任务成功率。**

默认 `--state-mode joint-limits` 将 X5 URDF 的物理关节范围和 `[0, 最大夹爪宽度]` 映射到 `[-1,1]`，再裁剪并补到 32 维。这只是公开记录的探测假设，不等同于预训练的 quantile 归一化。`--state-mode raw` 可做消融：原始 rad/m 直接裁剪到 `[-1,1]`。两者都不能证明 X5 与预训练机器人一致。

腕部图像放入 `left_wrist_0_rgb` 槽位；其余相机键省略，由 LeRobot 创建缺失图像 mask。单臂到左腕槽位也是适配假设。分词完全采用 OpenPI 的 π0.5 格式：清洗任务文本、256 桶状态离散化、BOS 和右侧零填充，超过长度时显式报错。

输出保留模型空间，不做缺少依据的反归一化、delta/absolute 转换或真实控制。要评价 X5 实际任务完成能力，还需确定动作维度/单位、状态统计与视角适配，再部署闭环评测；基座模型卡将其定位为针对具体场景微调的起点。

## 本机实测

2026-09-11，RTX 5090，PyTorch 2.7.1+cu128，bfloat16、10 次去噪，使用上述 episode 的第 0/30/60 帧与指令 `Pick up the blue cube.`。三个动作块均为有限值的 50×32 数组。模型加载 59.2 秒；首块 0.609 秒，后两块分别 0.133/0.133 秒；PyTorch 峰值分配 8.87 GiB（不等于整卡显存占用）。单次小样本测量，不是任务成功率评测。

实际结果保存在 `outputs/zero-shot/blue-cube-cuda128/`。当前测试只证明本地权重可在真实观测上完成推理，未验证抓取效果。

## 检查

```bash
./run.sh test tests/policy          # 映射、统计、推理与旧链路入口（主环境）
bash src/embodied/policy/pi05_zeroshot/run.sh --help
```

模型加载使用严格 state_dict 匹配；缺失/不兼容权重直接失败，避免上游 `from_pretrained` 捕获异常后返回未加载模型。只使用所固定 revision 的私有键迁移方法，因此升级 LeRobot 时需要重新验证。

参考：[LeRobot π0.5](https://huggingface.co/lerobot/pi05_base)、[OpenPI 分词器实现](https://github.com/Physical-Intelligence/openpi/blob/main/src/openpi/models/tokenizer.py)。

## 真机预抓取与现场观测

启动机械臂服务（单独终端）：

```bash
bash scripts/shortcuts/启动机械臂服务.sh
```

查看真机状态（只读）：

```bash
bash src/embodied/policy/pi05_zeroshot/real.sh
```

实际低速移动到统一预抓取位置，然后采集现场图像并运行 π0.5：

```bash
bash src/embodied/policy/pi05_zeroshot/real.sh --move-pregrasp --infer --prompt 'Pick up the blue cube and place it in the white area.'
```

只移动到位可省略 `--infer`；已经处于预抓取保持状态时可只用 `--infer`。上述命令只执行预抓取预设，模型预测不下发。下文的 `--execute-step` 是独立选择的单步实验入口。

统一姿态来自 `robot_core/presets.py`，与 Quest 遥操共用 `[0, 1.50, 1.45, -1.52, 0, 0]` rad。真机入口请求关节速度上限 0.15 rad/s、预设加速度 0.3 rad/s²、jerk 1 rad/s³；夹爪保持起始开度，报告记录实际值。规划上限不等于每次电机反馈的测量速度。服务执行 Ruckig 规划、关节边界、桌面保护和控制权互斥；已有控制者时先退出其控制会话，入口不会抢占。

到位判定为最大关节误差 <0.03 rad、反馈速度 <0.05 rad/s，持续 0.5 秒。超时、桌面拒绝或规划跟踪偏差 >0.15 rad 时停止。Ctrl+C 请求服务减速停止并保持，不自动回零。服务故障/连接失效时无法保证客户端完成停止，应检查设备实际状态。到位后服务保持姿态，进程退出也不撤销此保持。

现场观测保存在 `outputs/zero-shot/live-<时间>/`：`wrist.png`、含机器人/相机/时间戳的 `observation.json`，以及 `prediction/`。拍摄时检查预抓取姿态、静止状态和图像/状态时间差（≤100 ms）；这是静止姿态下的观测，不是运动中的时序对齐。硬件客户端用遥操环境，模型由独立推理环境运行。

2026-09-11 已实机执行从接近零位到预抓取位，服务保持成功、桌面保护未触发，最大关节误差约 0.011 rad。现场图像中可以看到蓝色方块；该测试没有执行模型抓取。

真机入口的设备隔离测试：

```bash
./run.sh test tests/policy/test_zeroshot_real.py
```

## 使用 X5 分位数统计做一次小幅策略试探

这是一套明确的实验映射：把基座前 6 维解释为 X5 的关节增量，第 7 维解释为夹爪绝对宽度。统计来自本地演示，不是基座的预训练统计；没有对权重进行训练，不保证模型已学会这个机器人。

先用遥操环境计算统计（无需解码视频、无需 LeRobot/PyTorch）：

```bash
PYTHONPATH=src .venv/bin/python -m embodied.policy.pi05_zeroshot.compute_stats \
  --output outputs/zero-shot-assets/x5_quantiles_h50.json
```

本机该文件已经生成。计算器拒绝覆盖已有文件；重新计算时选择新文件名。读取 `~/work/data/quest_x5/.raw`，复用支持当前压缩记录格式的训练导出加载器检查完整性、时间戳、动作与图像引用，并校验 URDF/关节范围。排除丢弃、失败、非驱动记录和少于 50 个锚点的 episode。包含 `unknown` 条目仅用于实验统计，不能当作成功演示。

每个真实图像选此前最新状态/请求动作，形成约 30 Hz 的锚点。每个锚点取后续 50 个动作，六关节都减去该锚点的实际关节角，夹爪保持绝对值；episode 尾部重复最后动作，绝不跨 episode。50 步对应这份基座的 chunk_size（约 1.67 秒），与现有微调配置的 15 步不同。由此分别计算状态和动作 q01/q99；文件记录来源哈希、单位、URDF、纳入/排除原因与样本量。

只查看按统计解码的实时预测：

```bash
bash src/embodied/policy/pi05_zeroshot/real.sh --move-pregrasp --infer \
  --stats outputs/zero-shot-assets/x5_quantiles_h50.json \
  --prompt 'Pick up the blue cube and place it in the white area.'
```

**实际执行一个受限关节目标：**

```bash
bash src/embodied/policy/pi05_zeroshot/real.sh --move-pregrasp --execute-step \
  --stats outputs/zero-shot-assets/x5_quantiles_h50.json \
  --prompt 'Pick up the blue cube and place it in the white area.'
```

执行流程是预抓取到位保持 → 加载模型 → 拍摄新观测 → 推理 → 解码第一个动作 → 限幅 → 低速执行 → 停稳保持。只取动作块第一步，没有执行其余 49 步，没有循环 rollout。夹爪保持拍照时的开度，预测夹爪值仅记录，不执行闭合。

约束固定在代码中：

- 原始单步关节增量 >0.5 rad 直接拒绝；其余向量按同一比例缩小，使每关节目标变化 ≤0.02 rad、指尖目标平移 ≤0.01 m，不放大小动作。
- 关节速度请求上限 0.05 rad/s，加速度 0.15 rad/s²，jerk 0.5 rad/s³；实际位移 >0.05 rad、跟踪异常或桌面拒绝时停止。
- 下发时图像年龄 ≤2 秒、机械臂与观测偏差 ≤0.01 rad、服务实例一致且静止。获取独占控制权后再次核对，不复用旧预测。
- 分位数不是物理边界。允许归一化值超出 `[-1,1]` 至多 0.1 后裁剪（即统计区间跨度的 5%）；更大的超出拒绝执行，裁剪维度与超出量写入报告。
- Ctrl+C 或执行异常请求减速停止并保持；不自动回预抓取位。

`prediction.npz` 保存原始 50×32 动作、实验解码的 50×7 动作与模型输入状态；`prediction.json` 保存统计哈希、原始解码首步、裁剪情况与耗时。`execution.json` 保存限幅比例、实际发送目标、最终关节位置、实际位移与完成状态，`execution_trace.jsonl` 保存执行过程。`completed` 仅表示单次运动结束，不表示抓取成功。

全套测试在遥操环境执行，控制测试使用临时 socket 和模拟机械臂：

```bash
PYTHONPATH=src:tests .venv/bin/python -m unittest discover -s src/embodied/policy/pi05_zeroshot -p 'test_*.py' -v
```

2026-09-11 单步实测：109 条记录、21,115 个图像锚点生成统计，7 条排除。使用指令 `Pick up the blue cube and place it in the white area.`，成功下发一次限幅策略关节目标，实际最大关节位移约 0.0156 rad，结束后服务保持。结果在 `outputs/zero-shot/live-20260911-052632/`。未闭合夹爪、未完成抓放；腕部图像中尚不能明确辨认白色目标区域。

## 模型卡住时停止与回零

另开终端，进入项目目录即可运行；不依赖 GPU 或模型推理进度：

```bash
# 结束本项目策略进程，机械臂低速回到六关节零位并保持
bash scripts/arm/home.sh

# 只结束策略并保持当前位置，不回零
bash scripts/inference/stop_pi05.sh
```

正常运行时先用 Ctrl+C。终端无响应时用上述独立入口：先给策略控制进程 SIGTERM，让其执行停止保持；仍无响应时仅结束本项目匹配的 Python 策略/推理进程，并等待控制租约释放后重新建立保持。不会使用 `killall python`，不会终止机械臂服务、相机服务或其他任务。

回零复用 `home` 服务轨迹：前段速度上限 0.3 rad/s、后段 0.15 rad/s，保留夹爪开度，关节与桌面保护继续生效。其他控制者（例如 Quest）占用时拒绝抢占；先退出对应控制会话。机械臂服务自身不可用时此入口会报错，不会通过重启硬件服务强行恢复。若服务确实已退出，先单独启动 `bash scripts/shortcuts/启动机械臂服务.sh`，再运行回零脚本。

只回零功能已通过模拟服务测试；这不是硬件急停，服务或通讯故障时应使用设备物理急停。
