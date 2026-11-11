# 远程 π0.5 策略（双臂 14 维）

模型跑在 GPU 服务器上，机器人这边只做「观测 → 请求 → 执行」。控制台「策略」页负责配置与启停，实际客户端是内核侧的一个策略进程。

## 1. 服务端（GPU 机器）

```bash
cd /home/qyj/openpi && CUDA_VISIBLE_DEVICES=5 setsid /home/qyj/openpi/.venv/bin/python \
  /home/qyj/serve_sweep_dual.py > /home/qyj/logs/serve_sweep_dual.log 2>&1 < /dev/null &
```

等日志出现 `warmup base`、`warmup rtc`、`listening on 8000` 后才可用（预热约 25 s + 53 s，与 GPU 有关）。

- 停止：`pkill -f serve_sweep_dual.py`
- 端口：默认 8000，可用 `PORT` 环境变量改
- 本机转发：`ssh -N -L 8000:127.0.0.1:8000 qyj@192.168.96.202`（客户端不在 GPU 机器上时）

训练产物与验证结论见交接文档 `dual_arm_pi05_interface_20261106.md`。

## 2. 协议

- websocket，`ws://<host>:8000`；msgpack + numpy 扩展（`msgpack_numpy` 格式）
- 连接后服务端先发一条 metadata（本模型为 `{}`），之后一问一答

观测（客户端 → 服务端）：

| 键 | 形状 | 含义 |
|---|---|---|
| `state` | float32 (14,) | left_joint1..6、right_joint1..6、gripper_left、gripper_right |
| `wrist_rgb` + `_valid` | uint8 (480,640,3) | 右腕相机 |
| `wrist_second_rgb` + `_valid` | uint8 (480,640,3) | 左腕相机 |
| `exterior_rgb` + `_valid` | uint8 (480,640,3) | 固定外部相机 |
| `prompt` | str | 固定任务文本 |

RTC 可选字段（同时出现才走引导采样）：`prev_chunk_left_over` (N,14) float32、`inference_delay` int32、`execution_horizon` int32、`rtc_guidance_weight` float32、`rtc_prefix_schedule` str。

动作：`{"actions": float32 (50,14)}`，30 Hz；前 12 维绝对关节目标（rad），后 2 维夹爪宽度（m）。执行侧按内核限位与夹爪最大宽度裁剪。

## 3. 机器人端

策略页（控制台 `/policy`）：

1. 模式选「远程 π0.5 服务（双臂 14 维）」；
2. 填服务端地址（直连 `ws://192.168.96.202:8000` 或隧道后的 `ws://127.0.0.1:8000`）；
3. 填任务文本；可选 RTC（块大小 / 重推理剩余 / 混合步数 / 推理时延，见下）与「只推理不下发」；
4. 运行时接管内核控制权——VR 遥操占用时会拒绝，先停遥操。

运行前可以点「准备位」把两臂送到演示数据的起始位姿（`configs/control/ready_pose.json`：左臂零位、右臂 `[-0.29, 0.81, 0.15, 0.53, -0.46, -0.18]`），让策略从中分布内的状态开始；「回参考位」仍是内核 park 位。两个按钮都需要连点两次确认，速度默认 25%。

命令行等价：

```bash
./run.sh policy remote --run outputs/runtime/control/<RUN> \
  --server ws://127.0.0.1:8000 \
  --prompt "sweep the garbage on the table into the yellow dustpan and pour it into the trash bin" \
  --seconds 60 [--rtc --chunk-steps 50 --reinfer-remaining 30 --blend 16 --guidance 10] [--dry-run]
```

RTC 参数与页面上的四个滑杆一致：

| 参数 | 含义 | 是否可调 |
|---|---|---|
| `chunk_steps` | 每块执行多少步 | **训练固定 50**，页面只读 |
| `reinfer_remaining` | 队列剩多少步时请求新块 | 可调，默认 30 |
| `blend` | 新块开头多少步与旧块线性混合（客户端平滑）；切换发涩就加大，跟不上就减小 | 可调，默认 16 |
| `guidance` | 引导权重 `rtc_guidance_weight`：越大越贴近旧前缀 | 可调，默认 10 |
| `inference_delay` | 推理时延（tick） | **运行时读取**（取最近一次 chunk 的实测时延），页面只读 |
| `execution_horizon` | 引导前缀覆盖到第几步（之后自由采样） | 运行时算：`inference_delay + 10`（`runner.GUIDANCE_TAIL`），与下面的对齐配套 |

页面上的时序图按这四个数画出：白条＝新预测、斜纹＝混合段、细条＝推理期间已执行、虚线＝切换点、红叉＝队列空转。
RTC 开启后推理在后台线程进行，新块一到就切换（不再等队列排空）。

**交接点对齐**：新块的第 0 步对应「发请求那一刻」的状态（实测与录制动作首步差 0.007 rad），
而收到时机械臂已经执行了 `arrival_delay` 步。客户端因此丢弃新块开头这 `arrival_delay` 步
（日志 `skipped_steps`），再从对齐的位置接手。

**引导窗口要盖过这段延迟**：服务端只把新块的前 `execution_horizon` 步往旧块上前缀拉（探针实测：
`delay>0` 时第 0..horizon-1 步贴合前缀，之后自由生成）。若 `execution_horizon` 小于延迟，
跳过延迟后正好落在自由采样区——每次重规划都换成一段新的随机采样，表现为机械臂突然反向或加速。
因此 `execution_horizon = inference_delay + 10`（当前延迟 ~20 → ~30）。

- 三路相机默认 `wrist_rgb,wrist_second_rgb,exterior_rgb`，顺序固定（右腕、左腕、外部）；
- `--dry-run` 只请求不执行，用来检查联通与动作形状；
- 运行产物写到 `outputs/policy_remote/<时间戳>/`：`commands.jsonl`（每个动作索引一行，含插值前的目标）与 `summary.json`（chunk 数、时延、状态）。

**执行节奏**：动作块本身是 30 Hz，客户端按「当前动作步的推进比例」在相邻目标之间线性插值，以 `SEND_HZ=250`
（实际受 RPC 限制约 140 Hz）下发，内核拿到的是一条连续的关节轨迹而不是 33 ms 一个台阶。时间基准每轮按
墙钟重算，发送慢了不会补偿式突发；块切换时从「上一个实际下发的点」继续插值，所以边界不跳变。
一次动作被内核拒绝只计一次 `rejected`，该动作剩余的子步跳过。

## 4. 代码位置与扩展点

| 位置 | 内容 |
|---|---|
| `src/embodied/policy/remote/wire.py` | msgpack + numpy 编解码（与 `msgpack_numpy` 格式一致） |
| `src/embodied/policy/remote/client.py` | 连接、metadata、单次推理、动作形状校验 |
| `src/embodied/policy/remote/runner.py` | 观测循环、RTC 重规划、按臂夹爪下发 |
| `src/embodied/control/commands.py` | `joint` 命令新增 `gripper_targets`（双臂各一维） |
| `src/embodied/applications/console/operations.py` | 策略操作 `mode=remote` 的参数校验与命令拼装 |

扩展方式：

- **换模型/换服务端**：只要仍是一问一答的 msgpack 接口，改页面上的地址即可；字段不同时在 `runner.build_observation()` 里加映射，`client.infer()` 的形状校验按新契约调整。
- **换机器人**：`build_state()` 目前写死 12 关节 + 2 夹爪；单臂或其它厂商在这里改映射，内核侧不需要变。
- **异步 RTC**：现在是同步节奏（执行到 `execution_horizon` 步后用剩余块重规划）。要做真正并发的 RTC，把推理放到独立线程、动作队列由执行线程消费，`client` 接口不变。

## 5. 验证

离线（无需真机）：

```bash
PYTHONPATH=src configs/environments/arx_motion/.venv/bin/python -m pytest tests/motion/test_policy_remote.py -q
```

含：编解码往返、14 维状态构造、限位裁剪、对假服务端的握手与单次推理、错误动作形状拒绝。

真机顺序建议：`--dry-run` 确认时延与形状 → 短时（10 s）真跑，人工扶臂 → 正常时长。切到真跑前确认周围安全、急停在手边。
