# 远程调用策略（本机作为服务端）

外部程序（另一台机器、脚本、评测任务）只需要给 **任务描述** 和 **时长**；模型地址、RTC
参数和 dry-run 由本机在策略页选定后保存。运行与你手动点「运行策略」完全一致。

## 本机准备（一次性）

策略页 → 选模型卡片 → 「保存当前设置为远程默认」。等价于：

```bash
curl -X PUT http://<本机IP>:8000/api/v1/policy/remote/config \
  -H 'content-type: application/json' \
  -d '{"model_id":"towel_v2_local","rtc":true,"chunk_steps":50,
       "reinfer_remaining":30,"blend":16,"guidance":10,"dry_run":false}'
```

前提：内核已启动、控制权空闲（遥操或手机占用时会返回 409）。

## 对方发起运行

```bash
curl -X POST http://<本机IP>:8000/api/v1/policy/remote \
  -H 'content-type: application/json' \
  -d '{"prompt":"Fold the towel on the table.","seconds":60}'
```

```json
{"state":"running","prompt":"Fold the towel on the table.","seconds":60,
 "model":{"id":"towel_v2_local","name":"叠毛巾 v2 · 本地 5070","url":"ws://127.0.0.1:8008"},
 "operation":{"kind":"policy","state":"running","log":"policy-20261117-160000.log"}}
```

`seconds` 省略时默认 60，范围 1..300；`prompt` 1..500 字符。

## 对方获取执行反馈

```bash
curl http://<本机IP>:8000/api/v1/policy/remote
```

| 字段 | 含义 |
| --- | --- |
| `state` / `running` | `idle` / `running` / `stopped` / `failed`，以及是否在跑 |
| `progress.chunks` | 已执行的块数 |
| `progress.rejected` | 被内核拒绝的动作数（桌面平面、限位等） |
| `progress.last_chunk_latency_ms` | 最近一次推理时延 |
| `events` | 运行日志最近 20 条事件：`connected` / `chunk` / `rejected` |
| `kernel` | 实时反馈：`phase`、控制权 `owner`、12 维关节 `q`、每臂夹爪 `gripper_widths` |
| `run.directory` | 本次产物目录（`commands.jsonl`、`summary.json`） |
| `run.summary` | 结束后才有：块数、命令数、拒绝数、结束原因 |

轮询示例：

```python
import time, requests

run = requests.post(f"{base}/api/v1/policy/remote",
                    json={"prompt": "Fold the towel on the table.", "seconds": 60}).json()
while True:
    status = requests.get(f"{base}/api/v1/policy/remote").json()
    if not status["running"]:
        print(status["run"].get("summary"))
        break
    time.sleep(1)
```

## 提前结束

```bash
curl -X POST http://<本机IP>:8000/api/v1/policy/remote/stop
```

## 错误码

| 码 | 原因 |
| --- | --- |
| 400 | `prompt`/`seconds` 不合法，或本机还没选定远程模型 |
| 409 | 内核控制权被遥操/手机占用，或已有策略在运行 |
| 503 | 读不到内核状态 |

控制台以 `--token` 启动时，所有请求加 `?token=<配对令牌>`。
