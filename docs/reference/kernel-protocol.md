# 内核命令协议（commands.sock）

所有客户端（工作台、手机、动作、标定、策略、网页控制台）经 `control/run/<RUN>/commands.sock` 访问内核。每个连接发送一条长度前缀 JSON 请求，返回 `{"ok": true, "result": …}` 或 `{"ok": false, "error": "<类型>: <说明>"}`。Python 客户端用 `embodied.contracts.runtime.control.ControlClient` / `ControlSession`，不需要自己拼请求。

- 单位：关节 rad，长度 m，位姿 `[x, y, z, qx, qy, qz, qw]`（`base_link` 中的 TCP `tcp_link`），时间为主机 CLOCK_MONOTONIC 纳秒。
- 租约：同一时刻只有一个拥有者；租约在最后一条带租约请求后 0.5 s 过期，`status` 不续租。任何撤销（过期、release、stop、接管）都使 `epoch` 加一，之前的 token 立即失效。
- 安全由内核统一负责：关节限位、桌面平面、单步与导数界限、0.15 m 笛卡尔领先距离、目标过期后保持，对所有来源一致。
- 金样本：`tests/golden/kernel/samples.json`（每个动作一条真实请求与结果，长列表已截短）；`./run.sh check all` 的 `sim` 组用 `scripts/check_kernel_protocol.py` 在 MuJoCo 内核上逐个动作核对，`--update` 重新生成样本。
- 观测流（`observer.sock`，RobotTickV2）为只读遥测，见 `control/adapters/robot_tick_v2.json` 与 `embodied.contracts.runtime.RuntimeReader`。

下面的表由 `embodied.contracts.protocol` 生成（`PYTHONPATH=src python3 -m embodied.contracts.protocol`）。

<!-- protocol:start -->
| Field | Type | | Meaning |
| --- | --- | --- | --- |
| `version` | int | required | always 1 |
| `runtime_id` | str | required | runtime.json runtime_id; a request for another run is refused |
| `action` | str | required | one of ACTIONS |
| `sent_monotonic_ns` | int | required | client send time; queued requests older than 250 ms are refused |

Leased actions also carry:

| Field | Type | | Meaning |
| --- | --- | --- | --- |
| `token` | str | required | from acquire |
| `epoch` | int | required | from acquire; any revocation increments it |
| `sequence` | int | required | strictly increasing per lease |

### `describe`

Robot limits, control rate, park pose, gripper range, CAN interface, model id, workspace; no feedback needed.

Result: `protocol`, `runtime_id`, `model_id`, `robot`, `workspace`, `actions`

### `status`

Latest measured state, owner and operations; clients treat feedback older than 150 ms as stale.

Result: `runtime_id`, `model_id`, `epoch`, `owner`, `last_revocation`, `lease_remaining_s`, `stopped`, `phase`, `q`, `dq`, `pose`, `gripper_width`, `feedback_monotonic_ns`, `operations`, `ingresses`

### `acquire`

Take the single control lease; refused while owned or while the arm moves (>0.05 rad/s).

| Field | Type | | Meaning |
| --- | --- | --- | --- |
| `source` | str | required | ASCII identifier, at most 31 characters |
| `metadata` | object | optional | JSON object, at most 8 KiB, recorded in source-metadata.json |
| `ingress` | str | optional | rpc (default) or an installed native ingress |
| `purpose` | str | optional | operation (default) or teleop |

Result: `source`, `token`, `epoch`, `ingress`, `purpose`, `q`, `pose`, `gripper_width`, `status`

### `stop`

Latch a stop for the rest of the run (no lease needed).

Result: `status`, `stopped`

### `heartbeat` (lease)

Renew the lease (expires 0.5 s after the last leased request); never renews targets.

Result: `status`

### `source-metadata` (lease)

Append metadata for the current lease.

| Field | Type | | Meaning |
| --- | --- | --- | --- |
| `metadata` | object | optional | JSON object, at most 8 KiB |

Result: `status`, `source`, `epoch`

### `release` (lease)

Give up the lease; the kernel holds the measured pose.

Result: `status`

### `hold` (lease)

Drop targets and cancel the operation; hold the measured pose.

Result: `status`, `physical_stopped`

### `cancel` (lease)

Same as hold.

Result: `status`, `physical_stopped`

### `preview`

Plan without moving: joint path and times, TCP path (<=160 poses), checked against limits, derivative bounds and the table plane. Valid for 30 s and only for the same owner and model.

| Field | Type | | Meaning |
| --- | --- | --- | --- |
| `kind` | str | optional | movej, movel, home or jog |
| `q` | q | optional | movej / jog goal |
| `pose` | pose | optional | movel goal (unit quaternion) |
| `speed` | float | optional | joint speed bound 0.01..0.5 rad/s (default 0.15) |
| `gripper_width` | float | optional | width held during the move, 0..gripper_max_width (default: current) |

Result: `kind`, `epoch`, `owner_present`, `model_id`, `q`, `times`, `gripper_width`, `plan_id`, `expires`, `collision_check`, `tcp`, `workspace`

### `execute` (lease)

Run a preview from where it started (within 0.03 rad, at rest); poll status.operations until it settles.

| Field | Type | | Meaning |
| --- | --- | --- | --- |
| `plan_id` | str | required | from preview |

Result: `operation_id`, `status`

### `free-drive` (lease)

Arm free to move by hand while renewed.

| Field | Type | | Meaning |
| --- | --- | --- | --- |
| `ttl_ms` | int | optional | 20..250, default 100; the kernel holds when the newest target expires |
| `source_sequence` | int | optional | optional, -1..2^63, recorded with each tick |
| `gripper_width` | float | optional | optional: hold the gripper at this width while the arm is free |

Result: `status`

### `joint` (lease)

Stream a joint target; the kernel moves toward it within its per-tick step limit.

| Field | Type | | Meaning |
| --- | --- | --- | --- |
| `ttl_ms` | int | optional | 20..250, default 100; the kernel holds when the newest target expires |
| `source_sequence` | int | optional | optional, -1..2^63, recorded with each tick |
| `q` | q | required | joint target within limits |
| `gripper_width` | float | optional | gripper target (default: current) |

Result: `status`

### `cartesian` (lease)

Stream a TCP target through the kernel IK.

| Field | Type | | Meaning |
| --- | --- | --- | --- |
| `ttl_ms` | int | optional | 20..250, default 100; the kernel holds when the newest target expires |
| `source_sequence` | int | optional | optional, -1..2^63, recorded with each tick |
| `pose` | pose | required | TCP target within 0.15 m of the current TCP and above the table plane |
| `gripper_width` | float | optional | gripper target (default: current) |

Result: `status`

### `gripper` (lease)

Hold the arm and stream a gripper target.

| Field | Type | | Meaning |
| --- | --- | --- | --- |
| `ttl_ms` | int | optional | 20..250, default 100; the kernel holds when the newest target expires |
| `source_sequence` | int | optional | optional, -1..2^63, recorded with each tick |
| `gripper_width` | float | optional | gripper target, 0..gripper_max_width |

Result: `status`

`status.owner`: `source`, `epoch`, `ingress`, `purpose`. `status.operations[]`: `operation_id`, `status`, `started`, `kind` (+ `reason`), status one of `running`, `completed`, `failed`, `cancelled`.
<!-- protocol:end -->
