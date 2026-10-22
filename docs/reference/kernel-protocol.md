# 内核命令协议（commands.sock）

所有客户端（工作台、手机、动作、标定、策略、网页控制台）经 `outputs/runtime/control/<RUN>/commands.sock` 访问内核。每个连接发送一条长度前缀 JSON 请求，返回 `{"ok": true, "result": …}` 或 `{"ok": false, "error": "<类型>: <说明>"}`。Python 客户端用 `embodied.contracts.runtime.control.ControlClient` / `ControlSession`，不需要自己拼请求。

- 单位：关节 rad，长度 m，位姿 `[x, y, z, qx, qy, qz, qw]`（`base_link` 中的 TCP `tcp_link`），时间为主机 CLOCK_MONOTONIC 纳秒。
- 租约：同一时刻只有一个拥有者；租约在最后一条带租约请求后 0.5 s 过期，`status` 不续租。任何撤销（过期、release、stop、接管）都使 `epoch` 加一，之前的 token 立即失效。
- 安全由内核统一负责：关节限位、桌面平面、单步与导数界限、0.15 m 笛卡尔领先距离、目标过期后保持，对所有来源一致。
- 金样本：`tests/golden/kernel/samples.json`（每个动作一条真实请求与结果，长列表已截短）；`./run.sh check all` 的 `sim` 组用 `scripts/check_kernel_protocol.py` 在 MuJoCo 内核上逐个动作核对，`--update` 重新生成样本。
- 观测流（`observer.sock`，RobotTickV3）为只读遥测，见 `src/embodied/control/robot_tick_v3.json` 与 `embodied.contracts.runtime.RuntimeReader`。V3 使用独立原生类型与 `/telemetry/robot_tick/v3` topic，显式 `joint_count` 为 1..15，JSON 数组只含实际轴数；不得用 V2 名称解析 V3 ABI。episode schema 2 是独立的存储与动作定义版本，保持不变。

下面的表由 `embodied.contracts.protocol` 生成（`PYTHONPATH=src python3 -m embodied.contracts.protocol`）。

<!-- protocol:start -->
Joint vectors (`q`) contain one finite number per model joint. The kernel validates their length against `describe.robot.joint_count`; poses always contain seven finite numbers.

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

Robot limits, control rate, park pose, gripper range, CAN interface, model id, workspace and motion policy; no feedback needed. motion_limits reports defaults, per-joint velocity capabilities, their source, control_hz and settling thresholds. Real capabilities come from active SDK configuration; simulation capabilities come from nominal robot configuration. These are not measured hardware ratings.

Result: `protocol`, `runtime_id`, `model_id`, `robot`, `workspace`, `motion_limits`, `actions`

### `status`

Latest measured state, owner and operations; clients treat feedback older than 150 ms as stale.

Result: `runtime_id`, `model_id`, `epoch`, `owner`, `last_revocation`, `lease_remaining_s`, `stopped`, `phase`, `q`, `dq`, `pose`, `gripper_width`, `gripper_velocity`, `gripper_effort`, `gripper_contact`, `feedback_monotonic_ns`, `operations`, `ingresses`

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

Give up the lease. After a completed operation, keep its final joint and gripper targets with zero joint velocity. Releasing an active operation cancels it and holds current feedback. A new owner, hold, cancel, stop, fault or lease expiry resets the reference to current feedback.

Result: `status`

### `hold` (lease)

Drop targets and cancel the operation; hold the measured pose.

Result: `status`, `physical_stopped`

### `cancel` (lease)

Same as hold.

Result: `status`, `physical_stopped`

### `preview`

Plan without moving: joint path and times, TCP path (<=160 poses), checked against limits, derivative bounds and the table plane. MoveJ, home and jog use the shared Ruckig trajectory; MoveL preserves its Cartesian path while scaling time to satisfy joint and optional TCP bounds. MovePath uses one blended quintic trajectory with no intermediate settling; its coefficients are hashed in trajectory. TCP limits use explicit m/s, m/s^2, m/s^3 and rad/s, rad/s^2, rad/s^3 units; limits.tcp and sampled FK peak metrics are part of plan_id. trajectory describes the evaluator; limits includes requested speed/acceleration/jerk and effective joint_velocity_rad_s; metrics reports duration, peak derivatives, limiting_factors and derivative_evaluation. These fields are included in plan_id. Acceleration and jerk are planning parameters, not hardware ratings. Valid for 30 s and only for the same owner and model.

| Field | Type | | Meaning |
| --- | --- | --- | --- |
| `kind` | str | optional | movej, movel, move_path, home or jog |
| `q` | q | optional | movej / jog goal |
| `pose` | pose | optional | movel goal (unit quaternion) |
| `waypoints` | poses | optional | move_path: 1..64 base-frame TCP targets; start is current pose |
| `blend_radius` | float | optional | move_path internal corner radius in metres; default .01 |
| `tcp_speed` | float | optional | positive TCP linear speed cap in m/s |
| `tcp_acceleration` | float | optional | positive TCP linear acceleration cap in m/s^2 |
| `tcp_jerk` | float | optional | positive TCP linear jerk cap in m/s^3 |
| `angular_speed` | float | optional | positive TCP angular speed cap in rad/s |
| `angular_acceleration` | float | optional | positive TCP angular acceleration cap in rad/s^2 |
| `angular_jerk` | float | optional | positive TCP angular jerk cap in rad/s^3 |
| `speed` | float | optional | finite nonnegative joint speed cap, rad/s (default 0.15); 0 uses backend per-joint velocity capabilities, positive values use min(speed, capability) on each joint before speed_scale |
| `speed_scale` | float | optional | finite multiplier in (0, 1] on each resolved joint velocity cap; default 1 |
| `acceleration` | float | optional | finite positive planned joint acceleration, rad/s^2 (default 2) |
| `jerk` | float | optional | finite positive planned joint jerk, rad/s^3 (default 10) |
| `gripper_width` | float | optional | width held during the move, 0..gripper_max_width (default: current) |

Result: `kind`, `epoch`, `owner_present`, `model_id`, `q`, `times`, `gripper_width`, `plan_id`, `expires`, `trajectory`, `limits`, `metrics`, `collision_check`, `tcp`, `workspace`

### `execute` (lease)

Run a preview from its start at rest (joint speed <=0.05 rad/s). Built-in trajectories require the configured settling position tolerance (default 0.0035 rad); position-only planner plugins retain the 0.03 rad start tolerance. Poll status.operations until it settles. Completion holds the final joint and gripper targets with zero joint velocity; cancellation, timeout, fault or lease expiry resets the hold reference to current feedback.

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

### `grasp` (lease)

Capability-dependent contact grasp. The arm holds while closing effort ramps. Poll operations: result reports stop_reason (contact, minimum_width, timeout), confirmed, actual_width_m, actual_effort and effort_unit. Completed means the bounded operation ended; only confirmed=true indicates contact. Contact retains closing effort through subsequent arm moves; gripper position/cancel/release clears it. ARX motor_Nm is SDK current-estimated motor torque, not calibrated fingertip force. Detection requires a 0.3 s cosine effort ramp, at least 0.5 mm closing travel, width above minimum_width + 1 mm, absolute measured velocity <=2 mm/s and absolute measured effort >=20% of requested effort throughout contact_time. Feedback must be finite, fresh (<100 ms) and progress without gaps >100 ms. Simulation additionally requires actual contacts on both jaws against external bodies.

| Field | Type | | Meaning |
| --- | --- | --- | --- |
| `effort` | float | required | positive closing effort in describe.robot.gripper.effort_unit |
| `minimum_width` | float | optional | minimum closing width in metres; default 0 |
| `timeout` | float | optional | bounded operation duration in seconds, <=60; default 5 |
| `contact_time` | float | optional | sustained contact evidence duration in seconds; default .2 |

Result: `operation_id`, `status`

### `simulation-snapshot`

Optional simulator capability, advertised in describe.actions. Copy physics state on the worker without acquiring control. The backend owns this snapshot format. The MuJoCo adapter includes complete integration state and robot/named-body observations (metres, XYZW). Does not capture a running motion or Python task. No rendering or disk access occurs in the control loop.

Result: `schema`, `schema_version`, `backend`, `engine_version`, `model_id`, `configuration_id`, `state_spec`, `state`, `observation`

### `simulation-restore` (lease)

Simulator only. Refuse while an operation or stream is active; restore physics and enter hold. Advance the ownership epoch; the caller must use the returned epoch for later commands (ControlSession does this automatically). Invalidates previews and creates an episode boundary; finish the current recording first. Does not resume an old motion.

| Field | Type | | Meaning |
| --- | --- | --- | --- |
| `snapshot` | object | required | snapshot from the same model, configuration and engine version |

Result: `status`, `epoch`

### `simulation-reset` (lease)

Same boundary and lease semantics as simulation-restore; restore the simulator initial state.

Result: `status`, `epoch`

`status.owner`: `source`, `epoch`, `ingress`, `purpose`. `status.operations[]`: `operation_id`, `status`, `started`, `kind` (+ `reason`), status one of `running`, `completed`, `failed`, `cancelled`.
<!-- protocol:end -->
