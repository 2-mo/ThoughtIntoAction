"""Run a remote π0.5 policy against the dual-arm arx kernel.

Observe the kernel state (14 numbers) and three cameras at 30 Hz, ask the
policy server for a 50-step chunk, and stream it back as kernel joint commands
(12 absolute joint targets + one gripper width per arm).

RTC mode runs inference asynchronously: when `reinfer_remaining` actions are
left in the queue, the remaining actions are sent as `prev_chunk_left_over`
and the new chunk is requested while the robot keeps executing. When the new
chunk arrives, its first `blend` actions are blended toward the old queue so
the switch is smooth; ticks spent waiting for the chunk are counted as dry.

See docs/guides/policy-remote.md for the wire contract and server commands.
"""
from __future__ import annotations

import argparse
import json
import signal
import threading
import time
from collections import deque
from pathlib import Path

import numpy as np

FPS = 30
# Actions arrive at 30 Hz; the kernel is fed an interpolated point at this rate
# so the joint targets never step and the arm runs smoothly (RPC-bound in practice).
SEND_HZ = 250
CHUNK_HORIZON = 50
# The server guides the chunk's steps [0, execution_horizon) toward the prefix
# and samples freely after it. The client discards the first `delay` steps at
# handoff, so the guided part must cover them plus a short continuation; with a
# horizon that stops before the handoff point, the arm jumps to a fresh (and,
# being stochastic, unrelated) sample on every replan.
GUIDANCE_TAIL = 10
MAX_CONSECUTIVE_REJECTS = 45  # 1.5 s of refused targets means the policy is asking for something impossible
CAMERAS = ("wrist_rgb", "wrist_second_rgb", "exterior_rgb")  # right wrist, left wrist, fixed base


def clip_actions(actions, joint_min, joint_max, gripper_max):
    """Keep every target inside the kernel's own limits before sending it.

    Clipping happens in float64: a float32 0.082 rounds to 0.082000003, which
    the kernel's strict `<= max` check would reject.
    """
    clipped = np.array(actions, dtype=np.float64, copy=True)
    clipped[:, :12] = np.clip(clipped[:, :12], joint_min, joint_max)
    clipped[:, 12:] = np.clip(clipped[:, 12:], 0.0, float(gripper_max))
    return clipped


def build_state(status):
    """14 numbers: left 6 joints, right 6 joints, gripper_left, gripper_right."""
    q = np.asarray(status["q"], dtype=np.float32)
    if q.shape != (12,):
        raise ValueError(f"远程双臂策略需要 12 关节内核，当前报告 {q.shape[0]} 关节")
    widths = status.get("gripper_widths")
    if not widths or len(widths) != 2:
        width = float(status.get("gripper_width") or 0.0)
        widths = [width, width]
    return np.concatenate([q, np.asarray(widths, dtype=np.float32)])


def build_observation(session, cameras, prompt, *, rtc=None):
    """One observation: kernel state plus one frame per configured camera."""
    status = session.status()
    state = build_state(status)
    observation = {"state": state, "prompt": prompt}
    for camera_id, camera in cameras.items():
        frame = camera.read()
        # Cameras deliver BGR; the dataset export flips to RGB before training,
        # so the policy was trained on RGB and must be fed the same order.
        image = np.asarray(frame["image"], dtype=np.uint8)[..., ::-1].copy()
        observation[camera_id] = image
        observation[f"{camera_id}_valid"] = np.bool_(True)
    if rtc:
        observation.update(rtc)
    return observation, status


def blend_into(new_actions, old_actions, blend):
    """Linearly pull the head of a new chunk toward the actions still queued."""
    if blend <= 0 or old_actions is None or len(old_actions) == 0:
        return new_actions
    count = min(blend, len(old_actions), len(new_actions))
    blended = np.array(new_actions, dtype=np.float32, copy=True)
    for index in range(count):
        weight = 1.0 - index / max(blend, 1)
        blended[index] = (1.0 - weight) * blended[index] + weight * old_actions[index]
    return blended


class ChunkQueue:
    """Actions the executor drains, plus the pending chunk from the worker."""

    def __init__(self):
        self.lock = threading.Lock()
        self.actions = deque()
        self.pending = None
        self.requested = False
        self.request_tick = None

    def remaining(self):
        with self.lock:
            return len(self.actions)

    def snapshot(self):
        with self.lock:
            return np.asarray(self.actions, dtype=np.float32) if self.actions else None

    def take(self):
        with self.lock:
            return self.actions.popleft() if self.actions else None

    def deliver(self, actions, tick):
        with self.lock:
            self.pending = (actions, tick)
            self.requested = False

    def adopt(self, tick, blend):
        """Take the pending chunk at the step the arm has actually reached.

        Its first actions describe the moment the observation was taken, which
        is `tick - request_tick` ticks in the past; playing them now would walk
        the arm back along its own recent motion.
        """
        with self.lock:
            if self.pending is None:
                return None
            actions, request_tick = self.pending
            self.pending = None
            delay = max(0, (tick - request_tick) if request_tick is not None else 0)
            skipped = 0
            if delay and len(actions) > delay + 5:
                actions, skipped = actions[delay:], delay
            old = list(self.actions)
            actions = blend_into(actions, np.asarray(old, dtype=np.float32) if old else None, blend)
            self.actions = deque(actions)
            return {"request_tick": request_tick, "arrival_tick": tick, "skipped": skipped,
                    "blended": bool(old) and blend > 0}

    def request(self, tick):
        with self.lock:
            if self.requested or self.pending is not None:
                return None
            self.requested = True
            self.request_tick = tick
            return np.asarray(self.actions, dtype=np.float32) if self.actions else None


def inference_worker(client, session, cameras, prompt, queue, stop, *, guidance, prefix_schedule,
                     fps, on_error, joint_min, joint_max, gripper_max):
    while not stop.is_set():
        if not queue.lock.acquire(timeout=0.05):
            continue
        try:
            waiting = queue.requested and queue.pending is None
            leftover = np.asarray(queue.actions, dtype=np.float32) if waiting and queue.actions else None
            request_tick = queue.request_tick
        finally:
            queue.lock.release()
        if not waiting:
            stop.wait(0.005)
            continue
        try:
            rtc = None
            if leftover is not None and len(leftover):
                delay = max(0, round((client.last_latency_s or 0.0) * fps))
                horizon = min(CHUNK_HORIZON, len(leftover), delay + GUIDANCE_TAIL)
                rtc = {
                    "prev_chunk_left_over": leftover,
                    "inference_delay": np.int32(delay),
                    "execution_horizon": np.int32(max(1, horizon)),
                    "rtc_guidance_weight": np.float32(guidance),
                    "rtc_prefix_schedule": prefix_schedule,
                }
            observation, _status = build_observation(session, cameras, prompt, rtc=rtc)
            reply = client.infer(observation)
            # Same limit clipping as the blocking path: an unclipped float32
            # gripper width is enough for the kernel to reject the whole chunk.
            actions = clip_actions(reply["actions"], joint_min, joint_max, gripper_max)
            queue.deliver(actions.astype(np.float32), request_tick)
        except Exception as exc:
            on_error(f"{type(exc).__name__}: {exc}")
            return


def run(session, client, cameras, output, prompt, *, seconds, fps=FPS, chunk_steps=CHUNK_HORIZON,
        rtc_enabled=False, reinfer_remaining=30, blend=16, guidance=10.0, prefix_schedule="linear",
        dry_run=False):
    """Execute a chunk schedule; with RTC the next chunk is inferred while the queue drains."""
    description = session.describe()
    robot = description["robot"]
    joint_min = np.asarray(robot["joint_min"], dtype=np.float32)
    joint_max = np.asarray(robot["joint_max"], dtype=np.float32)
    gripper_max = float(robot["gripper_max_width"])
    queue = ChunkQueue()
    stop = threading.Event()
    errors = []
    worker = None
    summary = {"status": "running", "prompt": prompt, "fps": fps, "rtc": rtc_enabled,
               "chunk_steps": chunk_steps, "reinfer_remaining": reinfer_remaining, "blend": blend,
               "dry_run": dry_run, "chunks": 0, "commands": 0, "dry_ticks": 0,
               "kernel_runtime_id": session.identity["runtime_id"], "model_metadata": client.metadata}
    started = time.monotonic()
    deadline = started + seconds
    consecutive = 0  # refused commands in a row; reset by any accepted one

    def tick_now():
        return int((time.monotonic() - started) * fps)

    if rtc_enabled:
        # The worker only infers; the executor decides whether to send. A dry run
        # still needs it, otherwise the RTC loop would request and never receive.
        worker = threading.Thread(target=inference_worker,
                                  args=(client, session, cameras, prompt, queue, stop),
                                  kwargs={"guidance": guidance, "prefix_schedule": prefix_schedule,
                                          "fps": fps, "on_error": errors.append,
                                          "joint_min": joint_min, "joint_max": joint_max,
                                          "gripper_max": gripper_max},
                                  daemon=True)
        worker.start()

    # The arm is fed a point interpolated between the last sent target and the
    # action being executed, so a 30 Hz chunk becomes a continuous path instead
    # of 33 ms steps. The step clock is wall time, never accumulated.
    send_period = 1.0 / SEND_HZ
    step_period = 1.0 / fps
    initial = session.status()
    widths = initial.get("gripper_widths") or [initial.get("gripper_width") or 0.0] * 2
    last_point = np.asarray([*initial["q"], *widths], dtype=np.float64)
    current = None            # action the interpolation is heading for
    step_started = time.monotonic()
    logged_index = None
    rejected_action = False   # this action was refused; skip its remaining sub-steps
    next_send = time.monotonic()

    with (output / "commands.jsonl").open("x") as log:
        try:
            while time.monotonic() < deadline:
                now = time.monotonic()
                if now < next_send:
                    time.sleep(next_send - now)
                    now = time.monotonic()
                # A slow send restarts the pace; never repay the debt in a burst.
                next_send = now + send_period
                tick = tick_now()
                if rtc_enabled:
                    # Hand off as soon as the new chunk arrives; the remaining old
                    # actions are blended into its head instead of being drained.
                    adopted = queue.adopt(tick, blend)
                    if adopted is not None:
                        summary["chunks"] += 1
                        summary["skipped_steps"] = summary.get("skipped_steps", 0) + adopted["skipped"]
                        print(json.dumps({"event": "chunk", "index": summary["chunks"],
                                          "arrival_delay_ticks": tick - adopted["request_tick"],
                                          "skipped_steps": adopted["skipped"],
                                          "blended": adopted["blended"]}, ensure_ascii=False), flush=True)
                if current is None or now - step_started >= step_period:
                    action = queue.take()
                    if action is None:
                        if not rtc_enabled:
                            # Simple mode: block for one chunk when the queue runs dry.
                            observation, _status = build_observation(session, cameras, prompt)
                            reply = client.infer(observation)
                            chunk = clip_actions(reply["actions"], joint_min, joint_max, gripper_max)[:chunk_steps]
                            queue.lock.acquire()
                            queue.actions = deque(chunk)
                            queue.lock.release()
                            summary["chunks"] += 1
                            print(json.dumps({"event": "chunk", "index": summary["chunks"],
                                              "steps": len(chunk),
                                              "latency_ms": None if client.last_latency_s is None
                                              else round(client.last_latency_s * 1000, 1)},
                                             ensure_ascii=False), flush=True)
                            current = None
                            step_started = time.monotonic()
                            continue
                        # Async with nothing queued: one dry tick per action step.
                        if current is not None or now - step_started >= step_period:
                            summary["dry_ticks"] += 1
                            step_started = now
                        current = None
                        queue.request(tick)
                        continue
                    current = np.asarray(action, dtype=np.float64)
                    step_started = now
                    rejected_action = False
                    if not dry_run and logged_index != tick:
                        logged_index = tick
                        log.write(json.dumps({"host_monotonic_ns": time.monotonic_ns(), "tick": tick,
                                              "action_index": tick,
                                              "action": [round(float(value), 5) for value in current]}) + "\n")
                        log.flush()
                alpha = min(max((now - step_started) / step_period, 0.0), 1.0)
                point = last_point + (current - last_point) * alpha
                if dry_run:
                    last_point = point
                elif not rejected_action:
                    stamp = time.monotonic_ns()
                    try:
                        session.command("joint", q=[float(value) for value in point[:12]],
                                        gripper_targets=[float(point[12]), float(point[13])],
                                        ttl_ms=150, sent_monotonic_ns=stamp)
                    except RuntimeError as exc:
                        # The kernel refuses a target past the table plane or a
                        # step limit. Count the action once, skip its remaining
                        # sub-steps, and stop if the policy keeps asking for the same.
                        summary["rejected"] = summary.get("rejected", 0) + 1
                        rejected_action = True
                        consecutive += 1
                        print(json.dumps({"event": "rejected", "tick": tick,
                                          "action_index": tick, "reason": str(exc)},
                                         ensure_ascii=False), flush=True)
                        if consecutive >= MAX_CONSECUTIVE_REJECTS:
                            raise
                    else:
                        consecutive = 0
                        summary["commands"] += 1
                        last_point = point
                if rtc_enabled and queue.remaining() <= reinfer_remaining:
                    queue.request(tick)
            summary["status"] = "time_limit"
        except KeyboardInterrupt:
            summary["status"] = "interrupted"
        except Exception as exc:
            summary.update(status="failed", reason=f"{type(exc).__name__}: {exc}")
            raise
        finally:
            stop.set()
            if worker is not None:
                worker.join(timeout=2)
            if errors:
                summary.setdefault("errors", []).extend(errors[:3])
                if summary["status"] == "time_limit":
                    summary["status"] = "failed"
                    summary["reason"] = errors[0]
            if not dry_run:
                try:
                    session.command("hold")
                except Exception as exc:
                    summary["hold_error"] = str(exc)
            (output / "summary.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n")
    return summary


def interrupt(signum, frame):
    raise KeyboardInterrupt


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--run", type=Path, required=True, help="运行中的内核目录 outputs/runtime/control/<RUN>")
    parser.add_argument("--server", required=True, help="策略服务地址，例如 ws://127.0.0.1:8000")
    parser.add_argument("--prompt", required=True)
    parser.add_argument("--seconds", type=float, default=60)
    parser.add_argument("--fps", type=int, default=FPS)
    parser.add_argument("--cameras", default=",".join(CAMERAS), help="逗号分隔的相机 ID，顺序固定")
    parser.add_argument("--chunk-steps", type=int, default=CHUNK_HORIZON)
    parser.add_argument("--rtc", action="store_true", help="启用异步 RTC 引导采样")
    parser.add_argument("--reinfer-remaining", type=int, default=30, help="剩余多少步时请求新块")
    parser.add_argument("--blend", type=int, default=16, help="新块开头多少步与旧块混合")
    parser.add_argument("--guidance", type=float, default=10.0)
    parser.add_argument("--prefix-schedule", default="linear", choices=("linear", "exp", "zeros", "ones"))
    parser.add_argument("--dry-run", action="store_true", help="只推理不下发，用于联通性检查")
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    if not 0 < args.seconds <= 300:
        parser.error("seconds 必须在 0..300")
    if not 1 <= args.fps <= 60:
        parser.error("fps 必须在 1..60")
    if not 1 <= args.chunk_steps <= CHUNK_HORIZON:
        parser.error(f"chunk-steps 必须在 1..{CHUNK_HORIZON}")
    if not 0 <= args.reinfer_remaining < args.chunk_steps:
        parser.error("reinfer-remaining 必须在 0..chunk-steps-1")
    if not 0 <= args.blend <= args.chunk_steps:
        parser.error("blend 必须在 0..chunk-steps")

    from embodied.contracts.runtime.control import ControlSession
    from embodied.core.project import PROJECT_ROOT, load_config
    from embodied.devices.sensors.client import SharedCamera
    from embodied.policy.remote.client import RemotePolicy

    output = args.output or PROJECT_ROOT / "outputs/policy_remote" / time.strftime("%Y%m%d-%H%M%S")
    output.mkdir(parents=True, exist_ok=False)
    camera_ids = [value.strip() for value in args.cameras.split(",") if value.strip()]
    config = load_config().get("cameras") or {}
    if any(camera_id not in config for camera_id in camera_ids):
        parser.error(f"未配置的相机：{[c for c in camera_ids if c not in config]}")
    signal.signal(signal.SIGTERM, interrupt)
    stack = []
    summary = {}
    try:
        cameras = {}
        for camera_id in camera_ids:
            options = config[camera_id]
            camera = SharedCamera(options["serial"], options.get("width") or 640,
                                  options.get("height") or 480, options.get("fps") or 30,
                                  camera_id=camera_id)
            camera.__enter__()
            stack.append(camera)
            cameras[camera_id] = camera
        policy = RemotePolicy(args.server).__enter__()
        stack.append(policy)
        print(json.dumps({"event": "connected", "server": args.server, "metadata": policy.metadata},
                         ensure_ascii=False), flush=True)
        session = ControlSession(args.run, source="policy", purpose="operation",
                                 metadata={"policy": "remote", "server": args.server})
        session.__enter__()
        stack.append(session)
        summary = run(session, policy, cameras, output, args.prompt, seconds=args.seconds, fps=args.fps,
                      chunk_steps=args.chunk_steps, rtc_enabled=args.rtc,
                      reinfer_remaining=args.reinfer_remaining, blend=args.blend,
                      guidance=args.guidance, prefix_schedule=args.prefix_schedule,
                      dry_run=args.dry_run)
    finally:
        for item in reversed(stack):
            try:
                item.__exit__(None, None, None)
            except Exception:
                pass
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 0 if summary.get("status") in ("time_limit", "interrupted") else 1


if __name__ == "__main__":
    raise SystemExit(main())
