"""RM65 right-arm observations for the remote cube policy."""
from contextlib import ExitStack
import json
from pathlib import Path

import numpy as np

from embodied.contracts.runtime.control import ControlClient, ControlSession
from embodied.core.project import PROJECT_ROOT, load_config
from embodied.devices.sensors.client import SharedCamera
from embodied.policy.kernel_runner import POLICY_FPS, run, sample_pair, validate_policy_robot
from embodied.policy.remote.client import PolicyError, RemotePolicy

HORIZON = 15
DEPLOYMENT = PROJECT_ROOT / "configs/policies/rm65_cube_pi05.json"


def observe(session, cameras, cycle, robot):
    state, frames, _, stamp = sample_pair(session, cameras)
    vector = np.asarray([*state["q"], state["gripper_width"]], dtype=np.float32)
    low, high = robot["data_contract"]["gripper"]["range"]
    if vector.shape != (7,) or not np.isfinite(vector).all() or not low - 1e-6 <= vector[-1] <= high + 1e-6:
        raise PolicyError("RM65 关节或夹爪反馈无效")
    (cycle / "observation.json").write_text(json.dumps({
        "state": vector.tolist(), "rgb_monotonic_ns": stamp,
        "feedback_monotonic_ns": state["feedback_monotonic_ns"],
        "camera_timestamps_ns": {role: frame["host_received_monotonic_ns"] for role, frame in frames.items()},
    }) + "\n")
    return {"rgb_monotonic_ns": stamp, "state": vector, "frames": frames}


def predict(policy, observation, cycle, prompt, gripper_range):
    request = {"state": observation["state"], "prompt": prompt}
    for role, frame in observation["frames"].items():
        request[role] = np.asarray(frame["image"], dtype=np.uint8)[..., ::-1].copy()
        request[f"{role}_valid"] = np.bool_(True)
    actions = policy.infer(request)["actions"]
    if actions.shape != (HORIZON, 7):
        raise PolicyError(f"RM65 动作形状应为 ({HORIZON}, 7)，实际为 {actions.shape}")
    np.save(cycle / "actions.npy", actions, allow_pickle=False)
    print(json.dumps({"event": "chunk", "steps": len(actions),
                      "latency_ms": round(policy.last_latency_s * 1000, 1)}), flush=True)
    actions = actions.astype(np.float64)
    actions[:, 6] = np.clip(actions[:, 6], *gripper_range)
    return actions


def execute(args, output):
    deployment = json.loads(DEPLOYMENT.read_text())
    prompt = args.prompt or deployment["prompt"]
    control = ControlClient(args.run)
    parameters = control.parameters
    hardware = parameters["effective_robot_config"]
    if control.identity["backend"] != "real" or hardware.get("hardware_plugin") != "realman.real":
        raise PolicyError("请选择右侧 RM65 真机的单臂 RUN")
    connection = json.loads(Path(hardware["connection_path"]).read_text())
    if connection.get("host") != deployment["robot_host"]:
        raise PolicyError(f"请选择控制器 {deployment['robot_host']} 对应的右臂 RUN")
    camera_map = deployment["camera_map"]
    if args.cameras:
        right_wrist, exterior = args.cameras.split(",")
        camera_map = {"right_wrist_rgb": right_wrist.strip(), "exterior_rgb": exterior.strip()}
    config = load_config()["cameras"]
    if any(role not in config for role in camera_map.values()) or len({config[role]["serial"] for role in camera_map.values()}) != 2:
        raise PolicyError("RM65 需要配置两台独立相机，顺序为右腕、外部")
    robot = validate_policy_robot(parameters, deployment, camera_map)
    with ExitStack() as stack:
        policy = stack.enter_context(RemotePolicy(args.server, timeout_s=1.5))
        expected = {"robot_type": robot["data_contract"]["robot_type"],
                    "action_dim": 7, "action_horizon": HORIZON, "fps": POLICY_FPS}
        expected["asset_id"] = args.asset_id or deployment["asset_id"]
        if any(policy.metadata.get(key) != value for key, value in expected.items()):
            raise PolicyError(f"RM65 模型元数据不匹配，要求 {expected}，收到 {policy.metadata}")
        print(json.dumps({"event": "connected", "server": args.server, "metadata": policy.metadata,
                          "camera_map": camera_map}, ensure_ascii=False), flush=True)
        cameras = {}
        for role, camera_id in camera_map.items():
            settings = config[camera_id]
            cameras[role] = stack.enter_context(SharedCamera(
                settings["serial"], settings["width"], settings["height"], settings["fps"], camera_id=camera_id))
        session = control if args.dry_run else stack.enter_context(
            ControlSession(args.run, source="rm65-cube-policy", purpose="operation",
                           metadata={"policy": "remote", "server": args.server}))
        return run(session, None, cameras, output, prompt, seconds=args.seconds,
                   chunk_steps=args.chunk_steps, robot=robot, dry_run=args.dry_run, observer=observe,
                   predict=lambda obs, cycle, prompt: predict(
                       policy, obs, cycle, prompt, robot["data_contract"]["gripper"]["range"]),
                   model_metadata=policy.metadata)
