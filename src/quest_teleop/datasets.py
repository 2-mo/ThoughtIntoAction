"""Inspect, validate and replay raw teleop episodes without opening robot hardware."""

import argparse
import json
from pathlib import Path
import time

import numpy as np

from robot_core.model import RobotModel
from robot_core.storage import save_json
from .video import media_path, read_frame


def load_episode(path):
    root = Path(path).expanduser().resolve()
    manifest = json.loads((root / "episode.json").read_text())
    if not isinstance(manifest, dict) or manifest.get("schema_version") != 1 or manifest.get("kind") != "quest_teleop_episode":
        raise ValueError("Unknown episode schema")
    if manifest.get("record_storage", {}).get("format") != "mcap":
        raise ValueError("Only MCAP episodes are supported")
    from .mcap_io import read_records
    states, frames = read_records(root / "episode.mcap")
    return root, manifest, states, frames


def image_path(root, relative):
    target = (root / relative).resolve()
    if not target.is_relative_to(root):
        raise ValueError("Image path leaves the episode directory")
    return target


def nearest_indices(reference, query):
    reference, query = np.asarray(reference, dtype=np.int64), np.asarray(query, dtype=np.int64)
    if len(reference) == 0 or np.any(np.diff(reference) <= 0):
        raise ValueError("Reference timestamps must be nonempty and increasing")
    right = np.clip(np.searchsorted(reference, query), 0, len(reference) - 1)
    left = np.clip(right - 1, 0, len(reference) - 1)
    return np.where(abs(query - reference[left]) <= abs(query - reference[right]), left, right)


def validate_episode(path, *, settings=None, robot=None, check_images=True):
    from robot_core.project import load_config, resolve_path
    config = load_config()
    settings = settings or config["recording"]
    robot = robot or RobotModel(resolve_path(config["robot"]["urdf"]))
    errors, warnings = [], []
    report = {"valid": False, "training_eligible": False, "errors": errors, "warnings": warnings}
    try:
        root, meta, states, frames = load_episode(path)
        report.update({"episode": meta["id"], "task": meta["task"], "lifecycle": meta["lifecycle"],
                       "outcome": meta["outcome"], "state_count": len(states), "frame_count": len(frames)})
        if meta["lifecycle"] != "complete":
            errors.append("Episode incomplete/interrupted/failed")
        if meta["robot"] != robot.metadata():
            errors.append("Robot model hash, frame, units or joint order mismatch")
        if not isinstance(meta["task"], str) or not meta["task"].strip():
            errors.append("Empty task description")
        if meta["outcome"] not in {"success", "failure", "unknown", "discarded"}:
            errors.append("Invalid outcome")
        if meta.get("error") or any(meta.get("dropped", {}).values()):
            errors.append("Writer reported a failure or dropped records")
        if any(meta.get("counts", {}).get(k) != n for k, n in (("state", len(states)), ("frame", len(frames)))):
            errors.append("Manifest counts differ from stored rows")
        if len(states) < 2 or len(frames) < 2:
            raise ValueError("Need at least two robot samples and two RGB frames")
        state_time = np.array([s["host_monotonic_ns"] for s in states], dtype=np.int64)
        frame_time = np.array([s["host_received_monotonic_ns"] for s in frames], dtype=np.int64)
        started, stopped = meta["started_monotonic_ns"], meta.get("stopped_monotonic_ns", 0)
        for name, timestamps, max_gap in (
            ("state", state_time, settings["max_state_gap_s"]),
            ("frame", frame_time, settings["max_frame_gap_s"])):
            gaps = np.diff(timestamps) / 1e9
            if np.any(gaps <= 0):
                errors.append(f"Non-increasing {name} timestamps")
            if np.any(gaps > max_gap):
                errors.append(f"{name} stream has gaps exceeding {max_gap}s")
            if timestamps[0] < started or timestamps[-1] > stopped:
                errors.append(f"{name} timestamps outside episode interval")
            if timestamps[0] - started > max_gap * 1e9 or stopped - timestamps[-1] > max_gap * 1e9:
                errors.append(f"{name} stream does not cover episode boundaries")
            report[f"{name}_max_gap_s"] = float(max(gaps))
            report[f"{name}_hz"] = float((len(timestamps) - 1) * 1e9 / max(1, timestamps[-1] - timestamps[0]))
            nominal = meta["control"].get("recording_rate_hz", meta["control"]["control_rate_hz"]) if name == "state" else meta["camera"]["fps"]
            if report[f"{name}_hz"] < 0.8 * nominal:
                errors.append(f"{name} average rate is below 80% of configured rate")
        duration = (min(state_time[-1], frame_time[-1]) - max(state_time[0], frame_time[0])) / 1e9
        report["overlap_duration_s"] = duration
        if duration < settings["min_duration_s"]:
            errors.append("Insufficient overlapping recording duration")
        width = meta["control"]["gripper_width_m"]
        submitted_count = 0
        previous_submission = None
        for row in states:
            for key in ("actual_pos", "actual_vel", "actual_torque", "sdk_cmd_pos", "sdk_cmd_vel", "sdk_cmd_torque", "kp", "kd"):
                vector = np.asarray(row[key], dtype=float)
                if vector.shape != (6,) or not np.isfinite(vector).all():
                    raise ValueError(f"Invalid six-joint vector: {key}")
            robot.joints(row["actual_pos"])
            for key in ("gripper_actual_pos", "gripper_actual_vel", "gripper_actual_torque",
                        "feedback_timestamp", "command_timestamp", "controller_timestamp"):
                if not np.isfinite(row[key]):
                    raise ValueError(f"Non-finite {key}")
            if not -.005 <= row["gripper_actual_pos"] <= width + .005:
                errors.append("Gripper feedback outside calibrated width range")
            if row["source"] not in {meta["control"].get("input_plugin", "quest"), "preset", "hold"}:
                raise ValueError("Unknown command source")
            if row["requested_pos"] is not None:
                values = np.asarray(row["requested_pos"])
                if values.shape != (6,) or not np.isfinite(values).all():
                    raise ValueError("Invalid requested joint target")
            submission = row["submission"]
            if submission is not None:
                submitted_count += 1
                for key in ("pos", "vel", "torque"):
                    values = np.asarray(submission[key])
                    if values.shape != (6,) or not np.isfinite(values).all():
                        raise ValueError(f"Invalid submitted {key}")
                robot.joints(submission["pos"])
                if (not np.isfinite(submission["gripper_pos"])
                        or not 0 <= submission["gripper_pos"] <= width
                        or not np.isfinite(submission["gripper_torque"])):
                    raise ValueError("Invalid gripper submission")
                if submission["host_monotonic_ns"] > row["host_monotonic_ns"]:
                    raise ValueError("Submission timestamp is later than its snapshot")
                if previous_submission is not None:
                    if submission["sequence"] < previous_submission["sequence"]:
                        raise ValueError("Submission sequence regressed")
                    if submission["sequence"] == previous_submission["sequence"] and submission != previous_submission:
                        raise ValueError("One submission sequence has inconsistent content")
                previous_submission = submission
        if np.any(np.diff([s["sequence"] for s in states]) <= 0):
            errors.append("Robot snapshot sequence repeated/regressed")
        sdk_time = np.asarray([s["feedback_timestamp"] for s in states])
        if np.any(np.diff(sdk_time) < 0) or sdk_time[-1] <= sdk_time[0]:
            errors.append("SDK feedback clock stalled/regressed")
        progress_host = state_time[0]
        for index in range(1, len(states)):
            if sdk_time[index] > sdk_time[index - 1]:
                progress_host = state_time[index]
            elif (state_time[index] - progress_host) / 1e9 > settings["max_state_gap_s"]:
                errors.append("SDK feedback clock stopped advancing within the episode")
                break
        numbers = np.array([f["frame_number"] for f in frames], dtype=np.int64)
        if np.any(np.diff(numbers) <= 0):
            errors.append("Repeated/regressed source camera frames")
        if np.any(np.diff(numbers) > 1):
            errors.append("Missing source camera frames")
        device_time = np.array([f["device_timestamp_ms"] for f in frames], dtype=float)
        if not np.isfinite(device_time).all() or np.any(np.diff(device_time) <= 0):
            errors.append("Invalid camera device timestamps")
        if len({f["device_timestamp_domain"] for f in frames}) != 1:
            errors.append("Camera timestamp domain changed inside episode")
        selected = nearest_indices(state_time, frame_time)
        skew = abs(state_time[selected] - frame_time) / 1e9
        report["receive_time_alignment_p99_s"] = float(np.quantile(skew, .99))
        report["receive_time_alignment_max_s"] = float(max(skew))
        if max(skew) > settings["max_sync_skew_s"]:
            errors.append("RGB receipt/robot sampling alignment exceeds limit")
        report["alignment_clock"] = "host monotonic receipt/sample times, not synchronized exposure"
        seen_images = set()
        for row in frames:
            image = media_path(root, row)
            frame_key = (image, row.get("video_frame_index"))
            if frame_key in seen_images:
                errors.append("Multiple frames reference one image")
            seen_images.add(frame_key)
            if not image.is_file():
                raise ValueError(f"Missing image: {image}")
            if check_images:
                import cv2
                decoded = read_frame(root, row)
                if decoded is None or decoded.shape != (meta["camera"]["height"], meta["camera"]["width"], 3):
                    raise ValueError(f"Corrupt image or incorrect resolution: {image}")
        if not meta["hand_eye_calibrated"]:
            warnings.append("Hand-eye uncalibrated; RGB/joint records remain usable")
        has_actions = meta["control"]["actuate"] and all(s["actuate"] for s in states) and submitted_count == len(states)
        if not has_actions:
            warnings.append("No complete actuated command stream; unsuitable for commanded-action training")
        report["valid"] = not errors
        report["training_eligible"] = not errors and has_actions and meta["outcome"] == "success"
    except (OSError, ValueError, KeyError, TypeError, OverflowError) as exc:
        errors.append(str(exc))
    return report


def replay(path, *, speed=1., headless=False, telemetry="local"):
    if not np.isfinite(speed) or speed <= 0:
        raise ValueError("Replay speed must be positive")
    root, meta, states, frames = load_episode(path)
    if not states or not frames:
        raise ValueError("No robot/RGB samples to replay")
    from robot_core.project import load_config, resolve_path
    robot = RobotModel(resolve_path(load_config()["robot"]["urdf"]))
    if meta["robot"] != robot.metadata():
        raise ValueError("Replay model mismatch")
    st = np.array([s["host_monotonic_ns"] for s in states], dtype=np.int64)
    ft = np.array([f["host_received_monotonic_ns"] for f in frames], dtype=np.int64)
    selected = nearest_indices(st, ft)
    for row in states:
        robot.joints(row["actual_pos"])
    if headless:
        print(json.dumps({"frames": len(frames), "states": len(states),
                          "duration_s": float((ft[-1] - ft[0]) / 1e9),
                          "robot": meta["robot"], "hardware_opened": False}, indent=2))
        return
    import cv2
    from robot_core.plugins import Registry
    actual = [robot.fk(s["actual_pos"])[:3, 3] for s in states]
    commands = [robot.fk(s["submission"]["pos"])[:3, 3] for s in states if s["submission"] is not None]
    sink = Registry().create("telemetry", telemetry, robot=robot, source={
        "kind": "replay", "input": "dataset", "episode": root.name})
    try:
        sink.path("recorded_feedback_path", actual, 0x33CC55)
        if commands:
            sink.path("recorded_command_path", commands, 0x2299FF)
        began = time.monotonic()
        for index, frame in enumerate(frames):
            due = began + (ft[index] - ft[0]) / 1e9 / speed
            time.sleep(max(0., due - time.monotonic()))
            row = states[selected[index]]
            target = None if row["submission"] is None else np.asarray(row["submission"]["pos"])
            sink.update(np.asarray(row["actual_pos"]), target)
            decoded = read_frame(root, frame)
            if decoded is None:
                raise ValueError("Missing/corrupt replay image")
            cv2.imshow("Recorded wrist RGB (q/Esc to exit)", decoded)
            if cv2.waitKey(1) & 0xff in (ord("q"), 27):
                break
    finally:
        try:
            sink.close()
        finally:
            cv2.destroyAllWindows()


def label_episode(path, outcome, note=""):
    root, meta, _, _ = load_episode(path)
    if meta["lifecycle"] == "recording":
        raise ValueError("Cannot label a recording/unfinalized episode")
    if outcome not in {"success", "failure", "unknown", "discarded"}:
        raise ValueError("Invalid outcome")
    if outcome == "success":
        report = validate_episode(path)
        if not report["valid"]:
            raise ValueError(f"Episode failed validation: {report['errors']}")
    meta.setdefault("outcome_history", []).append({"previous": meta["outcome"], "outcome": outcome,
                                                   "note": note, "unix_ns": time.time_ns()})
    meta["outcome"] = outcome
    save_json(root / "episode.json", meta)
    return {"episode": meta["id"], "outcome": outcome}


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    commands = parser.add_subparsers(dest="command", required=True)
    for command in ("inspect", "validate", "replay"):
        sub = commands.add_parser(command)
        sub.add_argument("episode")
        if command == "validate":
            sub.add_argument("--output", help="Optionally save a validation report")
        if command == "replay":
            sub.add_argument("--speed", type=float, default=1.)
            sub.add_argument("--headless", action="store_true")
            sub.add_argument("--telemetry", default="local")
    label = commands.add_parser("label", help="Set reviewed outcome; retain all raw files")
    label.add_argument("episode")
    label.add_argument("--outcome", choices=["success", "failure", "unknown", "discarded"], required=True)
    label.add_argument("--note", default="")
    args = parser.parse_args(argv)
    if args.command == "label":
        print(json.dumps(label_episode(args.episode, args.outcome, args.note), ensure_ascii=False, indent=2))
        return 0
    if args.command == "validate":
        result = validate_episode(args.episode)
        if args.output:
            save_json(args.output, result)
        print(json.dumps(result, ensure_ascii=False, indent=2))
        return 0 if result["valid"] else 1
    if args.command == "inspect":
        _, meta, states, frames = load_episode(args.episode)
        print(json.dumps({**meta, "stored_states": len(states), "stored_frames": len(frames)}, ensure_ascii=False, indent=2))
    else:
        replay(args.episode, speed=args.speed, headless=args.headless, telemetry=args.telemetry)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except KeyboardInterrupt:
        raise SystemExit(130)
    except (OSError, ValueError, KeyError) as exc:
        raise SystemExit(str(exc))
