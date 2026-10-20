"""Stationary, multi-frame observations with auditable raw images and robot brackets."""

from pathlib import Path
import time

import cv2
import numpy as np

from embodied.core.storage import save_json
from embodied.core.transforms import mean_pose, pose_error


class Session:
    def __init__(self, directory, robot, camera, target):
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=False)
        self.robot = robot
        self.data = {"schema_version": 1, "kind": "eye_in_hand_samples",
                     "created_unix_ns": time.time_ns(), "status": "collecting",
                     "robot": robot.metadata(), "camera": camera,
                     "target": target, "samples": [],
                     "timing": "Stationary bursts, robot reads bracket RGB receipt; no hardware clock synchronization"}
        self.save()

    def save(self):
        save_json(self.directory / "session.json", self.data)

    def finish(self, status, error=None):
        self.data["status"] = status
        if error is not None:
            self.data["error"] = str(error)
        self.save()

    def capture(self, camera, arm, detector, split, waypoint=None):
        if split not in {"fit", "validation"}:
            raise ValueError("Unknown sample split")
        anchor = np.asarray(arm.snapshot()["q"])
        # Discard queued pre-settle frames; retained frames must all pass a stationary bracket.
        for _ in range(3):
            camera.read()
        frames, rejections = [], []
        for _ in range(20):
            before = arm.snapshot()
            frame = camera.read()
            after = arm.snapshot()
            if (after["host_monotonic_ns"] - before["host_monotonic_ns"]) > 150_000_000:
                raise ValueError("RGB/robot bracket took >150 ms")
            qs = np.asarray([before["q"], after["q"]])
            if (np.max(np.abs(qs - anchor)) > 0.006
                    or np.max(np.abs(qs[0] - qs[1])) > 0.003
                    or np.max(np.abs([before["dq"], after["dq"]])) > 0.02):
                raise ValueError("Robot moved during capture; entire burst rejected")
            try:
                detection = detector.detect(frame["image"])
            except ValueError as exc:
                rejections.append(str(exc))
                continue
            frames.append({**frame, **detection, "robot_before": before, "robot_after": after})
        if len(frames) < 12:
            raise ValueError(f"Only {len(frames)}/20 valid marker frames (need 12): {rejections[:2]}")
        camera_pose = mean_pose([frame["T_camera_target"] for frame in frames])
        errors = np.asarray([pose_error(camera_pose, frame["T_camera_target"]) for frame in frames])
        if np.max(errors[:, 0]) > 0.003 or np.max(errors[:, 1]) > 1.0:
            raise ValueError("Marker/camera pose unstable within burst; sample rejected")
        q = np.mean([frame["robot_after"]["q"] for frame in frames], axis=0)
        pose = self.robot.fk(q)
        for previous in self.data["samples"]:
            distance, angle = pose_error(previous["T_base_eef"], pose)
            if distance < 0.001 and angle < 0.5:
                raise ValueError("Pose already captured; move to a distinct pose")
        index = len(self.data["samples"])
        folder = self.directory / f"sample_{index:04d}"
        folder.mkdir()
        raw = []
        for offset, frame in enumerate(frames):
            relative = f"{folder.name}/rgb_{offset:03d}.png"
            if not cv2.imwrite(str(self.directory / relative), frame["image"]):
                raise OSError("Failed to save raw RGB frame")
            raw.append({**{k: v for k, v in frame.items() if k not in {"image", "T_camera_target"}},
                        "T_camera_target": frame["T_camera_target"].tolist(), "image": relative})
        row = {"id": index, "split": split, "waypoint_index": waypoint, "q": q.tolist(),
               "T_base_eef": pose.tolist(), "T_camera_target": camera_pose.tolist(),
               "frames": raw, "rejected_frame_reasons": rejections,
               "burst_translation_max_m": float(np.max(errors[:, 0])),
               "burst_rotation_max_deg": float(np.max(errors[:, 1]))}
        self.data["samples"].append(row)
        self.save()
        print(f"Saved {split} pose {index}: {len(frames)} RGB frames", flush=True)
        return row
