"""Single-owner RealSense RGB stream with explicit receive/device clock metadata."""

import time

import numpy as np


class RealSenseCamera:
    def __init__(self, serial, width=640, height=480, fps=30):
        if not serial:
            raise ValueError("Set cameras.wrist_rgb.serial; camera selection must be explicit")
        self.serial, self.width, self.height, self.fps = serial, width, height, fps
        self.pipeline = None
        self.last_number = None

    def __enter__(self):
        import pyrealsense2 as rs
        pipeline, config = rs.pipeline(), rs.config()
        config.enable_device(self.serial)
        config.enable_stream(rs.stream.depth, self.width, self.height, rs.format.z16, self.fps)
        config.enable_stream(rs.stream.color, self.width, self.height, rs.format.bgr8, self.fps)
        started = False
        try:
            profile = pipeline.start(config)
            started = True
            intr = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            device = profile.get_device()
            self.depth_scale = device.first_depth_sensor().get_depth_scale()
            self.metadata = {
                "depth_scale_m": self.depth_scale, "depth_frame": "depth_optical",
                "depth_aligned_to_rgb": False,
                "serial": device.get_info(rs.camera_info.serial_number),
                "model": device.get_info(rs.camera_info.name),
                "width": intr.width, "height": intr.height, "fps": self.fps,
                "matrix": [[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]],
                "distortion_model": str(intr.model), "coefficients": list(intr.coeffs),
                "format": "bgr8", "frame": "color_optical",
                "host_clock": "monotonic_ns at frame receipt (not exposure time)",
            }
            if "D435I" not in self.metadata["model"].upper():
                raise ValueError(f"Expected wrist D435i, got {self.metadata['model']}")
            self.pipeline = pipeline
            # Allow auto-exposure to settle; never infer intrinsics from requested resolution.
            for _ in range(30):
                self.read()
            return self
        except BaseException:
            if started:
                pipeline.stop()
            self.pipeline = None
            raise

    def read(self):
        frames = self.pipeline.wait_for_frames(2000)
        frame = frames.get_color_frame()
        depth = frames.get_depth_frame()
        received = time.monotonic_ns()
        if not frame:
            raise RuntimeError("Missing RGB frame")
        number = frame.get_frame_number()
        if self.last_number is not None and number <= self.last_number:
            raise RuntimeError("Non-increasing camera frame number")
        self.last_number = number
        return {"depth_image": None if not depth else np.asanyarray(depth.get_data()).copy(),
                "depth_scale_m": self.depth_scale,
                "image": np.asanyarray(frame.get_data()).copy(), "frame_number": number,
                "device_timestamp_ms": float(frame.get_timestamp()),
                "device_timestamp_domain": str(frame.get_frame_timestamp_domain()),
                "host_received_monotonic_ns": received}

    def __exit__(self, *exc):
        if self.pipeline is not None:
            self.pipeline.stop()
            self.pipeline = None
