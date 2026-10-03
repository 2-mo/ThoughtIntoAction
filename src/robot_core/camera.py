"""Single-owner RealSense RGB stream with explicit receive/device clock metadata."""

import time

import numpy as np

RGB_OPTIONS = ("enable_auto_white_balance", "white_balance", "enable_auto_exposure",
               "exposure", "gain", "brightness", "contrast", "saturation", "gamma",
               "hue", "sharpness", "backlight_compensation", "power_line_frequency",
               "auto_exposure_priority")


def configure_rgb(sensor, rs, requested):
    """Validate all settings before writing; auto modes precede manual values."""
    if not isinstance(requested, dict) or set(requested) - set(RGB_OPTIONS):
        raise ValueError("Unknown RGB camera settings")
    for name, value in requested.items():
        option = getattr(rs.option, name)
        if not sensor.supports(option) or sensor.is_option_read_only(option):
            raise ValueError(f"RGB option is not writable: {name}")
        bounds = sensor.get_option_range(option)
        if not isinstance(value, (int, float)) or not np.isfinite(value) or not bounds.min <= value <= bounds.max:
            raise ValueError(f"RGB {name} must be between {bounds.min} and {bounds.max}")
        if bounds.step and not np.isclose((value - bounds.min) / bounds.step, round((value - bounds.min) / bounds.step)):
            raise ValueError(f"RGB {name} must use step {bounds.step}")
    for name in RGB_OPTIONS:
        if name in requested:
            option = getattr(rs.option, name)
            value = float(requested[name])
            # Avoid redundant USB control writes; read after any auto-mode change.
            if sensor.get_option(option) != value:
                sensor.set_option(option, value)
    return {name: float(sensor.get_option(getattr(rs.option, name)))
            for name in RGB_OPTIONS if sensor.supports(getattr(rs.option, name))}


class RealSenseCamera:
    def __init__(self, serial, width=640, height=480, fps=30, *, expected_model="D435i", color_options=None):
        if not serial:
            raise ValueError("Set cameras.<camera_id>.serial; camera selection must be explicit")
        self.serial, self.width, self.height, self.fps = serial, width, height, fps
        self.expected_model = expected_model
        self.color_options = {} if color_options is None else color_options
        self.pipeline = None
        self.last_number = None

    def __enter__(self):
        import pyrealsense2 as rs
        # Reject a missing/wrong device before pipeline.start can wait for it and
        # stall other camera feeds or the service's IPC handlers.
        context = rs.context()
        devices = [d for d in context.query_devices()
                   if d.get_info(rs.camera_info.serial_number) == self.serial]
        if not devices:
            raise RuntimeError(f"相机未连接：{self.serial}")
        model = devices[0].get_info(rs.camera_info.name)
        if self.expected_model and not model.upper().endswith(" " + self.expected_model.upper()):
            raise ValueError(f"Expected {self.expected_model}, got {model} ({self.serial})")
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
            color_sensor = device.first_color_sensor()
            rgb_controls = configure_rgb(color_sensor, rs, self.color_options)
            self.depth_scale = device.first_depth_sensor().get_depth_scale()
            self.metadata = {
                "depth_scale_m": self.depth_scale, "depth_frame": "depth_optical",
                "depth_aligned_to_rgb": False,
                "serial": device.get_info(rs.camera_info.serial_number),
                "model": device.get_info(rs.camera_info.name),
                "usb_type": device.get_info(rs.camera_info.usb_type_descriptor),
                "width": intr.width, "height": intr.height, "fps": self.fps,
                "rgb_controls": rgb_controls, "rgb_exposure_unit_us": 100,
                "matrix": [[intr.fx, 0, intr.ppx], [0, intr.fy, intr.ppy], [0, 0, 1]],
                "distortion_model": str(intr.model), "coefficients": list(intr.coeffs),
                "format": "bgr8", "frame": "color_optical",
                "host_clock": "monotonic_ns at frame receipt (not exposure time)",
            }
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
