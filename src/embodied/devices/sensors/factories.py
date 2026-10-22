"""Camera owners for the shared service; SDKs load only for the selected driver."""


def realsense(*, options):
    from .realsense import RealSenseCamera

    return RealSenseCamera(
        options["serial"], options["width"], options["height"], options["fps"],
        expected_model=options.get("model"), color_options=options.get("color", {}),
    )


def synthetic(*, options):
    from .synthetic import SyntheticCamera

    return SyntheticCamera(**{key: options[key] for key in ("serial", "width", "height", "fps")})
