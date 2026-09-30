from ...protocol import FeatureSpec


FEATURE = FeatureSpec(
    name="teleoperation",
    description="Low-latency hand-tracking target stream.",
    streams=("target",),
    commands=(),
)
