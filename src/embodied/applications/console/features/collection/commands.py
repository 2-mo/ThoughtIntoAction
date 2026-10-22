from embodied.applications.console.protocol import CommandSpec, FeatureSpec


FEATURE = FeatureSpec(
    name="collection",
    description="GR00T episode recording, validation, and review.",
    commands=(
        CommandSpec("prepare", "collection_prepare", "Prepare arm collection mode."),
        CommandSpec("status", "collection_status", "Read collection state.", False),
        CommandSpec("start", "collection_start", "Start an episode."),
        CommandSpec("stop", "collection_stop", "Stop and validate an episode."),
        CommandSpec("accept", "collection_accept", "Accept a reviewed episode."),
        CommandSpec("reject", "collection_reject", "Reject a reviewed episode."),
    ),
)
