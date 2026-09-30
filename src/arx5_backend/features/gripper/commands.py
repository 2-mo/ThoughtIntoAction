from ...protocol import CommandSpec, FeatureSpec


FEATURE = FeatureSpec(
    name="gripper",
    description="Position, force, reset, and live gripper control.",
    streams=("position",),
    commands=(
        CommandSpec("move", "gripper_position", "Move to an opening width."),
        CommandSpec("close", "reset_gripper", "Close to zero width."),
        CommandSpec("force-close", "gripper_force", "Close with a torque limit."),
    ),
)
