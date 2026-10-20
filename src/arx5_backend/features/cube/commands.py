from arx5_backend.protocol import CommandSpec, FeatureSpec


FEATURE = FeatureSpec(
    name="cube",
    description="Previewed cube pick, transfer, placement, and stacking stages.",
    commands=(
        CommandSpec("preview-pick", "cube_pick_preview", "Plan a cube grasp.", False),
        CommandSpec("pick", "cube_pick_execute", "Execute a reviewed grasp."),
        CommandSpec("preview-transfer", "cube_move_preview", "Plan transfer above a target.", False),
        CommandSpec("transfer", "cube_move_execute", "Execute the reviewed transfer."),
        CommandSpec("preview-place", "cube_place_preview", "Plan cube release.", False),
        CommandSpec("place", "cube_place_execute", "Execute the reviewed release."),
        CommandSpec(
            "preview-stack",
            "cube_stack_preview",
            "Plan the complete continuous stacking task.",
            False,
        ),
        CommandSpec(
            "stack",
            "cube_stack_execute",
            "Execute the complete reviewed stacking task.",
        ),
    ),
)
