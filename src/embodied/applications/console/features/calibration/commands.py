from embodied.applications.console.protocol import CommandSpec, FeatureSpec


FEATURE = FeatureSpec(
    name="calibration",
    description="Independent payload, gripper, and hand-eye calibration workflows.",
    commands=(
        CommandSpec("payload-start", "payload_calibration_start", "Start payload calibration."),
        CommandSpec("payload-confirm", "payload_calibration_confirm", "Advance payload calibration."),
        CommandSpec("payload-cancel", "payload_calibration_cancel", "Cancel payload calibration."),
        CommandSpec("gripper-start", "gripper_calibration_start", "Start gripper calibration."),
        CommandSpec("gripper-confirm", "gripper_calibration_confirm", "Advance gripper calibration."),
        CommandSpec("gripper-cancel", "gripper_calibration_cancel", "Cancel gripper calibration."),
        CommandSpec("hand-eye-start", "handeye_calibration_start", "Start hand-eye calibration."),
        CommandSpec("hand-eye-confirm", "handeye_calibration_confirm", "Advance hand-eye calibration."),
        CommandSpec("hand-eye-cancel", "handeye_calibration_cancel", "Cancel hand-eye calibration."),
        CommandSpec("hand-eye-validate", "handeye_validate", "Validate the latest result.", False),
        CommandSpec("hand-eye-enable", "handeye_enable", "Enable a validated result."),
        CommandSpec("hand-eye-disable", "handeye_disable", "Disable the active result."),
    ),
)
