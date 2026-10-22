from embodied.applications.console.protocol import CommandSpec, FeatureSpec


FEATURE = FeatureSpec(
    name="arm",
    description="Connection, state, motion preview, execution, and safety controls.",
    streams=("cartesian",),
    commands=(
        CommandSpec("connect", "connect", "Connect without moving the arm."),
        CommandSpec("disconnect", "disconnect", "Release the controller safely."),
        CommandSpec("state", "state", "Read the latest arm state.", False),
        CommandSpec("emergency-damping", "emergency", "Enter emergency damping immediately."),
        CommandSpec("activate", "activate", "Hold the current pose."),
        CommandSpec("damping", "damping", "Enter normal damping mode."),
        CommandSpec("clear-fault", "clear_fault", "Clear a controller fault."),
        CommandSpec("set-reference", "set_reference", "Record the current software reference."),
        CommandSpec("preview-reference", "preview_park", "Preview return to reference.", False),
        CommandSpec("return-reference", "park", "Execute the reviewed return trajectory."),
        CommandSpec("preview-pose", "preview_xyz", "Preview an end-effector target.", False),
        CommandSpec("preview-pose-live", "preview_xyz_live", "Solve a display-only live preview.", False),
        CommandSpec("move-pose", "move_xyz", "Execute the reviewed end-effector motion."),
        CommandSpec("move-joint", "move_joint", "Move one joint by a bounded delta."),
        CommandSpec("move-joint-target", "move_joint_target", "Move one joint to a target."),
        CommandSpec("reset-joint", "reset_joint", "Return one joint to its reference."),
    ),
)
