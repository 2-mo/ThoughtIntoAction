"""Quest A/B preset state machine over the project's X5 controller."""

from .collection import Press, EpisodeActions, StickAction

import threading
import time

import numpy as np

from .controller import X5TeleopController


DEFAULT_HOME_JOINTS = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
# With the Stanford X5 kinematic chain used by X5_2025_hybrid.urdf, this
# configuration points the eef/gripper x-axis approximately straight down
# while keeping the arm forward of the base. Treat it as a starting preset and
# override it after validating the exact hardware setup if needed.
DEFAULT_GRASP_READY_JOINTS = (0.0, 1.50, 1.45, -1.52, 0.0, 0.0)


class PresetSingleX5Controller(X5TeleopController):
    """A prepositions then records; B saves or homes; the stick manages tasks."""

    def __init__(
        self,
        *args,
        home_joints: tuple[float, float, float, float, float, float],
        grasp_ready_joints: tuple[float, float, float, float, float, float],
        home_hold_s: float,
        preset_tolerance_rad: float,
        collection_task: str = "X5 遥操采集",
        **kwargs,
    ) -> None:
        if not isinstance(collection_task, str) or not collection_task.strip() or len(collection_task) > 2000:
            raise ValueError("collection_task must contain 1..2000 characters")
        if home_hold_s <= 0.0:
            raise ValueError("home_hold_s must be positive")
        if preset_tolerance_rad <= 0.0:
            raise ValueError("preset_tolerance_rad must be positive")

        self._home_joints = np.asarray(home_joints, dtype=float)
        self._grasp_ready_joints = np.asarray(grasp_ready_joints, dtype=float)
        self._home_hold_s = float(home_hold_s)
        self._preset_tolerance_rad = float(preset_tolerance_rad)

        self._record_on_arrival = False
        self._start_recording_ready = False
        self._preset_lock = threading.Lock()
        self._preset_target: np.ndarray | None = None
        self._preset_name: str | None = None
        self._reset_buttons()
        self._collection = EpisodeActions(lambda: self._session, collection_task)

        super().__init__(*args, **kwargs)
        self._session_control["presets"] = {
            "home_joints": self._home_joints.tolist(), "grasp_ready_joints": self._grasp_ready_joints.tolist(),
            "home_hold_s": self._home_hold_s, "tolerance_rad": self._preset_tolerance_rad,
        }
        try:
            self._validate_preset("HOME", self._home_joints)
            self._validate_preset("GRASP_READY", self._grasp_ready_joints)
        except BaseException:
            self.close()
            raise

        print("Quest controls: A -> GRASP_READY, then record if task active; B short -> save")
        print(f"  B hold {self._home_hold_s:.1f}s: discard current episode, then HOME (release Grip)")
        print("  Right stick push: left start/resume task; right save/pause; up discard previous")
        print("  Stick click hold 1s: save/end task and reset count; down unbound")
        print("  Grip cancels preset and pending recording. Return stick to center between actions.")

    def _collection_status(self):
        status = self._collection.status()
        if self._preset_name is not None:
            status["state"] = "HOMING" if self._preset_name == "HOME" else "POSITIONING"
        return status

    def _reset_buttons(self):
        self._a_press = Press()
        self._b_press = Press(long_s=self._home_hold_s)
        self._task_end_press = Press(long_s=1.)
        self._stick_action = StickAction()

    def close(self):
        actions = getattr(self, "_collection", None)
        if actions:
            actions.close()
        super().close()

    def _validate_preset(self, name: str, target: np.ndarray) -> None:
        if target.shape != (6,) or not np.all(np.isfinite(target)):
            raise ValueError(f"{name} preset must contain six finite joint values")
        controller = next(iter(self.arm_controllers.values()))
        q_min = np.asarray(controller.robot_config.joint_pos_min, dtype=float)
        q_max = np.asarray(controller.robot_config.joint_pos_max, dtype=float)
        if np.any(target < q_min) or np.any(target > q_max):
            raise ValueError(
                f"{name} preset is outside X5 joint limits: target={target}, "
                f"min={q_min}, max={q_max}"
            )

    def _reset_teleop_reference(self) -> None:
        """Force the next Grip activation to establish a fresh XR/robot anchor."""
        for arm_name in self.manipulator_config:
            self.active[arm_name] = False
            self.ref_ee_xyz[arm_name] = None
            self.ref_ee_quat[arm_name] = None
            self.ref_controller_xyz[arm_name] = None
            self.ref_controller_quat[arm_name] = None

    def _start_preset(self, name: str, target: np.ndarray) -> None:
        self._reset_teleop_reference()
        with self._preset_lock:
            self._preset_name = name
            self._preset_target = target.copy()
        print(f"PRESET {name} started: q={np.round(target, 4)}")

    def _cancel_preset(self, reason: str) -> None:
        self._record_on_arrival = self._start_recording_ready = False
        if reason == "Headset pause":
            self._collection.cancel_home()
            self._reset_buttons()
        with self._preset_lock:
            if self._preset_target is None:
                return
            name = self._preset_name
            self._preset_target = None
            self._preset_name = None
        self._reset_teleop_reference()
        print(f"PRESET {name} cancelled: {reason}")

    def _finish_preset(self, expected_name: str) -> None:
        with self._preset_lock:
            if self._preset_name != expected_name:
                return
            self._preset_target = None
            self._preset_name = None
            self._start_recording_ready = expected_name == "GRASP_READY" and self._record_on_arrival
            self._record_on_arrival = False
        self._reset_teleop_reference()
        print(f"PRESET {expected_name} reached")

    def _pre_ik_update(self) -> None:
        now = time.monotonic()
        a = bool(self.xr_client.get_button_state_by_name("A"))
        b = bool(self.xr_client.get_button_state_by_name("B"))
        enabled = self._preset_target is None and not self._collection.busy
        a_event = self._a_press.update(a, now, enabled=enabled)
        b_event = self._b_press.update(b, now, enabled=not self._collection.busy)
        stick = bool(self.xr_client.get_button_state_by_name("right_axis_click"))
        end_event = self._task_end_press.update(stick, now, enabled=not self._collection.busy)
        direction_event = self._stick_action.update(
            self.xr_client.get_joystick_state("right"), now,
            enabled=not self._collection.busy and not (a or b or stick))
        grip_active = self.xr_client.get_key_value_by_name(
            self.manipulator_config["right_arm"]["control_trigger"]) > .9
        if grip_active:
            self._cancel_preset("Grip activated")
        if b_event or end_event == "long" or direction_event in ("pause_task", "start_task"):
            self._cancel_preset("Collection action")
        if b_event == "long":
            self._collection.submit("home")
        elif end_event == "long":
            self._collection.submit("end_task")
        elif b_event == "short":
            self._collection.submit("stop_episode")
        elif direction_event:
            self._collection.submit(direction_event)
        elif a_event and not (b or stick or grip_active) and not self._collection.busy:
            if self._preset_target is None and not self._collection.recording:
                self._start_preset("GRASP_READY", self._grasp_ready_joints)
                self._record_on_arrival = self._collection.task_active
        if self._start_recording_ready:
            self._start_recording_ready = False
            if self._collection.task_active and not grip_active:
                self._collection.submit("start_episode")
        if not grip_active and self._collection.take_home():
            self._start_preset("HOME", self._home_joints)

    def _send_command(self) -> None:
        self._ik_limit_report = None
        """Preset targets take priority; otherwise preserve stock teleoperation."""
        with self._preset_lock:
            preset_target = (
                None if self._preset_target is None else self._preset_target.copy()
            )
            preset_name = self._preset_name

        for arm_name, controller in self.arm_controllers.items():
            if preset_target is not None:
                controller.set_joint_positions(preset_target)
                q_actual = controller.get_joint_positions()[:6]
                error = float(np.max(np.abs(q_actual - preset_target)))
                if error <= self._preset_tolerance_rad and preset_name is not None:
                    self._finish_preset(preset_name)
            elif self.active.get(arm_name, False):
                q_des = self._bounded_ik_command(arm_name)
                controller.set_joint_positions(q_des)

            if "gripper_config" in self.manipulator_config[arm_name]:
                gripper_config = self.manipulator_config[arm_name]["gripper_config"]
                joint_name = gripper_config["joint_names"][0]
                gripper_target = self.gripper_pos_target[arm_name][joint_name]
                controller.set_catch_pos(gripper_target)
        self._publish_runtime(preset_name=preset_name)
