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
    """A starts/saves; B prepositions or homes; the stick manages tasks."""

    def __init__(
        self,
        *args,
        home_joints: tuple[float, float, float, float, float, float],
        grasp_ready_joints: tuple[float, float, float, float, float, float],
        home_hold_s: float,
        preset_tolerance_rad: float,
        collection_task: str = "X5 遥操采集",
        home_speed_rad_s: float = .3,
        home_acceleration_rad_s2: float = .5,
        preset_speed_rad_s: float = .5,
        preset_acceleration_rad_s2: float = 1.,
        **kwargs,
    ) -> None:
        if not isinstance(collection_task, str) or not collection_task.strip() or len(collection_task) > 2000:
            raise ValueError("collection_task must contain 1..2000 characters")
        if home_hold_s <= 0.0:
            raise ValueError("home_hold_s must be positive")
        if preset_tolerance_rad <= 0.0:
            raise ValueError("preset_tolerance_rad must be positive")

        self._home_joints = np.asarray(home_joints, dtype=float)
        if not np.isfinite(preset_speed_rad_s) or preset_speed_rad_s < 0:
            raise ValueError("Preset speed must be finite and nonnegative; 0 uses the session speed limit")
        for value in (home_speed_rad_s, home_acceleration_rad_s2, preset_acceleration_rad_s2):
            if not np.isfinite(value) or value <= 0:
                raise ValueError("Preset speed and acceleration must be finite and positive")
        self._preset_rates = {"HOME": (home_speed_rad_s, home_acceleration_rad_s2),
                              "GRASP_READY": (preset_speed_rad_s, preset_acceleration_rad_s2)}
        self._grasp_ready_joints = np.asarray(grasp_ready_joints, dtype=float)
        self._home_hold_s = float(home_hold_s)
        self._preset_tolerance_rad = float(preset_tolerance_rad)

        self._robot_replay = None
        self._replay_loading = False
        self._replay_generation = 0
        self._replay_closed = False
        self._replay_error = None
        self._preset_lock = threading.Lock()
        self._episode_position_pending = None
        self._record_after_position = False
        self._preset_target: np.ndarray | None = None
        self._preset_name: str | None = None
        self._reset_buttons()
        self._collection = EpisodeActions(lambda: self._session, collection_task)

        kwargs.update(home_speed_rad_s=home_speed_rad_s, home_acceleration_rad_s2=home_acceleration_rad_s2,
                      preset_speed_rad_s=preset_speed_rad_s, preset_acceleration_rad_s2=preset_acceleration_rad_s2)
        super().__init__(*args, **kwargs)
        self._session_control["presets"] = {
            "home_joints": self._home_joints.tolist(), "grasp_ready_joints": self._grasp_ready_joints.tolist(),
            "home_hold_s": self._home_hold_s, "tolerance_rad": self._preset_tolerance_rad,
            "motion_limits": self._preset_rates,
        }
        try:
            self._validate_preset("HOME", self._home_joints)
            self._validate_preset("GRASP_READY", self._grasp_ready_joints)
        except BaseException:
            self.close()
            raise

        print("  X: 网页回放上一条；Y: 真机回放 / 取消；回放前先保存，Grip / B 取消真机回放")
        print("Quest controls: A -> pregrasp then record / save then pregrasp; B short -> GRASP_READY")
        print(f"  B hold {self._home_hold_s:.1f}s: discard current episode, then HOME (release Grip)")
        print("  Right stick push: left start/resume task; right save/pause; up discard previous")
        print("  Stick click hold 1s: save/end task and reset count; down hold 1s: toggle control / standby")
        print("  Grip cancels preset and pending recording. Return stick to center between actions.")

    def _collection_status(self):
        status = self._collection.status()
        status["control_enabled"] = self._control_enabled
        status["control_stopping"] = any(arm.info.get("mode") == "stopping" for arm in self.arm_controllers.values())
        if self._episode_position_pending:
            status["state"] = "HOME_WAIT"
        if self._preset_name is not None:
            status["state"] = "HOMING" if self._preset_name == "HOME" else "POSITIONING"
        if self._robot_replay is not None:
            status["state"] = "REPLAY_POSITIONING" if self._robot_replay.phase == "positioning" else "REPLAYING"
        elif self._replay_loading:
            status["state"] = "REPLAY_LOADING"
        if self._replay_error: status["error"] = self._replay_error
        return status

    def button_feedback(self):
        now = time.monotonic()
        def hold(press):
            elapsed = max(0., now - press.started) if press.started is not None else 0.
            return {"progress": min(1., elapsed / press.long_s) if press.long_s else 0.,
                    "fired": press.fired and press.started is not None,
                    "eligible": press.allowed and press.armed}
        gate = self._activation_gate
        return {"B": hold(self._b_press), "R3": hold(self._task_end_press),
                "down": {"progress": min(1., max(0., now - gate.down_since)) if gate.down_since is not None else 0.,
                         "eligible": gate.centered},
                "Y": getattr(self, "_y_feedback", "等待 Y：按下后松开触发真机回放")}

    def _reset_buttons(self):
        self._x_press = Press()
        self._y_press = Press()
        self._a_press = Press()
        self._b_press = Press(long_s=self._home_hold_s)
        self._task_end_press = Press(long_s=1.)
        self._stick_action = StickAction()

    def close(self):
        self._replay_closed = True
        self._replay_generation += 1
        actions = getattr(self, "_collection", None)
        if actions:
            actions.close()
        super().close()

    def _control_gate_ready(self):
        now = time.monotonic()
        x = bool(self.xr_client.get_button_state_by_name("X"))
        y = bool(self.xr_client.get_button_state_by_name("Y"))
        if self._x_press.update(x, now, enabled=not y) == "short":
            self._collection.submit("replay_previous")
        if y:
            self._y_feedback = "Y 已按下，请松开触发" if not x else "请松开 X 后单独按 Y"
        if self._y_press.update(y, now, enabled=not x) == "short":
            self._y_feedback = "Y 已识别，正在检查回放条件"
            if self._robot_replay is not None or self._replay_loading:
                self._cancel_robot_replay("Y 取消")
            elif not self._control_enabled:
                self._y_feedback = "真机回放：请先下拨 1 秒启用控制"
                self._replay_error = self._y_feedback
                print(self._y_feedback, flush=True)
            elif self._episode_position_pending or self._preset_target is not None or self._collection.busy or self._collection.recording:
                self._y_feedback = "真机回放：请先完成并保存当前操作"
                self._replay_error = self._y_feedback
                print(self._y_feedback, flush=True)
            elif any(self.xr_client.get_key_value_by_name(key) > .1 for key in ("right_grip", "right_trigger")):
                self._y_feedback = "真机回放：请松开右 Grip 和扳机"
                self._replay_error = self._y_feedback
                print(self._y_feedback, flush=True)
            else:
                self._prepare_robot_replay()
        return super()._control_gate_ready()

    def _prepare_robot_replay(self):
        self._y_feedback = "Y 已响应，正在加载数据"
        self._replay_loading, self._replay_error = True, None
        self._replay_generation += 1
        generation = self._replay_generation
        def prepare():
            try:
                if self._session is None: raise ValueError("采集会话未启用")
                episode = self._session.load_previous_episode()
                from .replay import RobotReplay
                with self._state_lock:
                    if generation != self._replay_generation or self._replay_closed: return
                    if not self._control_enabled or self._collection.recording: return
                    self._reset_teleop_reference()
                    self._robot_replay = RobotReplay(episode)
                    print("真机回放：前往上一条起点；Y / Grip / B 可取消", flush=True)
            except Exception as exc:
                with self._state_lock:
                    if generation == self._replay_generation:
                        self._replay_error = str(exc)
                        print("真机回放未开始：" + str(exc), flush=True)
            finally:
                with self._state_lock:
                    if generation == self._replay_generation: self._replay_loading = False
        threading.Thread(target=prepare, name="prepare-robot-replay", daemon=True).start()

    def _cancel_robot_replay(self, reason):
        if self._robot_replay is None and not self._replay_loading: return
        self._replay_generation += 1
        self._replay_loading = False
        self._robot_replay = None
        for arm in self.arm_controllers.values():
            arm.stop_control()
        self._reset_teleop_reference()
        print("真机回放取消：" + reason, flush=True)

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
        self._cancel_robot_replay("切换预设")
        self._reset_teleop_reference()
        with self._preset_lock:
            self._preset_name = name
            self._preset_target = target.copy()
        print(f"PRESET {name} started: q={np.round(target, 4)}")

    def _cancel_preset(self, reason: str) -> None:
        self._cancel_robot_replay(reason)
        # Grip cancels a moving preset; while waiting for its release, retain A intent.
        if reason != "Grip activated" or self._preset_target is not None:
            self._episode_position_pending = None
            self._record_after_position = False
        if reason in ("Headset pause", "Control standby"):
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
            self._reset_teleop_reference()
        print(f"PRESET {expected_name} reached")
        if expected_name == "GRASP_READY" and self._record_after_position:
            self._record_after_position = False
            if self._control_enabled and not self._collection.recording:
                self._collection.submit("toggle_episode")

    def _pre_ik_update(self) -> None:
        if not self._control_enabled:
            return
        now = time.monotonic()
        a = bool(self.xr_client.get_button_state_by_name("A"))
        b = bool(self.xr_client.get_button_state_by_name("B"))
        enabled = self._episode_position_pending is None and self._preset_target is None and not self._collection.busy and self._robot_replay is None and not self._replay_loading
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
            if self._collection.recording:
                print("请先按 A 保存当前采集，再按 B 到预抓取位", flush=True)
            elif not grip_active and not (a or stick) and not self._collection.busy:
                self._start_preset("GRASP_READY", self._grasp_ready_joints)
        elif direction_event:
            self._collection.submit(direction_event)
        elif a_event and not (b or stick) and not self._collection.busy:
            if self._collection.recording:
                if self._collection.submit("toggle_episode"):
                    self._episode_position_pending = "saved"
            else:
                self._collection.clear_error()
                self._episode_position_pending = "start"
        if self._episode_position_pending and not self._collection.busy:
            if self._collection.error:
                self._episode_position_pending = None
            elif not grip_active and not (a or b or stick) and (self._episode_position_pending != "saved" or not self._collection.recording):
                start_recording = self._episode_position_pending == "start"
                self._episode_position_pending = None
                self._start_preset("GRASP_READY", self._grasp_ready_joints)
                self._record_after_position = start_recording
        if not grip_active and self._collection.take_home():
            self._start_preset("HOME", self._home_joints)

    def _send_command(self) -> None:
        if not self._control_enabled:
            self._publish_runtime()
            return
        if self._robot_replay is not None:
            try:
                if not self._robot_replay.tick(self.arm_controllers["right_arm"], time.monotonic()):
                    self._robot_replay = None
                    self._reset_teleop_reference()
                    print("真机回放完成", flush=True)
            except Exception as exc:
                self._replay_error = str(exc)
                self._cancel_robot_replay(str(exc))
            self._publish_runtime()
            return
        self._ik_limit_report = None
        """Preset targets take priority; otherwise preserve stock teleoperation."""
        with self._preset_lock:
            preset_target = (
                None if self._preset_target is None else self._preset_target.copy()
            )
            preset_name = self._preset_name

        for arm_name, controller in self.arm_controllers.items():
            if preset_target is not None:
                controller.set_joint_positions(preset_target, profile="home" if preset_name == "HOME" else "preset")
                q_actual = controller.get_joint_positions()[:6]
                error = float(np.max(np.abs(q_actual - preset_target)))
                state = controller.snapshot()
                planner = state.get("trajectory") or {}
                finished = planner.get("finished", False) and not planner.get("stopping", True)
                if finished and np.max(np.abs(state["actual_vel"])) < .03 and error <= self._preset_tolerance_rad and preset_name is not None:
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
