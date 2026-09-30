export interface PublicCommand {
  feature: string;
  command: string;
}

const ACTIONS: Record<string, PublicCommand> = {
  connect: { feature: "arm", command: "connect" },
  disconnect: { feature: "arm", command: "disconnect" },
  state: { feature: "arm", command: "state" },
  emergency: { feature: "arm", command: "emergency-damping" },
  activate: { feature: "arm", command: "activate" },
  damping: { feature: "arm", command: "damping" },
  clear_fault: { feature: "arm", command: "clear-fault" },
  set_reference: { feature: "arm", command: "set-reference" },
  preview_park: { feature: "arm", command: "preview-reference" },
  park: { feature: "arm", command: "return-reference" },
  preview_xyz: { feature: "arm", command: "preview-pose" },
  preview_xyz_live: { feature: "arm", command: "preview-pose-live" },
  move_xyz: { feature: "arm", command: "move-pose" },
  move_joint: { feature: "arm", command: "move-joint" },
  move_joint_target: { feature: "arm", command: "move-joint-target" },
  reset_joint: { feature: "arm", command: "reset-joint" },
  gripper_position: { feature: "gripper", command: "move" },
  reset_gripper: { feature: "gripper", command: "close" },
  gripper_force: { feature: "gripper", command: "force-close" },
  cube_pick_preview: { feature: "cube", command: "preview-pick" },
  cube_pick_execute: { feature: "cube", command: "pick" },
  cube_move_preview: { feature: "cube", command: "preview-transfer" },
  cube_move_execute: { feature: "cube", command: "transfer" },
  cube_place_preview: { feature: "cube", command: "preview-place" },
  cube_place_execute: { feature: "cube", command: "place" },
  cube_stack_preview: { feature: "cube", command: "preview-stack" },
  cube_stack_execute: { feature: "cube", command: "stack" },
  payload_calibration_start: { feature: "calibration", command: "payload-start" },
  payload_calibration_confirm: { feature: "calibration", command: "payload-confirm" },
  payload_calibration_cancel: { feature: "calibration", command: "payload-cancel" },
  gripper_calibration_start: { feature: "calibration", command: "gripper-start" },
  gripper_calibration_confirm: { feature: "calibration", command: "gripper-confirm" },
  gripper_calibration_cancel: { feature: "calibration", command: "gripper-cancel" },
  handeye_calibration_start: { feature: "calibration", command: "hand-eye-start" },
  handeye_calibration_confirm: { feature: "calibration", command: "hand-eye-confirm" },
  handeye_calibration_cancel: { feature: "calibration", command: "hand-eye-cancel" },
  handeye_validate: { feature: "calibration", command: "hand-eye-validate" },
  handeye_enable: { feature: "calibration", command: "hand-eye-enable" },
  handeye_disable: { feature: "calibration", command: "hand-eye-disable" },
  collection_status: { feature: "collection", command: "status" },
  collection_prepare: { feature: "collection", command: "prepare" },
  collection_start: { feature: "collection", command: "start" },
  collection_stop: { feature: "collection", command: "stop" },
  collection_accept: { feature: "collection", command: "accept" },
  collection_reject: { feature: "collection", command: "reject" },
};

const PUBLIC_NAMES = new Map(
  Object.entries(ACTIONS).map(([action, value]) => [
    `${value.feature}.${value.command}`,
    action,
  ]),
);

export function controlEndpoint(): string {
  const scheme = window.location.protocol === "https:" ? "wss" : "ws";
  const url = new URL("/api/v1/control", window.location.href);
  url.protocol = `${scheme}:`;
  const token = new URLSearchParams(window.location.search).get("token");
  if (token) url.searchParams.set("token", token);
  return url.toString();
}

export function serviceEndpoint(port: number, path: string): string {
  const scheme = window.location.protocol === "https:" ? "wss" : "ws";
  const url = new URL(`${scheme}://${window.location.hostname}:${port}${path}`);
  const token = new URLSearchParams(window.location.search).get("token");
  if (token) url.searchParams.set("token", token);
  return url.toString();
}

export function encodeCommand(action: string, payload: object, requestId: string | number): string {
  const target = ACTIONS[action];
  if (!target) throw new Error(`未注册的控制命令：${action}`);
  return JSON.stringify({
    type: "command",
    request_id: requestId,
    ...target,
    payload,
  });
}

export function encodeStream(
  feature: "arm" | "gripper" | "teleoperation",
  stream: string,
  command: "start" | "update" | "stop",
  sequence: number,
  payload: object,
): string {
  return JSON.stringify({
    type: "stream",
    feature,
    stream,
    command,
    sequence,
    payload,
  });
}

export function decodeControlEvent(serialized: string): Record<string, any> {
  const event = JSON.parse(serialized);
  const data = event.data ?? {};
  const action = PUBLIC_NAMES.get(`${event.feature}.${event.command}`) ?? event.command;
  const base = {
    ...data,
    ...(event.request_id !== undefined ? { id: event.request_id } : {}),
    ...(action ? { action } : {}),
    ...(event.ok !== undefined ? { ok: event.ok } : {}),
  };
  const names: Record<string, string> = {
    "session.ready": "hello",
    "session.fatal": "fatal",
    "state.snapshot": "state",
    "motion.progress": "motion_state",
    "command.started": "command_started",
    "command.succeeded": "command_result",
    "command.failed": "command_result",
    "teleoperation.state": "teleop_state",
    "servo.state": "servo_state",
    "calibration.payload": "payload_calibration",
    "calibration.gripper": "gripper_calibration",
    "calibration.hand-eye": "handeye_calibration",
    "calibration.hand-eye-plan": "handeye_plan",
    "calibration.hand-eye-preview": "handeye_preview",
    "calibration.hand-eye-validation": "handeye_validation",
  };
  return { type: names[event.type] ?? event.type, ...base };
}
