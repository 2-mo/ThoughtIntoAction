// Browser-local selection. It never changes the live service or sends robot commands.
export let mode = 'live';
export let replayFrame = null;
export function setMode(value) {
  mode = value;
  document.body.dataset.view = mode;
  window.dispatchEvent(new CustomEvent('view-mode', {detail: mode}));
}
export function setReplay(frame) { replayFrame = frame; }
export function displayedState(live) {
  if (mode !== 'replay') return live;
  const f = replayFrame;
  if (!f?.pose) return {actual: null, target: null, paths: {}, stream: {status: 'waiting', source: {kind: 'replay'}}};
  return {sequence: f.sample_index, actual: f.pose, target: f.target_pose || {...f.pose, q: null},
    target_visible: !!f.target_pose && f.q.some((q, i) => Math.abs(q - f.target_q[i]) > .005),
    paths: {recorded: {points: f.path_points || [], color: 0x91b8aa}},
    actual_vel: f.actual_vel, target_vel: f.target_vel, eef_velocity: f.eef_velocity,
    actual_torque: f.actual_torque, gripper_actual_torque: f.gripper_actual_torque,
    gripper_actual_vel: f.gripper_actual_vel, motor_diagnostics: f.motor_diagnostics,
    command_details: f.command_details,
    gripper: {actual: f.gripper_m, target: f.target_gripper_m, max: f.gripper_max},
    stream: {status: 'live', age_s: 0, stream_id: String(f.generation), sequence: f.sample_index, source: {kind: 'replay', input: f.episode}}};
}
