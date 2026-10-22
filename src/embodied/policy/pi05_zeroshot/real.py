"""Move X5 to the shared pregrasp pose and capture a live PI05 observation."""
import argparse
import json
from pathlib import Path
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[4]

import numpy as np
from legacy.robot_control.client import ArmClient
from embodied.devices.robots.arx5.pinocchio_model import X5PinocchioModel
from embodied.core.presets import DEFAULT_GRASP_READY_JOINTS
from embodied.core.project import load_config, resolve_path

MOTION = {'preset_speed_rad_s': .15, 'preset_acceleration_rad_s2': .3,
          'preset_jerk_rad_s3': 1., 'max_joint_acceleration_rad_s2': .3,
          'max_joint_jerk_rad_s3': 1., 'max_command_lead_rad': .05,
          'preview_time_s': .03, 'joint_soft_limit_margin_rad': .05}


def checked_state(arm):
    state = arm.snapshot()  # Existing client checks stale / nonfinite feedback.
    table = state.get('table_safety') or {}
    if table.get('blocked'):
        raise RuntimeError('桌面保护拒绝了运动，已停止本次操作')
    trajectory = state.get('trajectory')
    if trajectory:
        errors = np.abs(np.array(state['actual_pos']) - trajectory['position'])
        index = int(np.argmax(errors))
        if errors[index] > .15:
            raise RuntimeError(f'实际关节与服务规划偏差超过 0.15 rad：joint{index+1} '
                               f'实际={state["actual_pos"][index]:.4f}，规划={trajectory["position"][index]:.4f}，'
                               f'偏差={errors[index]:.4f} rad')
    return state


def stop_and_hold(arm):
    if arm.token is None:
        return
    arm.stop_control(release=True)  # Service finishes braking and retains position hold.
    deadline = time.monotonic() + 8
    while time.monotonic() < deadline:
        arm.snapshot()
        if arm.info['mode'] == 'hold' and arm.info['owner'] is None:
            return
        time.sleep(.02)
    state = arm.info['state']
    trajectory = state.get('trajectory') or {}
    error = np.max(np.abs(np.array(state['actual_pos']) - trajectory.get('position', state['actual_pos'])))
    raise RuntimeError(f'未确认服务停稳保持：mode={arm.info["mode"]}, owner={arm.info["owner"]}, '
                       f'规划完成={trajectory.get("finished")}, 最大位置偏差={error:.4f} rad；'
                       '请检查机械臂服务状态，当前不能确认已释放控制权')


def move_pregrasp(arm, *, timeout=40, open_gripper=False):
    if arm.info['owner'] is not None:
        raise RuntimeError(f"机械臂由 {arm.info['owner']} 占用，请先退出其控制会话")
    if not arm.info['safety'].get('table_enabled'):
        raise RuntimeError('预抓取入口要求服务启用桌面保护')
    target = arm.robot.joints(DEFAULT_GRASP_READY_JOINTS, margin=.05)
    initial = checked_state(arm)
    grip = min(float(initial['gripper_actual_pos']), arm.settings['gripper_open_limit_m'])
    grip_goal = min(.078, arm.settings['gripper_open_limit_m']) if open_gripper else grip
    print(f'前往统一预抓取位置：{target.tolist()}；Ctrl+C 停止并保持', flush=True)
    arm.start_control()
    try:
        started = time.monotonic()
        stable = None
        last_print = 0.
        while time.monotonic() - started < timeout:
            state = checked_state(arm)
            now = time.monotonic()
            error = float(np.max(np.abs(np.asarray(state['actual_pos']) - target)))
            speed = float(np.max(np.abs(state['actual_vel'])))
            if now - last_print >= 1:
                print(f'预抓取最大关节误差 {error:.4f} rad，速度 {speed:.3f} rad/s', flush=True)
                last_print = now
            planned = state.get('trajectory') or {}
            planned_at_goal = (planned.get('finished', False) and
                               np.max(np.abs(np.asarray(planned.get('position', [float('inf')]*6)) - target)) < .001)
            pose_ready = error < .03 and speed < .05 and planned_at_goal
            # Preserve opening during travel, then open gradually at the pregrasp pose.
            if open_gripper and pose_ready:
                grip += float(np.clip(grip_goal-grip, -.03*.02, .03*.02))
            arm.set_joint_positions(target, profile='preset')
            arm.set_catch_pos(grip)
            grip_ready = not open_gripper or (
                abs(grip-grip_goal) < 1e-6 and abs(state['gripper_actual_pos']-grip_goal) < .003)
            if pose_ready and grip_ready:
                stable = now if stable is None else stable
                if now - stable >= .5:
                    break
            else:
                stable = None
            time.sleep(.02)
        else:
            raise RuntimeError(f'{timeout:g} 秒内未到达预抓取位置或夹爪未就绪')
    finally:
        stop_and_hold(arm)
    state = checked_state(arm)
    if np.max(np.abs(np.asarray(state['actual_pos']) - target)) >= .03:
        raise RuntimeError('停稳后未保持在预抓取位置')
    if open_gripper and abs(state['gripper_actual_pos']-grip_goal) >= .003:
        raise RuntimeError('停稳后夹爪未保持张开')
    if open_gripper:
        print(f'夹爪已张开至 {state["gripper_actual_pos"]*1000:.1f} mm。', flush=True)
    print('已到达预抓取位置，服务保持姿态。', flush=True)
    return state


def capture(arm, output):
    import cv2
    from embodied.devices.sensors.client import SharedCamera
    config = load_config()['cameras']['wrist_rgb']
    with SharedCamera(config['serial'], config['width'], config['height'], config['fps']) as camera:
        before = checked_state(arm)
        frame = camera.read()
        after = checked_state(arm)
    if arm.info['owner'] is not None or arm.info['mode'] != 'hold':
        raise RuntimeError('拍摄时机械臂不处于无控制者的保持状态')
    target = np.asarray(DEFAULT_GRASP_READY_JOINTS)
    for state in (before, after):
        if np.max(np.abs(np.asarray(state['actual_pos']) - target)) >= .03:
            raise RuntimeError('实时观测必须从统一预抓取姿态开始，请使用 --move-pregrasp')
        if np.max(np.abs(state['actual_vel'])) >= .05:
            raise RuntimeError('拍摄时机械臂尚未停稳')
        if abs(frame['host_received_monotonic_ns'] - state['host_monotonic_ns']) > 100_000_000:
            raise RuntimeError('实时图像与关节状态时间差超过 100 ms')
    if not cv2.imwrite(str(output / 'wrist.png'), frame['image']):
        raise RuntimeError('无法保存相机图片')
    state = [*after['actual_pos'], after['gripper_actual_pos']]
    metadata = {'state_physical': state, 'robot': arm.robot.metadata(),
                'arm_service_id': arm.service_id, 'pregrasp': target.tolist(),
                'camera': camera.metadata,
                'rgb_monotonic_ns': frame['host_received_monotonic_ns'],
                'state_monotonic_ns': after['host_monotonic_ns']}
    (output / 'observation.json').write_text(json.dumps(metadata, indent=2) + '\n')
    return state


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--move-pregrasp', action='store_true', help='实际低速移动到预抓取位，完成后保持')
    parser.add_argument('--infer', action='store_true', help='拍摄预抓取位的实时观测并运行 PI05；预测不下发')
    parser.add_argument('--execute-step', action='store_true', help='按实验统计执行一个限幅关节动作，夹爪保持')
    parser.add_argument('--stats', type=Path, help='compute_stats.py 生成的 X5 分位数统计')
    parser.add_argument('--prompt', default='Pick up the blue cube and place it in the white target area beyond the cube, farther away from the robot base.')
    parser.add_argument('--output', type=Path)
    parser.add_argument('--service-socket')
    args = parser.parse_args()
    output = args.output or ROOT / 'outputs/zero-shot' / ('live-' + time.strftime('%Y%m%d-%H%M%S'))
    if args.execute_step and args.stats is None:
        parser.error('--execute-step requires --stats')
    if args.infer or args.execute_step:
        if not args.prompt.strip():
            parser.error('--prompt 不能为空')
        output.mkdir(parents=True, exist_ok=False)
    robot = X5PinocchioModel(resolve_path(load_config()['robot']['urdf']))
    arm = ArmClient(robot, path=args.service_socket, hardware=True, actuate=args.move_pregrasp,
                    input_name='zero-shot-pregrasp', speed=.15, motion=MOTION)
    try:
        s = arm.snapshot()
        print(json.dumps({'mode': arm.info['mode'], 'owner': arm.info['owner'],
                          'actual_pos': s['actual_pos'], 'gripper_m': s['gripper_actual_pos']}, ensure_ascii=False), flush=True)
        if args.move_pregrasp:
            move_pregrasp(arm)
        if args.execute_step or (args.infer and args.stats):
            from embodied.policy.pi05_zeroshot.experiment import run_trial
            run_trial(arm, output, args.stats, args.prompt, execute=args.execute_step)
            print(f'实验结果：{output.resolve()}', flush=True)
        elif args.infer:
            state = capture(arm, output)
            # Separate environments: hardware client never imports PyTorch or policy outputs.
            subprocess.run(['bash', str(ROOT / 'src/embodied/policy/pi05_zeroshot/run.sh'),
                '--image', str((output / 'wrist.png').resolve()), '--state', *map(str, state),
                '--prompt', args.prompt, '--gripper-width', str(arm.settings['gripper_width']),
                '--output', str((output / 'prediction').resolve())], check=True)
            print(f'现场观测与预测：{output.resolve()}；预测未下发机械臂。', flush=True)
    finally:
        arm.close()  # After stop(release=True), this does not release service-owned hold.


if __name__ == '__main__':
    def interrupt(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    try:
        main()
    except KeyboardInterrupt:
        print('已中断。若此前正在运动，已请求服务停止并保持。', file=sys.stderr)
        sys.exit(130)
