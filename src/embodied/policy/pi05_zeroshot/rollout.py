"""Continuous experimental PI05/X5 control, with fresh replanning and gripper commands."""
import argparse
from contextlib import contextmanager
import json
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

ROOT = Path(__file__).resolve().parents[4]
import numpy as np
from embodied.policy.pi05_zeroshot.real import MOTION, move_pregrasp, checked_state, stop_and_hold
from embodied.core.project import load_config, resolve_path
from embodied.devices.robots.arx5.pinocchio_model import X5PinocchioModel
from embodied.devices.sensors.client import SharedCamera
from legacy.robot_control.client import ArmClient
from embodied.policy.pi05_zeroshot.worker_client import PolicyWorker
from embodied.policy.pi05_zeroshot.x5_mapping import X5Mapping

DEFAULT_PROMPT = 'Pick up the blue cube and place it in the white target area beyond the cube, farther away from the robot base.'


@contextmanager
def heartbeat(arm):
    stop = threading.Event()
    errors = []
    def loop():
        while not stop.wait(.05):
            try:
                checked_state(arm)
            except BaseException as exc:
                errors.append(exc)
                return
    def check():
        if errors:
            raise RuntimeError(f'机械臂反馈异常：{errors[0]}')
    worker = threading.Thread(target=loop, daemon=True)
    worker.start()
    try:
        yield check
    finally:
        stop.set()
        worker.join(timeout=1)


def capture_owned(arm, camera, output):
    import cv2
    before = checked_state(arm)
    frame = camera.read()
    after = checked_state(arm)
    stamp = frame['host_received_monotonic_ns']
    # Use the closest bracketing sample and record its offset; no pregrasp assumption while moving.
    state = min((before, after), key=lambda s: abs(s['host_monotonic_ns']-stamp))
    skew = (state['host_monotonic_ns'] - stamp) / 1e9
    if abs(skew) > .06 or not 0 <= (time.monotonic_ns()-stamp)/1e9 <= .2:
        raise ValueError('相机/机器人时间差过大或图像过期')
    if not cv2.imwrite(str(output/'wrist.png'), frame['image']):
        raise RuntimeError('Cannot save live image')
    obs = {'robot': arm.robot.metadata(), 'camera': camera.metadata,
           'arm_service_id': arm.service_id, 'rgb_monotonic_ns': stamp,
           'state_monotonic_ns': state['host_monotonic_ns'], 'state_rgb_skew_s': skew,
           'state_physical': [*state['actual_pos'], state['gripper_actual_pos']]}
    (output/'observation.json').write_text(json.dumps(obs, indent=2)+'\n')
    return obs


def bounded_target(robot, desired, state, previous_grip, grip_limit, grip_speed, dt):
    desired = np.asarray(desired, dtype=float)
    if desired.shape != (7,) or not np.isfinite(desired).all():
        raise ValueError('模型返回无效 X5 动作')
    q = np.asarray(state['actual_pos'])
    # Service Ruckig determines physical speed. Keep the requested target close to feedback.
    lower, upper = robot.lower + .05, robot.upper - .05
    clipped = np.clip(desired[:6], lower, upper)
    target = robot.joints(np.clip(clipped, q - .08, q + .08))
    grip_goal = float(np.clip(desired[6], 0, grip_limit))
    grip = float(previous_grip + np.clip(grip_goal-previous_grip, -grip_speed*dt, grip_speed*dt))
    return target, grip, np.flatnonzero(np.abs(clipped-desired[:6]) > 1e-6).tolist()


def check_state_quantile(excess, limit):
    if not np.isfinite(limit) or limit <= 0:
        raise ValueError('state-quantile-limit 必须为有限正数')
    if not np.isfinite(excess) or excess < 0 or excess > limit:
        raise ValueError(f'当前状态超出 X5 数据统计阈值：excess={excess:.4f}, limit={limit:g}；停止本轮（详见 prediction.json）')


def control_loop(arm, worker, camera, output, prompt, *, seconds, chunk_steps, grip_speed,
                 state_quantile_limit=.5):
    summary = {'status': 'running', 'task_success': None, 'prompt': prompt,
               'seconds_limit': seconds, 'chunk_steps': chunk_steps,
               'policy_fps': 30, 'cycles': 0, 'commands': 0, 'stats_sha256': worker.digest,
               'state_quantile_limit': state_quantile_limit}
    started = time.monotonic()
    deadline = started + seconds
    grip = min(float(checked_state(arm)['gripper_actual_pos']), arm.settings['gripper_open_limit_m'])
    def save():
        (output/'rollout.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2)+'\n')
    save()
    with (output/'commands.jsonl').open('x') as log, heartbeat(arm) as health:
        def check():
            health()
            if time.monotonic() >= deadline:
                raise TimeoutError('本轮运行时间结束')
        try:
            while time.monotonic() < deadline:
                check()
                cycle = output/f'cycle_{summary["cycles"]:04d}'
                cycle.mkdir()
                # Brake while waiting for a new observation/prediction; no stale chunk replay.
                arm.stop_control()
                obs = capture_owned(arm, camera, cycle)
                prediction = worker.predict(cycle, prompt, check=check)
                check()
                if prediction['arm_service_id'] != arm.service_id:
                    raise ValueError('Policy observation belongs to another service')
                if prediction['rgb_monotonic_ns'] != obs['rgb_monotonic_ns']:
                    raise ValueError('Policy observation timestamp mismatch')
                excess = prediction.get('state_quantile_excess', float('inf'))
                summary['last_state_quantile_excess'] = float(excess) if np.isfinite(excess) else None
                check_state_quantile(excess, state_quantile_limit)
                with np.load(cycle/'prediction.npz', allow_pickle=False) as data:
                    key = 'actions_x5' if 'actions_x5' in data.files else 'actions_x5_experimental'
                    actions = data[key].copy()
                if actions.ndim != 2 or actions.shape[1] != 7 or len(actions) < chunk_steps or not np.isfinite(actions).all():
                    raise ValueError('Invalid action chunk')
                summary['cycles'] += 1
                chunk_start = time.monotonic()
                for index, action in enumerate(actions[:chunk_steps]):
                    check()
                    if (time.monotonic_ns()-obs['rgb_monotonic_ns'])/1e9 > 1.5:
                        raise ValueError('动作块的观测已过期')
                    state = checked_state(arm)
                    if np.max(np.abs(np.array(state['actual_pos'])-obs['state_physical'][:6])) > .2:
                        raise ValueError('机械臂偏离当前动作块观测过多')
                    target, grip, clipped = bounded_target(arm.robot, action, state, grip,
                        arm.settings['gripper_open_limit_m'], grip_speed, 1/30)
                    arm.set_joint_positions(target, profile='teleop')
                    arm.set_catch_pos(grip)
                    summary['commands'] += 1
                    log.write(json.dumps({'host_monotonic_ns': time.monotonic_ns(), 'cycle': summary['cycles']-1,
                        'action_index': index, 'raw_action': action.tolist(), 'target_rad': target.tolist(),
                        'gripper_m': grip, 'actual_pos': state['actual_pos'], 'clipped_joints': clipped})+'\n')
                    log.flush()
                    remaining = chunk_start+(index+1)/30-time.monotonic()
                    if remaining > 0:
                        time.sleep(remaining)
                print(f'PI05 循环 {summary["cycles"]} | 已发 {summary["commands"]} 步 | '
                      f'推理 {prediction["inference_seconds"]:.3f}s | 夹爪目标 {grip:.3f}m | '
                      f'统计超出 {excess:.3f}/{state_quantile_limit:g} | '
                      f'已运行 {time.monotonic()-started:.1f}s', flush=True)
                save()
            summary['status'] = 'time_limit'
        except TimeoutError as exc:
            summary.update(status='time_limit' if time.monotonic() >= deadline else 'failed', reason=str(exc))
            if summary['status'] == 'failed':
                raise
        except BaseException as exc:
            summary.update(status='interrupted' if isinstance(exc, KeyboardInterrupt) else 'failed',
                           reason=str(exc) or type(exc).__name__)
            raise
        finally:
            try:
                stop_and_hold(arm)
                state = arm.snapshot()
                summary.update(final_mode=arm.info['mode'], final_rad=state['actual_pos'],
                               final_gripper_m=state['gripper_actual_pos'])
            except BaseException as exc:
                summary.update(status='stop_unconfirmed', stop_error=str(exc))
                raise
            finally:
                summary['elapsed_seconds'] = time.monotonic()-started
                save()
    return summary


def connect(robot, output):
    try:
        return ArmClient(robot, hardware=True, actuate=True, speed=.15, motion=MOTION, input_name='pi05-rollout')
    except RuntimeError:
        # Only auto-start an absent service, never replace a live/faulty one or another owner.
        if resolve_path(load_config()['arm_service']['socket']).exists():
            raise
    with (output/'arm-service.log').open('ab', buffering=0) as log:
        subprocess.Popen(['bash', str(ROOT/'scripts/shortcuts/启动机械臂服务.sh')], stdin=subprocess.DEVNULL,
                         stdout=log, stderr=log, start_new_session=True)
    until = time.monotonic()+15
    while time.monotonic() < until:
        try:
            return ArmClient(robot, hardware=True, actuate=True, speed=.15, motion=MOTION, input_name='pi05-rollout')
        except RuntimeError:
            time.sleep(.2)
    raise RuntimeError(f'机械臂服务未启动，请查看 {output / "arm-service.log"}')


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--prompt', default=DEFAULT_PROMPT)
    p.add_argument('--seconds', type=float, default=60, help='模型就绪后的连续运行时长，秒')
    p.add_argument('--chunk-steps', type=int, default=10, help='每次执行动作块前几步，再拍摄重规划（1..15）')
    p.add_argument('--speed', type=float, default=.3, help='关节速度上限 rad/s（最大 0.5）')
    p.add_argument('--gripper-speed', type=float, default=.03, help='夹爪目标变化速度 m/s（最大 0.05）')
    p.add_argument('--stats', type=Path, default=ROOT/'outputs/zero-shot-assets/x5_quantiles_h50.json')
    p.add_argument('--output', type=Path)
    args = p.parse_args()
    if (not args.prompt.strip() or not np.isfinite([args.seconds,args.speed,args.gripper_speed]).all()
        or not 0 < args.seconds <= 300 or not 1 <= args.chunk_steps <= 15
        or not 0 < args.speed <= .5 or not 0 < args.gripper_speed <= .05):
        p.error('参数越界：seconds 0..300, chunk-steps 1..15, speed 0..0.5, gripper-speed 0..0.05')
    if not args.stats.exists():
        subprocess.run([sys.executable, '-m', 'embodied.policy.pi05_zeroshot.compute_stats', '--output', str(args.stats)], check=True)
    robot = X5PinocchioModel(resolve_path(load_config()['robot']['urdf']))
    mapping = X5Mapping(args.stats)
    if mapping.data['robot'] != robot.metadata():
        raise ValueError('X5 统计与当前 URDF 不匹配')
    output = args.output or ROOT/'outputs/zero-shot'/('rollout-'+time.strftime('%Y%m%d-%H%M%S'))
    output.mkdir(parents=True, exist_ok=False)
    (output/'stats.json').write_bytes(args.stats.read_bytes())
    print(f'任务：{args.prompt}\n连续运行 {args.seconds:g} 秒；夹爪由 PI05 控制；Ctrl+C 停止并保持。\n输出：{output}', flush=True)
    worker = PolicyWorker(output/'stats.json')  # Load once before acquiring or moving hardware.
    arm = None
    try:
        arm = connect(robot, output)
        move_pregrasp(arm)
        arm.speed = args.speed
        arm.motion.update(max_joint_acceleration_rad_s2=.5, max_joint_jerk_rad_s3=2.,
                          max_command_lead_rad=.08)
        c = load_config()['cameras']['wrist_rgb']
        with SharedCamera(c['serial'], c['width'], c['height'], c['fps']) as camera:
            arm.start_control()
            try:
                result = control_loop(arm, worker, camera, output, args.prompt, seconds=args.seconds,
                                      chunk_steps=args.chunk_steps, grip_speed=args.gripper_speed)
                print(f'本轮结束：{result["status"]}，{result["commands"]} 步；机械臂保持。', flush=True)
            finally:
                stop_and_hold(arm)
    finally:
        if arm is not None:
            arm.close()
        worker.close()


if __name__ == '__main__':
    def interrupt(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    try:
        main()
    except KeyboardInterrupt:
        print('已中断本轮。', flush=True)
        sys.exit(130)
