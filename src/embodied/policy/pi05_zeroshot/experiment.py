"""One bounded experimental policy step; no action queue or autonomous rollout."""
import hashlib
import json
import os
from pathlib import Path
import queue
import subprocess
import threading
import time

import numpy as np
from embodied.policy.pi05_zeroshot.real import ROOT, capture, checked_state, stop_and_hold
from embodied.core.model import RobotModel
from embodied.policy.pi05_zeroshot.worker_client import source_environment
from embodied.policy.pi05_zeroshot.x5_mapping import X5Mapping

MAX_STEP_RAD = .02
MAX_TCP_STEP_M = .01
MAX_OBSERVATION_AGE_S = 2.


def require_fresh(arm, prediction, state):
    if prediction['arm_service_id'] != arm.service_id:
        raise ValueError('预测来自另一机械臂服务实例')
    age = (time.monotonic_ns() - prediction['rgb_monotonic_ns']) / 1e9
    if not 0 <= age <= MAX_OBSERVATION_AGE_S:
        raise ValueError(f'观测已过期（{age:.3f}s），拒绝执行；需重新拍摄推理')
    observed = np.asarray(prediction['state_physical'], dtype=float)
    if observed.shape != (7,) or not np.isfinite(observed).all():
        raise ValueError('预测缺少有效观测状态')
    if np.max(np.abs(np.array(state['actual_pos']) - observed[:6])) > .01:
        raise ValueError('预测后机械臂位置已变化，拒绝执行')
    if np.max(np.abs(state['actual_vel'])) >= .05:
        raise ValueError('机械臂未停稳')


def step_target(robot, tcp, state, prediction):
    desired = np.asarray(prediction['first_action_x5_experimental'], dtype=float)
    observed = np.asarray(prediction['state_physical'], dtype=float)
    if desired.shape != (7,) or observed.shape != (7,) or not np.isfinite([desired, observed]).all():
        raise ValueError('Invalid decoded action')
    delta = desired[:6] - observed[:6]
    peak = float(np.max(np.abs(delta)))
    if peak > .5:
        raise ValueError('原始预测单步关节增量超过 0.5 rad，拒绝执行')
    scale = min(1., MAX_STEP_RAD / max(peak, 1e-12))
    q = np.asarray(state['actual_pos'])
    robot.joints(q, margin=.05)
    # Shorten the entire vector near a soft joint limit, preserving direction.
    for joint, change in enumerate(delta):
        if change > 1e-12:
            scale = min(scale, max(0., (robot.upper[joint] - .05 - q[joint]) / change))
        elif change < -1e-12:
            scale = min(scale, max(0., (robot.lower[joint] + .05 - q[joint]) / change))
    point = tcp.fk(q)[:3, 3]
    for _ in range(12):
        target = robot.joints(q + delta * scale, margin=.05)
        displacement = float(np.linalg.norm(tcp.fk(target)[:3, 3] - point))
        if displacement <= MAX_TCP_STEP_M:
            return target, {'raw_delta_rad': delta.tolist(), 'scale': scale,
                            'commanded_delta_rad': (target - q).tolist(),
                            'predicted_tcp_displacement_m': displacement,
                            'predicted_gripper_m_not_executed': float(desired[6])}
        scale *= .5
    raise ValueError('Cannot bound Cartesian displacement')


def execute_step(arm, prediction, output):
    state = checked_state(arm)
    if arm.info['owner'] is not None or arm.info['mode'] != 'hold':
        raise RuntimeError('策略试探要求无人占用的预抓取保持状态')
    if not arm.info['safety'].get('table_enabled'):
        raise RuntimeError('要求启用桌面保护')
    require_fresh(arm, prediction, state)
    excess = prediction.get('state_quantile_excess', float('inf'))
    if not np.isfinite(excess) or not 0 <= excess <= .1:
        raise ValueError('当前状态超出统计分位范围过多，先检查统计覆盖再执行')
    tcp = RobotModel(arm.robot.urdf, 'eef_link')
    target, plan = step_target(arm.robot, tcp, state, prediction)
    result = {'status': 'planned', 'plan': plan, 'target_rad': target.tolist(),
              'start_rad': state['actual_pos'], 'gripper_held_m': float(state['gripper_actual_pos']),
              'limits': {'joint_step_rad': MAX_STEP_RAD, 'tcp_step_m': MAX_TCP_STEP_M,
                         'speed_rad_s': .05, 'observation_age_s': MAX_OBSERVATION_AGE_S}}
    result_path = output / 'execution.json'
    result_path.write_text(json.dumps(result, indent=2) + '\n')
    print('PI05 单步试探计划：' + json.dumps(plan, ensure_ascii=False), flush=True)
    arm.speed, arm.actuate = .05, True
    arm.motion.update(preset_speed_rad_s=.05, preset_acceleration_rad_s2=.15,
                      preset_jerk_rad_s3=.5, max_joint_acceleration_rad_s2=.15,
                      max_joint_jerk_rad_s3=.5, max_command_lead_rad=.02)
    grip = min(float(state['gripper_actual_pos']), arm.settings['gripper_open_limit_m'])
    with (output / 'execution_trace.jsonl').open('x') as trace:
        try:
            arm.start_control()
            require_fresh(arm, prediction, checked_state(arm))
            began, stable = time.monotonic(), None
            while time.monotonic() - began < 8:
                actual = checked_state(arm)
                if np.max(np.abs(np.array(actual['actual_pos']) - state['actual_pos'])) > .05:
                    raise RuntimeError('实际位移超过单步试探的 0.05 rad 监控边界')
                arm.set_joint_positions(target, profile='preset')
                arm.set_catch_pos(grip)
                trace.write(json.dumps({'host_monotonic_ns': actual['host_monotonic_ns'],
                    'actual_pos': actual['actual_pos'], 'actual_vel': actual['actual_vel'],
                    'target': target.tolist()}) + '\n')
                trace.flush()
                err = np.max(np.abs(np.array(actual['actual_pos']) - target))
                if err < .015 and np.max(np.abs(actual['actual_vel'])) < .05:
                    stable = time.monotonic() if stable is None else stable
                    if time.monotonic() - stable >= .5 and time.monotonic() - began >= 1.:
                        result['status'] = 'completed'
                        break
                else:
                    stable = None
                time.sleep(.02)
            else:
                raise RuntimeError('试探动作未按时停稳')
        except BaseException as exc:
            result.update(status='failed', error=str(exc) or type(exc).__name__)
            raise
        finally:
            try:
                stop_and_hold(arm)
                final = arm.snapshot()
                result['final_rad'] = final['actual_pos']
                result['actual_delta_rad'] = (np.array(final['actual_pos']) - state['actual_pos']).tolist()
                result['final_mode'] = arm.info['mode']
            except BaseException as exc:
                result.update(status='stop_unconfirmed', stop_error=str(exc))
                raise
            finally:
                result_path.write_text(json.dumps(result, indent=2) + '\n')
    print('单次策略试探结束，机械臂保持；没有执行夹爪闭合。', flush=True)


def run_trial(arm, output, stats, prompt, *, execute=True):
    mapping = X5Mapping(stats)
    if mapping.data['robot'] != arm.robot.metadata():
        raise ValueError('统计与当前 X5 URDF/单位/关节顺序不匹配')
    digest = hashlib.sha256(stats.read_bytes()).hexdigest()
    python = os.environ.get('PI05_PYTHON', str(ROOT / 'outputs/envs/pi05-zero-shot/bin/python'))
    process = subprocess.Popen([python, '-m', 'embodied.policy.pi05_zeroshot.policy_worker', '--stats', str(stats.resolve())],
                               stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                               text=True, bufsize=1, env=source_environment())
    messages = queue.Queue()
    def read():
        try:
            for line in process.stdout:
                if line.startswith('@pi05 '):
                    messages.put(json.loads(line[len('@pi05 '):]))
                else:
                    print(line, end='', flush=True)
        finally:
            messages.put({'error': '推理进程已退出'})
    reader = threading.Thread(target=read, daemon=True)
    reader.start()
    def receive(timeout):
        value = messages.get(timeout=timeout)
        if 'error' in value:
            raise RuntimeError(value['error'])
        return value
    try:
        ready = receive(180)
        if not ready.get('ready') or ready['stats_sha256'] != digest:
            raise ValueError('推理进程统计不一致')
        print('模型就绪，现在拍摄新观测。', flush=True)
        capture(arm, output)
        process.stdin.write(json.dumps({'output': str(output.resolve()), 'prompt': prompt}) + '\n')
        process.stdin.flush()
        prediction = receive(15)['prediction']
        if prediction['stats_sha256'] != digest:
            raise ValueError('预测统计不一致')
        if execute:
            execute_step(arm, prediction, output)
        else:
            print('已保存按 X5 统计解码的预测，未执行。', flush=True)
    finally:
        process.stdin.close()
        if process.poll() is None:
            process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)
        reader.join(timeout=2)
        process.stdout.close()
