"""Model-independent stop/hold and home recovery for this checkout's PI05 processes."""
import argparse
import os
from pathlib import Path
import signal
import sys
import time

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'src'))
import numpy as np
from robot_core.model import RobotModel
from robot_core.presets import DEFAULT_HOME_JOINTS
from robot_core.collection_motion import collection_motion
from robot_core.project import load_config, resolve_path
from robot_control.client import ArmClient
from real import checked_state, stop_and_hold, move_pregrasp

POLICY_SCRIPTS = {str(ROOT/'src/zero-shot'/name) for name in ('real.py','rollout.py','policy_worker.py','infer.py')}
POLICY_SCRIPTS |= {str(ROOT/'src/infer_after_sft'/name) for name in ('run.py','worker.py')}
HOME_SPEED, HOME_MOTION = collection_motion()


def policy_processes():
    found = {}
    for proc in Path('/proc').iterdir():
        if not proc.name.isdigit() or int(proc.name) == os.getpid():
            continue
        try:
            if proc.stat().st_uid != os.getuid():
                continue
            argv = (proc/'cmdline').read_bytes().split(b'\0')
            if not argv or not Path(os.fsdecode(argv[0])).name.startswith('python'):
                continue
            cwd = (proc/'cwd').resolve()
            # Only a Python script entrypoint; never match a filename passed to -c/-m or as data.
            for arg in argv[1:]:
                value = os.fsdecode(arg)
                if value in ('-B', '-u', '-I', '-E', '-s', '-S', '-O', '-OO'):
                    continue
                if value and not value.startswith('-'):
                    script = str((cwd/value).resolve())
                    if script in POLICY_SCRIPTS:
                        found[int(proc.name)] = script
                break
        except (FileNotFoundError, ProcessLookupError, PermissionError, OSError):
            continue
    return found


def stop_policy_processes():
    found = policy_processes()
    # Controllers must get their finally/stop-and-hold before the GPU workers exit.
    controllers = {pid for pid, path in found.items() if Path(path).name in ('real.py','rollout.py','run.py')}
    for pid in controllers:
        try:
            os.kill(pid, signal.SIGTERM)
            print(f'正在停止策略控制进程 {pid}…', flush=True)
        except ProcessLookupError:
            pass
    deadline = time.monotonic()+8
    while controllers & policy_processes().keys() and time.monotonic() < deadline:
        time.sleep(.1)
    remaining = policy_processes()
    for pid in remaining:
        try:
            os.kill(pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    deadline = time.monotonic()+3
    while policy_processes() and time.monotonic() < deadline:
        time.sleep(.1)
    for pid in policy_processes():
        try:
            print(f'进程 {pid} 未响应，结束该策略进程。', flush=True)
            os.kill(pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
    deadline = time.monotonic()+2
    while policy_processes() and time.monotonic() < deadline:
        time.sleep(.1)
    if policy_processes():
        raise RuntimeError('策略进程未退出，未执行回零；请检查系统进程状态')


def wait_available(arm):
    deadline = time.monotonic()+10
    stationary_since = None
    while time.monotonic() < deadline:
        state = arm.snapshot()
        owner = arm.info['owner']
        if owner and owner not in ('pi05-rollout','zero-shot-pregrasp','pi05-sft'):
            raise RuntimeError(f'机械臂仍由 {owner} 控制；请先退出该控制会话')
        if (owner and arm.info['mode'] == 'stopping'
                and arm.info.get('stopped_hold_recovery_protocol') != 1):
            raise RuntimeError('当前旧机械臂服务卡在 stopping，未加载停止恢复接口。'
                               '重跑停止.sh 无法恢复；请先支撑机械臂，在原机械臂服务终端 Ctrl+C 退出，'
                               '再运行 bash 启动机械臂服务.sh。只重启模型程序不会更新服务。')
        if owner is None and np.max(np.abs(state['actual_vel'])) < .05:
            return state
        if (owner and arm.info['mode'] == 'stopping'
                and (state.get('trajectory') or {}).get('finished')
                and np.max(np.abs(state['actual_vel'])) < .03):
            stationary_since = time.monotonic() if stationary_since is None else stationary_since
            if time.monotonic()-stationary_since >= .7:
                try:
                    result = arm.recover_stopped_hold(owner)
                except RuntimeError as exc:
                    raise RuntimeError('停止恢复未完成：' + str(exc) +
                        '。若为旧服务不支持接口，需先支撑机械臂并退出原服务，再启动 启动机械臂服务.sh；'
                        '当前不能确认已保持或释放控制权') from exc
                print(f'已将停止目标恢复为实际停稳位置，原跟踪偏差 '
                      f'{result["previous_tracking_error_rad"]:.4f} rad；服务保持。',flush=True)
                continue
        else:
            stationary_since = None
        time.sleep(.05)
    raise RuntimeError(f'停止恢复未完成：mode={arm.info["mode"]}, owner={owner}, '
                       f'最大反馈速度={np.max(np.abs(state["actual_vel"])):.4f} rad/s, '
                       f'规划完成={(state.get("trajectory") or {}).get("finished")}；'
                       '尚未满足连续停稳及控制权释放条件')


def hold(arm):
    state = wait_available(arm)
    if arm.info['mode'] == 'hold':
        return state
    # Reestablish hold after a killed controller's lease expires into damping.
    arm.start_control()
    try:
        arm.snapshot()
    finally:
        stop_and_hold(arm)
    return arm.snapshot()


def home(arm, timeout=60):
    state = wait_available(arm)
    target = arm.robot.joints(DEFAULT_HOME_JOINTS, margin=.05)
    grip = min(float(state['gripper_actual_pos']), arm.settings['gripper_open_limit_m'])
    print('回零目标：[0, 0, 0, 0, 0, 0] rad；保持当前夹爪开度；Ctrl+C 停止。', flush=True)
    arm.start_control()
    try:
        deadline, stable, last_print = time.monotonic()+timeout, None, 0.
        while time.monotonic() < deadline:
            state = checked_state(arm)
            arm.set_joint_positions(target, profile='home')
            arm.set_catch_pos(grip)
            now = time.monotonic()
            err = float(np.max(np.abs(np.asarray(state['actual_pos'])-target)))
            if now-last_print >= 1:
                print(f'回零中：最大关节误差 {err:.4f} rad', flush=True)
                last_print = now
            plan = state.get('trajectory') or {}
            reached = (plan.get('finished',False) and
                       np.max(np.abs(np.asarray(plan.get('position',[float('inf')]*6))-target)) < .001)
            if reached and err < .03 and np.max(np.abs(state['actual_vel'])) < .05:
                stable = now if stable is None else stable
                if now-stable >= .5:
                    break
            else:
                stable = None
            time.sleep(.02)
        else:
            raise RuntimeError('回零超时，已请求停稳保持')
    finally:
        stop_and_hold(arm)
    state = arm.snapshot()
    if np.max(np.abs(np.asarray(state['actual_pos'])-target)) >= .03:
        raise RuntimeError('停稳后回零误差超限')
    print('已回零，机械臂保持零位。', flush=True)
    return state


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    movement = parser.add_mutually_exclusive_group()
    movement.add_argument('--home', action='store_true', help='结束策略进程后按数据采集参数回零')
    movement.add_argument('--pregrasp', action='store_true', help='结束策略进程后按数据采集参数到预抓取位并张开夹爪')
    args = parser.parse_args()
    stop_policy_processes()
    robot = RobotModel(resolve_path(load_config()['robot']['urdf']))
    # Do not reset/restart a faulty service in recovery; preserve its device ownership.
    arm = ArmClient(robot, hardware=True, actuate=True, input_name='terminal-recovery',
                    speed=HOME_SPEED, motion=HOME_MOTION)
    try:
        hold(arm)
        if args.home:
            home(arm)
        elif args.pregrasp:
            move_pregrasp(arm, open_gripper=True)
        else:
            print('策略进程已停止，机械臂保持当前位置。', flush=True)
    finally:
        arm.close()


if __name__ == '__main__':
    def interrupt(signum, frame):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupt)
    try:
        main()
    except KeyboardInterrupt:
        print('操作已中断。', flush=True)
        sys.exit(130)
