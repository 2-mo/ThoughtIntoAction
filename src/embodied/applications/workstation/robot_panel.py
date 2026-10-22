"""Two-key robot console using the existing leased Robot API.

P/H submit motion; space cancels this panel's operation. D then D explicitly
requests real kernel shutdown into SDK damping; opening the panel is read-only.
"""
import math
import time

from embodied.applications.operations.robot import Robot
from embodied.applications.workstation.status import Status
from embodied.contracts.runtime.control import ControlClient
from embodied.core.presets import DEFAULT_GRASP_READY_JOINTS
from embodied.control.homing import ReleaseThenHome


class RobotControls:
    def __init__(self):
        self.robot = None
        self.motion = None
        self.run = None
        self.target = None
        self.started = None
        self.message = Status('按 P 到预抓取位，按 H 回零位', 'ready')
        self.plan_message = None
        self.description = None
        self.description_run = None
        self.damping_armed_until = 0.
        self.damping_run = None

    def close(self):
        robot, self.robot = self.robot, None
        motion, self.motion = self.motion, None
        if robot is not None:
            try:
                if motion is not None:
                    robot.cancel()
            finally:
                robot.close()

    def press(self, key, found):
        if key not in ('p', 'h', ' ', 's', 'd'):
            self.damping_armed_until = 0.
            return
        try:
            if key == 'd':
                from embodied.applications.workstation import bench
                if found is None or found[1].get('backend') != 'real' or bench.owned_run() != found:
                    raise RuntimeError('D 仅用于本工作台正在运行的真机内核')
                if time.monotonic() >= self.damping_armed_until or self.damping_run != found[0]:
                    self.damping_run = found[0]
                    self.damping_armed_until = time.monotonic() + 5
                    self.message = Status('请扶住机械臂：5 秒内再按 D，退出内核进入阻尼；其他键取消', 'waiting')
                    return
                self.damping_armed_until = 0.
                self.close()
                bench.stop_kernel()
                self.message = Status('已请求退出进入阻尼，请扶住机械臂；恢复控制需在总控重新启动 robot', 'waiting')
                return
            self.damping_armed_until = 0.
            if key in (' ', 's'):
                if self.robot is None:
                    self.message = Status('本面板没有正在执行的动作', 'ready')
                    return
                self.close()
                self.message = Status('动作已取消，已请求原位保持', 'waiting')
                return
            if self.motion is not None:
                self.message = Status('动作执行中；空格停止后可重新选择，不排队重复动作', 'waiting')
                return
            if found is None or not found[1].get('managed'):
                raise RuntimeError('robot 尚未就绪；请先在总控启动')
            run, identity = found
            if key == 'p' and (identity.get('plugins') or {}).get('robot') not in ('arx.real', 'arx.sim'):
                raise ValueError('当前机器人未配置预抓取位姿')
            # The connection is lazy and explicit: opening the page owns no lease.
            self.run = run
            self.robot = Robot.connect(run).__enter__()
            state = self.robot.state
            if state.get('owner') or state.get('stopped'):
                raise RuntimeError('robot 已被其他控制源占用或停止；未发送动作')
            self.target = (list(DEFAULT_GRASP_READY_JOINTS) if key == 'p'
                           else self.robot.describe()['robot']['park_q'])
            self.started = time.monotonic()
            self.motion = (self.robot.home(release_gripper=True, wait=False) if key == 'h'
                           else self.robot.movej(self.target, wait=False))
            name = '预抓取' if key == 'p' else '回零'
            self.plan_message = None
            self.describe_plan(name)
            self.message = Status('先张开夹爪，确认松开后回零；空格停止' if key == 'h'
                                  else f'正在{name}；空格停止并保持', 'waiting')
        except (OSError, ValueError, RuntimeError, EOFError) as exc:
            self.close()
            self.message = Status(f'动作未完成：{exc}', 'error')

    def describe_plan(self, name):
        metrics = self.motion.metrics
        if metrics:
            self.plan_message = (f'{name} · 轨迹 {metrics["duration_s"]:.2f} s · '
                                 f'规划峰值 {metrics["peak_velocity_rad_s"]:.2f} rad/s')
            factors = metrics.get('limiting_factors', [])
            labels = {'speed': '速度', 'acceleration': '加速度', 'jerk': 'jerk'}
            if factors:
                self.plan_message += ' · 受限于 ' + '、'.join(labels.get(v, v) for v in factors)

    def rows(self, found):
        """Read feedback and preserve the kernel's actual completion/failure result."""
        rows = []
        had_motion = self.motion is not None
        try:
            if self.damping_armed_until and time.monotonic() >= self.damping_armed_until:
                self.damping_armed_until = 0.
                self.message = Status('阻尼确认已过期，未发送退出请求', 'ready')
            if self.robot is not None and (found is None or found[0] != self.run):
                self.close()
                raise RuntimeError('运行实例已变化，当前动作已结束')
            if found is not None and found[1].get('managed'):
                client = ControlClient(found[0])
                if self.description_run != found[0]:
                    self.description = client.describe().get('motion_limits')
                    self.description_run = found[0]
                state = client.status()
                if self.motion is not None:
                    if isinstance(self.motion, ReleaseThenHome):
                        self.motion.step(state)
                        self.describe_plan('回零')
                        if self.motion.operation_id:
                            self.message = Status('夹爪已张开，正在回零；空格停止', 'waiting')
                    operation = next((op for op in state['operations']
                                      if op['operation_id'] == self.motion.operation_id), None)
                    if operation is not None and operation['status'] != 'running':
                        self.motion = None
                        self.close()
                        elapsed = time.monotonic() - self.started
                        if operation['status'] == 'completed':
                            detail = operation.get('settling')
                            accuracy = '' if not detail else (
                                f' · J{detail["worst_joint"]} 最大偏差 '
                                f'{math.degrees(detail["max_joint_error_rad"]):.2f}°'
                                f' / 容差 {math.degrees(detail["position_tolerance_rad"]):.2f}°')
                            self.message = Status(f'已到位并保持{accuracy} · {elapsed:.2f} s', 'ready')
                        else:
                            reason = operation.get('reason') or operation['status']
                            if reason == 'Robot did not settle':
                                reason = self.settling_reason(operation, state)
                            self.message = Status(f'动作未通过：{reason} · {elapsed:.2f} s', 'error')
                rows.append(('速度 rad/s', '  '.join(f'{v:+.3f}' for v in state['dq'])))
                if self.target is not None:
                    errors = [math.degrees(a-b) for a, b in zip(state['q'], self.target)]
                    rows.append(('目标误差 °', '  '.join(f'{v:+.2f}' for v in errors)))
                if self.plan_message:
                    rows.append(('最近轨迹', self.plan_message))
                elif state.get('operations'):
                    last = state['operations'][-1]
                    if last['status'] == 'failed':
                        reason = last.get('reason') or '未知原因'
                        if reason == 'Robot did not settle':
                            reason = '未满足到位条件'
                        rows.append(('上次动作', Status(reason, 'error')))
        except (OSError, ValueError, RuntimeError, EOFError) as exc:
            try:
                self.close()
            except (OSError, ValueError, RuntimeError, EOFError) as cleanup:
                exc = RuntimeError(f'{exc}；结束控制未确认：{cleanup}')
            # A poll failure belongs to this refresh, not the persistent action
            # result. A missing socket during restart must disappear on recovery.
            rows.append(('反馈查询', Status(f'反馈暂不可用：{exc}', 'error')))
            if had_motion:
                self.message = Status(f'动作跟踪已中断；未确认到位：{exc}', 'error')
        if found is not None and self.description and self.description_run == found[0]:
            settings = self.description['defaults']
            speed = '后端每轴上限' if settings['speed'] == 0 else f'{settings["speed"]:g} rad/s'
            rows.append(('生效参数', f'速度 {speed} · 加速度 {settings["acceleration"]:g} rad/s² · jerk {settings["jerk"]:g} rad/s³'))
            settling = self.description['settling']
            rows.append(('到位判定', f'每轴误差 < {math.degrees(settling["position_tolerance_rad"]):.2f}°'
                         f' · 连续稳定 {settling["duration_s"]:g} s'
                         f' · 平均绝对速度 < {settling["velocity_tolerance_rad_s"]:g} rad/s'))
            rows.append(('参数来源', '内核启动时加载的 configs/control/motion.json'))
        rows.append(('操作结果', self.message))
        return rows

    def settling_reason(self, operation, state):
        detail = operation.get('settling')
        if detail:
            index, error, tolerance = detail['worst_joint'], detail['max_joint_error_rad'], detail['position_tolerance_rad']
        elif self.target is not None and self.description:
            errors = [abs(a-b) for a, b in zip(state['q'], self.target)]
            error = max(errors)
            index = errors.index(error) + 1
            tolerance = self.description['settling']['position_tolerance_rad']
        else:
            return '未满足到位条件'
        if error >= tolerance:
            return f'未满足到位条件 · J{index} 偏差 {math.degrees(error):.2f}°（要求 < {math.degrees(tolerance):.2f}°）'
        return '未满足到位条件 · 位置已进入容差，稳定时间或速度条件未通过'
