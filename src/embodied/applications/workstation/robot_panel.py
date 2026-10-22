"""Two-key robot console using the existing leased Robot API.

Opening, refreshing or closing this panel never starts/stops the kernel.
Only P/H submit motion; space cancels this panel's operation and releases it.
"""
import math
import time

from embodied.applications.operations.robot import Robot
from embodied.applications.workstation.status import Status
from embodied.contracts.runtime.control import ControlClient
from embodied.core.presets import DEFAULT_GRASP_READY_JOINTS


class RobotControls:
    def __init__(self):
        self.robot = None
        self.motion = None
        self.run = None
        self.target = None
        self.started = None
        self.message = Status('按 P 到预抓取位，按 H 回零位', 'ready')
        self.plan_message = None

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
        if key not in ('p', 'h', ' ', 's'):
            return
        try:
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
            # 0 delegates velocity bounds to the backend. Acceleration/jerk and
            # settling rules are the existing kernel configuration, not UI caps.
            self.motion = self.robot.movej(self.target, speed=0, wait=False)
            name = '预抓取' if key == 'p' else '回零'
            metrics = self.motion.metrics
            self.plan_message = (f'{name} · 轨迹 {metrics["duration_s"]:.2f} s · '
                                 f'规划峰值 {metrics["peak_velocity_rad_s"]:.2f} rad/s')
            self.message = Status(f'正在{name}；空格停止并保持', 'waiting')
        except (OSError, ValueError, RuntimeError, EOFError) as exc:
            self.close()
            self.message = Status(f'动作未完成：{exc}', 'error')

    def rows(self, found):
        """Read feedback and preserve the kernel's actual completion/failure result."""
        rows = []
        try:
            if self.robot is not None and (found is None or found[0] != self.run):
                self.close()
                raise RuntimeError('运行实例已变化，当前动作已结束')
            if found is not None and found[1].get('managed'):
                state = ControlClient(found[0]).status()
                if self.motion is not None:
                    operation = next((op for op in state['operations']
                                      if op['operation_id'] == self.motion.operation_id), None)
                    if operation is not None and operation['status'] != 'running':
                        self.motion = None
                        self.close()
                        elapsed = time.monotonic() - self.started
                        if operation['status'] == 'completed':
                            self.message = Status(f'已到位并保持 · {elapsed:.2f} s', 'ready')
                        else:
                            reason = operation.get('reason') or operation['status']
                            if reason == 'Robot did not settle':
                                reason = '未满足到位条件'
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
            self.close()
            self.message = Status(f'反馈异常：{exc}', 'error')
        rows.extend([('速度设置', '后端每轴上限 · 无额外应用层速度上限'), ('操作结果', self.message)])
        return rows
