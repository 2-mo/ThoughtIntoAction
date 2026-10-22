"""Nonblocking release-then-home sequence shared by phone and terminal clients."""
import time


class ReleaseThenHome:
    def __init__(self, session, *, limits=None):
        self.session = session
        config = session.parameters['effective_robot_config']
        self.width = config['gripper_max_width']
        self.open_deadline = time.monotonic() + (self.width / config['gripper_velocity_max_m_s'] if self.width else 0.) + 2.
        self.phase = 'opening' if self.width else 'moving'
        self.limits = limits or {}
        self.plan = self.operation_id = self.result = None

    @property
    def metrics(self):
        return self.plan['metrics'] if self.plan else None

    def step(self, state):
        """Advance using fresh feedback. The caller retains its lease and stop UI."""
        if self.result is not None:
            return self.result
        if self.phase == 'opening':
            if abs(state['gripper_width'] - self.width) > .002:
                if time.monotonic() >= self.open_deadline:
                    raise RuntimeError('夹爪未能张开，已取消回零；请检查是否卡住')
                self.session.command('gripper', gripper_width=self.width, ttl_ms=200)
                return None
            self.phase = 'moving'
        if self.operation_id is None:
            self.plan = self.session.preview('home', gripper_width=self.width, **self.limits)
            self.operation_id = self.session.execute(self.plan)['operation_id']
            return None
        operation = next((op for op in state['operations'] if op['operation_id'] == self.operation_id), None)
        if operation is None:
            raise RuntimeError('回零状态丢失，未确认到位')
        if operation['status'] != 'running':
            self.result = operation
            self.phase = operation['status']
        return operation

    def wait(self, timeout=None):
        deadline = time.monotonic() + timeout if timeout is not None else None
        try:
            while True:
                result = self.step(self.session.status())
                if result is not None and result['status'] != 'running':
                    if result['status'] != 'completed':
                        raise RuntimeError(f'回零未完成：{result.get("reason") or result["status"]}')
                    return result
                if deadline is None and self.plan is not None:
                    deadline = time.monotonic() + self.plan['times'][-1] + 8.
                if deadline is not None and time.monotonic() >= deadline:
                    raise TimeoutError('回零超时，未确认到位')
                time.sleep(.04)
        except BaseException:
            try:
                self.session.command('cancel')
            except (OSError, ValueError, RuntimeError, EOFError):
                pass
            raise
