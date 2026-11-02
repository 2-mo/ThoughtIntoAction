"""EG2-4C2 controller commands; no power, serial or enable changes."""


def status(client):
    raw = client.query('get_gripper_state')
    fields = ('enable', 'status', 'error', 'mode', 'current_force', 'temperature', 'actpos')
    if any(type(raw.get(key)) is not int for key in fields):
        raise ValueError(f'Incomplete EG2-4C2 status: {raw}')
    return {key: raw[key] for key in fields}


def move(client, action, *, speed=100, force=50):
    if action not in ('open', 'close') or type(speed) is not int or not 1 <= speed <= 1000:
        raise ValueError('Invalid gripper action/speed')
    if type(force) is not int or not 50 <= force <= 1000:
        raise ValueError('Invalid gripper force')
    before = status(client)
    if before['enable'] != 1 or before['status'] != 1 or before['error'] != 0:
        raise RuntimeError(f'Gripper not ready: {before}')
    if before['mode'] not in (1, 2, 3, 6):
        if before['mode'] in (4, 5):
            return {'action': action, 'accepted': False, 'reason': 'busy; press again after motion completes'}
        raise RuntimeError(f'Unknown gripper mode: {before}')
    message = {'command': 'set_gripper_release' if action == 'open' else 'set_gripper_pick',
               'speed': speed, 'block': False}
    if action == 'close':
        message['force'] = force
    reply = client.exchange(message, 'set_gripper')
    if reply.get('state') is not True:
        raise RuntimeError(f'Gripper command rejected: {reply}')
    return {'action': action, 'accepted': True, 'completed': False, 'speed': speed,
            'force': force if action == 'close' else None}


class Buttons:
    """Hold controls to jog; release after startup, stale data or arm engagement."""
    def __init__(self):
        self.ready = False
        self.trigger_down = False

    def update(self, a, trigger, *, fresh, grip, now):
        import math
        if not math.isfinite(trigger) or not 0 <= trigger <= 1:
            raise ValueError('Invalid trigger reading')
        self.trigger_down = trigger > .8 or (self.trigger_down and trigger >= .2)
        if not fresh or grip >= .2:
            self.ready = False
        elif not a and trigger < .2:
            self.ready = True
        if not self.ready or not fresh or grip >= .2 or bool(a) == self.trigger_down:
            return None
        return 'open' if a else 'close'


class Jog:
    """Bounded position segments; never supersede a segment still in flight.

    The legacy controller exposes no documented independent gripper stop here.
    Release stops scheduling, not the already accepted segment. Position moves
    inherit the controller's speed/force settings; we do not alter them implicitly.
    """
    step = 10  # raw protocol units out of 1..1000, NOT millimetres.
    period = .15

    def __init__(self):
        self.next_poll = 0.
        self.pending = None
        self.sent_at = None
        self.contact = False

    def update(self, client, action, now):
        if action not in (None, 'open', 'close'):
            raise ValueError('Invalid jog action')
        if action is None:
            self.contact = False
        if now < self.next_poll or (action is None and self.pending is None):
            return None
        self.next_poll = now + self.period
        before = status(client)
        if before['enable'] != 1 or before['status'] != 1 or before['error'] != 0:
            raise RuntimeError(f'Gripper not ready: {before}')
        mode = before['mode']
        if mode not in (1, 2, 3, 4, 5, 6):
            raise RuntimeError(f'Unknown gripper mode: {before}')
        pos = before['actpos']
        if not -5 <= pos <= 1005:
            raise RuntimeError(f'Gripper opening outside expected raw range: {pos}')
        if mode == 6:
            self.contact = True
            self.pending = None
        if mode in (4, 5):
            if self.pending is not None and now - self.sent_at > 5.:
                raise RuntimeError('Gripper position segment still moving after 5 s')
            return {'accepted': False, 'reason': 'moving; waiting', 'feedback': before}
        if self.pending is not None:
            # An idle response immediately after ACK may still describe the old
            # position. Wait for target feedback rather than queueing more moves.
            if abs(pos - self.pending) > 3:
                if now - self.sent_at > 5.:
                    raise RuntimeError('Gripper position segment did not reach target in 5 s')
                return {'accepted': False, 'reason': 'awaiting position feedback', 'feedback': before}
            self.pending = None
        if action is None or (action == 'close' and self.contact):
            return {'accepted': False, 'reason': 'released/contact; no new segment', 'feedback': before}
        target = max(1, min(1000, pos + (self.step if action == 'open' else -self.step)))
        if abs(target - pos) <= 3:
            return {'accepted': False, 'reason': 'position boundary', 'feedback': before}
        reply = client.exchange({'command': 'set_gripper_position', 'position': target,
                                 'block': False}, 'set_gripper')
        if reply.get('state') is not True:
            raise RuntimeError(f'Gripper position command rejected: {reply}')
        self.pending, self.sent_at = target, now
        if action == 'open':
            self.contact = False
        return {'action': action, 'accepted': True, 'completed': False,
                'position_target_raw': target, 'segment_raw': self.step, 'feedback': before}
