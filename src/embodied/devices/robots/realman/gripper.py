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
        raise RuntimeError(f'Gripper is moving or state unknown: {before}')
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
    """Release both controls to arm; consume presses even when arm is engaged."""
    def __init__(self):
        self.ready = False
        self.previous_a = False
        self.trigger_down = False
        self.last_action = float('-inf')

    def update(self, a, trigger, *, fresh, grip, now):
        import math
        if not math.isfinite(trigger) or not 0 <= trigger <= 1:
            raise ValueError('Invalid trigger reading')
        if not fresh:
            self.ready = False
        down = trigger > .8 or (self.trigger_down and trigger >= .2)
        opened = a and not self.previous_a
        closed = down and not self.trigger_down
        self.previous_a, self.trigger_down = bool(a), down
        if fresh and not a and trigger < .2:
            self.ready = True
        if not self.ready or not fresh or grip >= .2 or now - self.last_action < 1.:
            return None
        if opened == closed:
            return None
        self.last_action = now
        return 'open' if opened else 'close'
