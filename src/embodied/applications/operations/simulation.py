"""Scene state and repeatable tasks over an existing simulation kernel.

The public interface has no MuJoCo or device import. Physics snapshots belong
to the selected simulator; task success is an explicit predicate on observation.
"""
from copy import deepcopy
from dataclasses import dataclass


class Simulation:
    def __init__(self, robot):
        self.robot = robot

    def _client(self):
        client = self.robot._connected()
        if self.robot.backend != 'sim':
            raise ValueError('Scene snapshot/reset/restore are simulation capabilities')
        if 'simulation-snapshot' not in client.describe()['actions']:
            raise ValueError('The selected simulation plugin does not provide scene state')
        return client

    def snapshot(self):
        """Copy physics and observations. Running control tasks are not captured."""
        return self._client().request('simulation-snapshot')

    def observe(self):
        """Robot state and named body poses (metres, XYZW), from one physics state."""
        return self.snapshot()['observation']

    def restore(self, snapshot):
        """Restore physics and hold; finish recording and active motions first.

        The kernel advances the ownership epoch, invalidating old previews and
        separating recordings across the reset. Hardware never accepts this.
        """
        self._client()
        return self.robot._acquire().command('simulation-restore', snapshot=deepcopy(snapshot))

    def reset(self):
        """Restore the simulator's initial scene and hold its measured pose."""
        self._client()
        return self.robot._acquire().command('simulation-reset')

    def task(self, name, *, instruction, success, objects=(), goal=''):
        """Capture this scene as a task's repeatable initial state."""
        if not isinstance(name, str) or not name.strip() or len(name) > 200:
            raise ValueError('Task name must contain 1..200 characters')
        if not isinstance(instruction, str) or not instruction.strip() or len(instruction) > 2000:
            raise ValueError('Task instruction must contain 1..2000 characters')
        if not callable(success):
            raise ValueError('Task success must be an explicit observation predicate')
        if not isinstance(goal, str):
            raise ValueError('Task goal description must be text')
        if isinstance(objects, str):
            raise ValueError('objects must be a sequence of model body names')
        objects = tuple(objects)
        if any(not isinstance(name, str) or not name for name in objects) or len(set(objects)) != len(objects):
            raise ValueError('Task objects must be unique body names')
        initial = deepcopy(self.snapshot())
        missing = set(objects) - set(initial['observation'].get('bodies', {}))
        if missing:
            raise ValueError(f'Task objects absent from scene: {sorted(missing)}')
        return SimulationTask(self, name.strip(), instruction.strip(), objects, goal, initial, success)


@dataclass
class SimulationTask:
    simulation: Simulation
    name: str
    instruction: str
    objects: tuple
    goal: str
    initial_snapshot: dict
    success: object

    def reset(self):
        return self.simulation.restore(self.initial_snapshot)

    def check_success(self):
        result = self.success(self.simulation.observe())
        if type(result) is not bool:
            raise ValueError('Task success predicate must return a boolean')
        return result

    def describe(self):
        """Serializable task evidence; a callable is never disguised as metadata."""
        return {'schema_version': 1, 'name': self.name, 'instruction': self.instruction,
                'objects': list(self.objects), 'goal': self.goal,
                'initial_snapshot': deepcopy(self.initial_snapshot),
                'success_evaluator': 'caller-supplied observation predicate'}
