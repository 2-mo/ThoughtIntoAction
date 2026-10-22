"""Explicit demonstration origin; diagnostic/control traffic is not training data."""
from copy import deepcopy


def validate_demonstration(value):
    if not isinstance(value, dict) or value.get('schema') != 'embodied.demonstration/1':
        raise ValueError('Expected embodied.demonstration/1 provenance')
    allowed = {'human': {'quest', 'phone'}, 'scripted': {'robot-api'}}
    kind, sources = value.get('kind'), value.get('sources')
    if (kind not in allowed or not isinstance(sources, list) or not sources or
            any(not isinstance(source, str) for source in sources) or
            len(set(sources)) != len(sources) or not set(sources) <= allowed[kind]):
        raise ValueError('Demonstration sources must explicitly match human or scripted provenance')
    if kind == 'scripted' and (not isinstance(value.get('task_id'), str) or not value['task_id']):
        raise ValueError('A scripted demonstration must identify its task execution')
    return deepcopy(value)


def training_sources(metadata):
    value = metadata.get('demonstration')
    if value is None:
        return ('quest', 'phone')  # Historical human demonstrations; never authorize robot-api implicitly.
    value = validate_demonstration(value)
    if value['kind'] == 'scripted':
        execution = metadata.get('task_execution') or {}
        if execution.get('task_id') != value['task_id'] or execution.get('status') != 'completed':
            raise ValueError('Scripted demonstration requires its completed task execution; failed tasks remain raw data')
    return tuple(value['sources'])


def training_phases(metadata):
    value = metadata.get('demonstration')
    return (1, 4) if value is not None and validate_demonstration(value)['kind'] == 'scripted' else (1,)
