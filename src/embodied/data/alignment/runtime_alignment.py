"""Causal RGB anchors for separately timestamped robot feedback and commands."""
from bisect import bisect_right
from collections import Counter
from embodied.data.episode.contract import runtime_contract, vector


def align(states, commands, frames, *, max_age_ns=100_000_000, sources=('quest',), phases=(1,), contract=None):
    """Return selected anchors and exclusion counts; leave raw records intact.

    Select the latest command first, then check its mode. Searching only valid
    commands would replay an old following action across Home or FreeDrive.
    """
    contract = contract if contract is not None else runtime_contract({})  # Historical X5 callers.
    clocks = [[row['host_monotonic_ns'] for row in rows] for rows in (states, commands)]
    for clock in clocks:
        if any(type(t) is not int or t <= 0 for t in clock) or any(a > b for a, b in zip(clock, clock[1:])):
            raise ValueError('Robot timestamps regress')
    selected, excluded = [], Counter()
    for frame in frames:
        stamp = frame['host_received_monotonic_ns']
        si, ci = (bisect_right(clock, stamp) - 1 for clock in clocks)
        if min(si, ci) < 0:
            excluded['no-causal-history'] += 1
            continue
        state, command = states[si], commands[ci]
        if (state.get('owner_epoch') != command.get('owner_epoch') or state['source'] != command['source']):
            excluded['owner-boundary'] += 1
            continue
        if max(stamp - clocks[0][si], stamp - clocks[1][ci]) > max_age_ns:
            excluded['stale-robot-data'] += 1
            continue
        if any(row['phase'] not in phases or row['source'] not in sources or row['error'] for row in (state, command)):
            excluded['phase-source-or-error'] += 1
            continue
        if any(row.get('mode') != 3 or (contract['gripper'] and row.get('gripper_mode') != 3) for row in (state, command)):
            excluded['not-position-following'] += 1
            continue
        if command.get('submitted') is not True:
            excluded['command-not-submitted'] += 1
            continue
        try:
            observation, action = vector(state, contract), vector(command, contract)
        except (ValueError, KeyError, TypeError):
            excluded['invalid-robot-vector'] += 1
            continue
        selected.append({'frame': frame, 'observation': observation, 'action': action,
            'state_monotonic_ns': clocks[0][si], 'action_monotonic_ns': clocks[1][ci],
            'state_sequence': state['sequence'], 'action_sequence': command['sequence']})
    return selected, dict(excluded)
