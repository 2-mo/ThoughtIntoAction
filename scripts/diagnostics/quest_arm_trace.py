"""Read native Quest/arm topics from one run and write timestamped JSON Lines."""
import argparse
import json
from pathlib import Path
import time
import tomllib

from embodied.control.ipc.services import create_node, make_subscriber, open_service
from embodied.control.ipc.types import Pose7, XrControllerInput
from embodied.devices.inputs.quest.native_health import TeleopGate, TeleopRouteHealth


def topics(run):
    arm = tomllib.loads((run / 'arm.toml').read_text())
    coordinator = tomllib.loads((run / 'coordinator.toml').read_text())
    return {
        'xr_input': (arm['leader_state_topic'].removesuffix('/pose') + '/input', XrControllerInput),
        'xr_pose': (arm['leader_state_topic'], Pose7),
        'gate': (arm['leader_gate_topic'], TeleopGate),
        'permit': (coordinator['permit_topic'], TeleopGate),
        'route_health': (arm['route_health_topic'], TeleopRouteHealth),
        'route_target': (arm['follower_command_topic'], Pose7),
        'ee_feedback': (arm['follower_state_topic'], Pose7),
    }


def capture(run, seconds, output):
    node = create_node('quest-arm-trace')
    subscriptions = {
        name: make_subscriber(open_service(node, topic, payload))
        for name, (topic, payload) in topics(run).items()
    }
    started = time.monotonic_ns()
    counts = dict.fromkeys(subscriptions, 0)
    print(json.dumps({'stage': 'start', 'run': str(run), 'monotonic_ns': started,
                      'wall_ns': time.time_ns()}), file=output)
    try:
        while time.monotonic_ns() - started < seconds * 1e9:
            for name, subscriber in subscriptions.items():
                # Drain in bounded batches so a busy topic cannot starve other topics.
                for _ in range(128):
                    sample = subscriber.receive()
                    if sample is None:
                        break
                    received_ns = time.monotonic_ns()
                    value = sample.payload().contents
                    row = {'stage': name, 'observed_monotonic_ns': received_ns,
                           'producer_timestamp_us': value.timestamp_us}
                    for key in ('sequence', 'grip', 'connection_state', 'tracking_valid',
                                'engaged', 'filter_valid', 'action', 'state'):
                        if hasattr(value, key):
                            row[key] = getattr(value, key)
                    if isinstance(value, Pose7):
                        row['pose'] = list(value.values)
                    print(json.dumps(row), file=output)
                    counts[name] += 1
                    del sample
            time.sleep(.002)
    except KeyboardInterrupt:
        pass
    print(json.dumps({'stage': 'end', 'counts': counts}), file=output)
    return all(counts.values())


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', type=Path, required=True, help='Live Quest kernel directory')
    parser.add_argument('--seconds', type=int, default=30)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.seconds < 1:
        parser.error('--seconds must be positive')
    try:
        with args.output.open('x') as output:
            complete = capture(args.run, args.seconds, output)
    except (OSError, ValueError, KeyError) as exc:
        parser.exit(1, f'{exc}\n')
    if not complete:
        parser.exit(1, 'Some topics produced no samples. Check the JSONL end record.\n')


if __name__ == '__main__':
    main()
