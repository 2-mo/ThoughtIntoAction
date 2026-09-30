"""Run one selected source plugin in an independent workstation process."""
import argparse
import json
import signal
from component_api.registry import Registry


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--run', required=True)
    parser.add_argument('--plugin', required=True)
    parser.add_argument('--options', default='{}')
    args = parser.parse_args()
    source = Registry().create('source', args.plugin, directory=args.run, **json.loads(args.options))
    def stop(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, stop)
    try:
        source.run()
    except KeyboardInterrupt:
        return 130
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
