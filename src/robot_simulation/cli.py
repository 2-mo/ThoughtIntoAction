"""Standalone, read-only 3D arm monitor. No input source or arm backend is opened."""

import argparse
import signal

from robot_core.model import DEFAULT_URDF, RobotModel
from robot_core.telemetry import StateSubscriber, endpoint
from robot_core.plugins import Registry


def main(argv=None):
    parser = argparse.ArgumentParser(prog="view-arm", description=__doc__)
    parser.add_argument("--urdf", default=str(DEFAULT_URDF))
    parser.add_argument("--eef", default="link6")
    parser.add_argument("--endpoint", help="Local state publisher endpoint; defaults to project telemetry config")
    parser.add_argument("--port", type=int, default=0, help="Browser HTTP port (0 chooses a free port)")
    parser.add_argument("--viewer", default="web", help="Display plugin; independent of controllers")
    parser.add_argument("--camera", action=argparse.BooleanOptionalAction, default=False, help="Ensure independent shared RGB service is running")
    parser.add_argument("--connections", action=argparse.BooleanOptionalAction, default=True, help="Read-only arm-service and USB RTT probes")
    args = parser.parse_args(argv)
    address = endpoint(args.endpoint)
    robot = RobotModel(args.urdf, args.eef)
    scene = Registry().create("viewer", args.viewer, robot=robot, port=args.port)
    previous = signal.getsignal(signal.SIGTERM)
    def interrupted(*_):
        raise KeyboardInterrupt
    signal.signal(signal.SIGTERM, interrupted)
    subscriber = None
    try:
        if args.connections and hasattr(scene, "connection_probe"):
            from robot_core.project import load_config, resolve_path
            from .connections import ConnectionProbe
            scene.connection_probe = ConnectionProbe(resolve_path(load_config()["arm_service"]["socket"]))
        if args.camera:
            from robot_core.camera_client import ensure_service
            try:
                ensure_service()
            except (OSError, RuntimeError) as exc:
                print(f"Camera service: {exc}", flush=True)
        subscriber = StateSubscriber(address)
        print(f"Watching robot state: {address}; waiting for arm-service", flush=True)
        while True:
            try:
                packet = subscriber.receive()
                if packet is not None:
                    scene.accept(packet)
            except (ValueError, TypeError, KeyError, OverflowError) as exc:
                scene.set_stream_error(str(exc))
    except KeyboardInterrupt:
        pass
    finally:
        if subscriber is not None:
            subscriber.close()
        if getattr(scene, "connection_probe", None):
            scene.connection_probe.close()
        scene.close()
        signal.signal(signal.SIGTERM, previous)


if __name__ == "__main__":
    main()
