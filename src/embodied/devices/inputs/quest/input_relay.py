"""Forward controller input, suppressing samples published before the first valid frame.

The vendor coordinator disables the whole teleop group when the Episode
controller produces a single invalid sample. The xrobot device always emits one
such frame (`tracking_valid = 0`) right after it starts, so the dual-arm
coordinator would otherwise fall back to Disabled on every launch. After the
first valid frame every sample is forwarded unchanged, so a real tracking loss
still reaches the coordinator.
"""
import argparse
import time

import numpy as np

from embodied.control.ipc import create_node, make_publisher, make_subscriber, open_service, send
from embodied.control.ipc.types import Pose7, XrControllerInput


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--prefix', required=True)
    parser.add_argument('--channel', default='left_controller')
    args = parser.parse_args(argv)

    node = create_node('quest-input-relay')
    source = f'{args.prefix}/xr/{args.channel}/states/input'
    target = f'{args.prefix}/xr/{args.channel}/states/input_filtered'
    subscriber = make_subscriber(open_service(node, source, XrControllerInput))
    publisher = make_publisher(open_service(node, target, XrControllerInput, mode=True))
    pose_source = f'{args.prefix}/xr/{args.channel}/states/pose'
    pose_target = f'{args.prefix}/xr/{args.channel}/states/pose_filtered'
    pose_subscriber = make_subscriber(open_service(node, pose_source, Pose7))
    pose_publisher = make_publisher(open_service(node, pose_target, Pose7, mode=True))
    forwarding = pose_forwarding = False
    while True:
        sample = subscriber.receive()
        if sample is not None:
            payload = sample.payload().contents
            valid = payload.connection_state == 1 and payload.tracking_valid != 0
            if valid or forwarding:
                forwarding = True
                send(publisher, payload)
        pose_sample = pose_subscriber.receive()
        if pose_sample is not None:
            pose = pose_sample.payload().contents
            values = np.asarray(list(pose.values), dtype=float)
            valid_pose = np.isfinite(values).all() and np.linalg.norm(values[3:]) > .9
            if valid_pose or pose_forwarding:
                pose_forwarding = True
                send(pose_publisher, pose)
        if sample is None and pose_sample is None:
            time.sleep(.002)


if __name__ == '__main__':
    main()
