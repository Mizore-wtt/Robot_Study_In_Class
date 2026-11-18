#!/usr/bin/env python3
"""Finish active mapping trajectories and save .pbstream + 5 cm .pgm/.yaml."""
import argparse
from pathlib import Path
import subprocess

import rclpy
from cartographer_ros_msgs.srv import FinishTrajectory, GetTrajectoryStates, WriteState


def call(node, service_type, name, request, timeout=120.):
    client = node.create_client(service_type, name)
    if not client.wait_for_service(timeout_sec=10.):
        raise RuntimeError(f'Service unavailable: {name}')
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    if not future.done() or future.result() is None:
        raise RuntimeError(f'Service timed out: {name}')
    result = future.result()
    if result.status.code != 0:
        raise RuntimeError(f'{name}: {result.status.message}')
    node.destroy_client(client)
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--map', type=Path, default=Path.cwd() / 'maps' / 'parking',
                        help='Map stem without suffix; defaults to ./maps/parking')
    parser.add_argument('--overwrite', action='store_true')
    args = parser.parse_args()
    stem = args.map.expanduser().resolve()
    outputs = [Path(str(stem) + suffix) for suffix in ('.pbstream', '.yaml', '.pgm')]
    if not args.overwrite and any(p.exists() for p in outputs):
        parser.error('Map already exists; choose another --map or use --overwrite')
    stem.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = rclpy.create_node('save_cartographer_map')
    try:
        states = call(node, GetTrajectoryStates, '/get_trajectory_states', GetTrajectoryStates.Request())
        entries = list(zip(states.trajectory_states.trajectory_id,
                           states.trajectory_states.trajectory_state))
        if any(state == 2 for _, state in entries):
            raise RuntimeError('A frozen trajectory is loaded: save maps from mapping mode, not localization')
        for trajectory_id, state in entries:
            if state == 0:
                req = FinishTrajectory.Request()
                req.trajectory_id = trajectory_id
                call(node, FinishTrajectory, '/finish_trajectory', req)
        request = WriteState.Request()
        request.filename = str(outputs[0])
        request.include_unfinished_submaps = True
        call(node, WriteState, '/write_state', request)
        subprocess.run(['ros2', 'run', 'cartographer_ros', 'cartographer_pbstream_to_ros_map',
                        '-pbstream_filename', str(outputs[0]), '-map_filestem', str(stem),
                        '-resolution', '0.05'], check=True)
        if not all(p.is_file() and p.stat().st_size > 0 for p in outputs):
            raise RuntimeError('Map export produced missing or empty files')
        print('Saved map (mapping trajectory is now finished):')
        for p in outputs:
            print(p)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
