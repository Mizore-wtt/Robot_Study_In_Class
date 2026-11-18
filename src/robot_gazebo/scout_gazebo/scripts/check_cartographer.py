#!/usr/bin/env python3
"""Evaluate real SLAM against Gazebo truth; --drive follows course-scene aisles.

Ground truth is used only by this validation controller and error calculation.
It is never published to Cartographer or used to construct a map.
"""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from rclpy.signals import SignalHandlerOptions
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, OccupancyGrid
from cartographer_ros_msgs.msg import LandmarkList
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformException


def angle(x):
    return math.atan2(math.sin(x), math.cos(x))


def yaw(q):
    return math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))


def seconds(stamp):
    return stamp.sec + stamp.nanosec*1e-9


class Check(Node):
    def __init__(self):
        super().__init__('check_cartographer')
        self.truth = None
        self.history = deque(maxlen=500)
        self.grid = None
        self.scan = None
        self.tags = set()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.command = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subs = [
            self.create_subscription(Odometry, '/ground_truth/odom', self.on_truth, 100),
            self.create_subscription(OccupancyGrid, '/map', self.on_map,
                                     QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)),
            self.create_subscription(LaserScan, '/scan', lambda m: setattr(self, 'scan', m), qos_profile_sensor_data),
            self.create_subscription(LandmarkList, '/landmark', self.on_landmarks, 10),
        ]

    def on_truth(self, msg):
        p = msg.pose.pose
        self.truth = [seconds(msg.header.stamp), p.position.x, p.position.y, yaw(p.orientation)]
        self.history.append(self.truth)

    def on_map(self, msg):
        self.grid = msg

    def on_landmarks(self, msg):
        self.tags.update(entry.id for entry in msg.landmarks)

    def estimate(self):
        try:
            tf = self.buffer.lookup_transform('map', 'base_footprint', Time())
        except TransformException:
            return None
        stamp = seconds(tf.header.stamp)
        if not self.history:
            return None
        truth = min(self.history, key=lambda p: abs(p[0]-stamp))
        if abs(truth[0]-stamp) > .2:
            return None
        p = tf.transform.translation
        return [stamp, p.x, p.y, yaw(tf.transform.rotation)], truth

    def drive(self, x=0., z=0.):
        command = Twist()
        command.linear.x, command.angular.z = float(x), float(z)
        self.command.publish(command)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--drive', action='store_true', help='Moves Scout; stop keyboard teleop first')
    parser.add_argument('--route', choices=['parking', 'short'], default='parking')
    parser.add_argument('--reference', type=Path, help='Mapping report supplying a fixed world-to-map alignment')
    parser.add_argument('--duration', type=float, default=60., help='Read-only sampling duration in simulation seconds')
    parser.add_argument('--speed', type=float, default=0.9)
    parser.add_argument('--report', type=Path, required=True)
    args = parser.parse_args()
    # Keep the context alive until finally can publish the stop command on Ctrl+C.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = Check()
    alignment = None
    if args.reference:
        alignment = json.loads(args.reference.read_text())['world_to_map']
    # Main aisles in the full mapping.world, not straight lines through parked cars.
    route = [(-12.6, 0., math.pi/2), (-44.15, 0., math.pi/2),
             (-50., 0., None), (-50., -16., None), (25., -16., None),
             (25., -33., None), (-12.65, -33., -math.pi/2),
             (-44.14, -33., -math.pi/2), (-50., -33., None),
             (-50., 0., None), (0., 0., 0.)]
    if args.route == 'short':
        route = [(-12.6, 0., math.pi/2), (0., 0., 0.)]
    records, checkpoints = [], []
    index, phase, phase_start = 0, 'travel', None
    last_sample, start_sim = -1., None
    progress_time, best_distance = None, float('inf')
    deadline = time.monotonic() + (2700 if args.drive else max(180, args.duration*5))
    reason = 'timeout'
    try:
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.05)
            if node.truth is None or node.grid is None:
                continue
            now, x, y, heading = node.truth
            pair = node.estimate()
            if pair is None:
                continue
            estimated, truth = pair
            if start_sim is None:
                start_sim = now
            if alignment is None:
                theta = angle(estimated[3]-truth[3])
                c, s = math.cos(theta), math.sin(theta)
                alignment = [estimated[1]-c*truth[1]+s*truth[2],
                             estimated[2]-s*truth[1]-c*truth[2], theta]
            if now-last_sample >= 1.:
                tx, ty, theta = alignment
                c, s = math.cos(theta), math.sin(theta)
                expected = [tx+c*truth[1]-s*truth[2], ty+s*truth[1]+c*truth[2]]
                error = math.hypot(estimated[1]-expected[0], estimated[2]-expected[1])
                error_yaw = abs(angle(estimated[3]-truth[3]-theta))
                records.append({'sim_time': now, 'truth': truth[1:], 'estimate': estimated[1:],
                                'position_error_m': error, 'yaw_error_rad': error_yaw})
                last_sample = now
            if not args.drive:
                if now-start_sim >= args.duration:
                    reason = 'sampling_complete'
                    break
                continue
            gx, gy, final_yaw = route[index]
            distance = math.hypot(gx-x, gy-y)
            if phase == 'travel':
                if progress_time is None or distance < best_distance-.2:
                    progress_time, best_distance = now, distance
                if now-progress_time > 35.:
                    reason = f'no_progress_at_waypoint_{index}'
                    break
                if distance < .25:
                    phase, phase_start = 'orient', now
                    node.drive()
                else:
                    error = angle(math.atan2(gy-y, gx-x)-heading)
                    linear = min(args.speed, .8*distance)*max(0., math.cos(error))
                    if abs(error) > .3:
                        linear = 0.
                    if node.scan is not None and linear > 0:
                        a = node.scan.angle_min+np.arange(len(node.scan.ranges))*node.scan.angle_increment
                        front = np.array(node.scan.ranges)[np.abs(a) < .22]
                        if len(front) and np.nanmin(front) < .8:
                            reason = f'obstacle_at_waypoint_{index}'
                            break
                    node.drive(linear, max(-.7, min(.7, 1.2*error)))
            elif phase == 'orient':
                error = 0. if final_yaw is None else angle(final_yaw-heading)
                if abs(error) < .04:
                    node.drive()
                    phase, phase_start = 'dwell', now
                else:
                    node.drive(0., max(-.6, min(.6, 1.2*error)))
            elif now-phase_start >= 3.:
                checkpoints.append({'waypoint': index, **records[-1]})
                print(f'Waypoint {index+1}/{len(route)}: ({x:.2f}, {y:.2f}), '
                      f'error {records[-1]["position_error_m"]:.3f} m, tags {sorted(node.tags)}', flush=True)
                index += 1
                if index == len(route):
                    reason = 'route_complete'
                    break
                phase, progress_time, best_distance = 'travel', None, float('inf')
        errors = [r['position_error_m'] for r in records]
        tail = records[-min(20, len(records)):]
        tail_errors = [r['position_error_m'] for r in tail]
        grid = node.grid
        result = {
            'passed': reason in ('route_complete', 'sampling_complete') and bool(tail_errors)
                      and float(np.mean(tail_errors)) < .5
                      and float(np.mean([r['yaw_error_rad'] for r in tail])) < math.radians(5.),
            'reason': reason, 'world_to_map': alignment, 'observed_tag_ids': sorted(node.tags),
            'samples': len(records), 'position_rmse_m': float(np.sqrt(np.mean(np.square(errors)))) if errors else None,
            'final_mean_error_m': float(np.mean(tail_errors)) if tail_errors else None,
            'final_mean_yaw_error_deg': float(np.degrees(np.mean([r['yaw_error_rad'] for r in tail]))) if tail else None,
            'map': {'width': grid.info.width, 'height': grid.info.height,
                    'resolution': grid.info.resolution,
                    'known_cells': sum(v >= 0 for v in grid.data)} if grid else None,
            'checkpoints': checkpoints, 'trajectory': records,
        }
        args.report.parent.mkdir(parents=True, exist_ok=True)
        args.report.write_text(json.dumps(result, indent=2)+'\n')
        print(json.dumps({k: v for k, v in result.items() if k not in ('trajectory', 'checkpoints')}, indent=2))
        print('RESULT: PASS' if result['passed'] else 'RESULT: FAIL', flush=True)
        return 0 if result['passed'] else 1
    except KeyboardInterrupt:
        return 130
    finally:
        if args.drive:
            for _ in range(5):
                node.drive()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
