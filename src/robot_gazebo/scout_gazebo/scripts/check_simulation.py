#!/usr/bin/env python3
"""Check live ROS 2 simulation data; --motion additionally drives the robot."""

import argparse
from collections import defaultdict, deque
import json
import math
from pathlib import Path
import subprocess
import time
import xml.etree.ElementTree as ET

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import GetModelList
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from rclpy.time import Time
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import CameraInfo, Image, Imu, JointState, LaserScan, PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy
from tf2_ros import Buffer, TransformListener


def stamp_seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


def yaw(pose):
    q = pose.orientation
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


class SimulationCheck(Node):
    TOPICS = {
        '/clock': Clock,
        '/odom': Odometry,
        '/ground_truth/odom': Odometry,
        '/joint_states': JointState,
        '/velodyne_points': PointCloud2,
        '/scan': LaserScan,
        '/imu/data': Imu,
        '/camera/color/image_raw': Image,
        '/camera/color/camera_info': CameraInfo,
        '/camera/depth/image_raw': Image,
        '/camera/depth/camera_info': CameraInfo,
    }

    def __init__(self, timeout):
        super().__init__('check_simulation')
        self.timeout = timeout
        self.messages = {}
        self.counts = defaultdict(int)
        self.stamps = defaultdict(lambda: deque(maxlen=500))
        self.backwards = set()
        self.subscriptions_ = [
            self.create_subscription(
                kind, name, lambda msg, topic=name: self.receive(topic, msg),
                QoSProfile(depth=10) if '/camera/' in name else qos_profile_sensor_data)
            for name, kind in self.TOPICS.items()
        ]
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.command = self.create_publisher(Twist, '/cmd_vel', 10)
        self.results = []
        self.metrics = {}

    def receive(self, topic, message):
        stamp = message.clock if topic == '/clock' else message.header.stamp
        value = stamp_seconds(stamp)
        previous = self.stamps[topic]
        if previous and value < previous[-1] - 1e-6:
            self.backwards.add(topic)
        previous.append(value)
        self.messages[topic] = message
        self.counts[topic] += 1

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while rclpy.ok() and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

    def record(self, name, passed, detail=''):
        entry = {'check': name, 'passed': bool(passed), 'detail': str(detail)}
        self.results.append(entry)
        print(f"[{'PASS' if passed else 'FAIL'}] {name}: {detail}", flush=True)

    def check_topics(self):
        deadline = time.monotonic() + self.timeout
        while time.monotonic() < deadline:
            if all(self.counts[name] >= 5 for name in self.TOPICS):
                break
            rclpy.spin_once(self, timeout_sec=0.1)
        for name in self.TOPICS:
            self.record('stream ' + name, self.counts[name] >= 5,
                        f'{self.counts[name]} messages')
        if any(name not in self.messages for name in self.TOPICS):
            return False
        self.spin_for(4.0)
        now = stamp_seconds(self.messages['/clock'].clock)
        rates = {}
        for name, stamps in self.stamps.items():
            if len(stamps) > 1 and stamps[-1] > stamps[0]:
                rates[name] = round((len(stamps) - 1) / (stamps[-1] - stamps[0]), 2)
        self.metrics['rates_per_sim_second'] = rates
        self.record('simulation clock advances', len(self.stamps['/clock']) > 1
                    and self.stamps['/clock'][-1] > self.stamps['/clock'][0], f'{now:.3f} s')
        self.record('timestamps monotonic', not self.backwards, sorted(self.backwards))
        stale = [name for name, stamps in self.stamps.items()
                 if now - stamps[-1] > 1.0 or stamps[-1] - now > 0.5]
        self.record('sensor timestamps follow /clock', not stale, stale or 'all fresh')
        expected = {'/odom': 50, '/joint_states': 30, '/velodyne_points': 10,
                    '/scan': 10, '/imu/data': 100, '/camera/color/image_raw': 15,
                    '/camera/depth/image_raw': 15}
        slow = {name: rates.get(name, 0) for name, rate in expected.items()
                if rates.get(name, 0) < rate * 0.65}
        self.record('sensor simulation-time rates', not slow, rates)
        return True

    def check_data(self):
        cloud = self.messages['/velodyne_points']
        xyz = read_points_numpy(cloud, field_names=('x', 'y', 'z'), skip_nans=True).reshape(-1, 3)
        xyz = xyz[np.isfinite(xyz).all(axis=1)]
        self.metrics['pointcloud_shape'] = [cloud.height, cloud.width]
        self.record('lidar point cloud', cloud.header.frame_id == 'velodyne'
                    and len(xyz) > 100, f'{cloud.height} x {cloud.width}, {len(xyz)} finite points')
        if len(xyz):
            angles = np.degrees(np.arctan2(xyz[:, 2], np.hypot(xyz[:, 0], xyz[:, 1])))
            beams = len(np.unique(np.round(angles, 1)))
            self.metrics['observed_lidar_elevations'] = beams
            self.record('16-beam lidar geometry', cloud.height == 16 or beams == 16,
                        f'{beams} observed vertical angles')
        scan = self.messages['/scan']
        values = np.asarray(scan.ranges)
        hits = values[np.isfinite(values)]
        self.metrics['scan_finite_bins'] = int(len(hits))
        self.record('2D scan contains obstacles', len(values) == 720 and len(hits) > 20
                    and not np.isnan(values).any() and np.all(hits >= scan.range_min - 1e-5)
                    and np.all(hits <= scan.range_max + 1e-5)
                    and scan.header.frame_id == cloud.header.frame_id,
                    f'{len(hits)}/{len(values)} finite bins in {scan.header.frame_id}')
        geometry_error = abs(scan.angle_max - scan.angle_min
                             - (len(scan.ranges) - 1) * scan.angle_increment)
        self.record('LaserScan angular geometry', geometry_error < 1e-5, geometry_error)
        imu = self.messages['/imu/data']
        a = imu.linear_acceleration
        gravity = math.sqrt(a.x * a.x + a.y * a.y + a.z * a.z)
        self.metrics['stationary_acceleration_norm'] = gravity
        self.record('IMU gravity and frame', 8.0 < gravity < 11.5
                    and imu.header.frame_id == 'imu_base', f'|a|={gravity:.3f} m/s^2')
        for camera in ('color', 'depth'):
            image = self.messages[f'/camera/{camera}/image_raw']
            info = self.messages[f'/camera/{camera}/camera_info']
            self.record(camera + ' image and intrinsics',
                        image.width == 640 and image.height == 480
                        and len(image.data) == image.step * image.height
                        and info.width == image.width and info.height == image.height
                        and info.k[0] > 0 and info.k[4] > 0
                        and image.header.frame_id == info.header.frame_id,
                        f'{image.width}x{image.height}, {image.encoding}, fx={info.k[0]:.2f}')
        depth = self.messages['/camera/depth/image_raw']
        if depth.encoding == '32FC1':
            dtype = np.dtype('>f4' if depth.is_bigendian else '<f4')
            depths = np.ndarray((depth.height, depth.width), dtype=dtype,
                                buffer=depth.data, strides=(depth.step, 4))
            valid = depths[np.isfinite(depths) & (depths > 0)]
            self.record('depth contains measured distances', len(valid) > 100,
                        f'{len(valid)} finite positive depths')
        else:
            self.record('depth encoding', False, depth.encoding)
        joints = self.messages['/joint_states']
        expected = {'front_left_wheel', 'rear_left_wheel', 'front_right_wheel', 'rear_right_wheel'}
        self.record('four physical wheel joint states', set(joints.name) == expected
                    and len(joints.position) == 4, joints.name)
        odom = self.messages['/odom']
        self.record('odometry frames', odom.header.frame_id == 'odom'
                    and odom.child_frame_id == 'base_footprint',
                    f'{odom.header.frame_id} -> {odom.child_frame_id}')

    def check_tf(self):
        frames = ['base_footprint', 'base_link', 'support', 'velodyne', 'imu_base',
                  'camera_link', 'camera_color_optical_frame', 'camera_depth_optical_frame',
                  'front_left_wheel_link', 'rear_left_wheel_link',
                  'front_right_wheel_link', 'rear_right_wheel_link']
        missing = [frame for frame in frames
                   if not self.tf_buffer.can_transform('odom', frame, Time())]
        self.record('complete odom/body/sensor/wheel TF tree', not missing, missing or frames)
        for name in ('/robot_state_publisher', '/pointcloud_to_scan'):
            # Nodes are checked by graph identity; use_sim_time is also validated via message stamps.
            known = {'/' + n for n, namespace in self.get_node_names_and_namespaces() if namespace == '/'}
            self.record('node ' + name, name in known, 'present' if name in known else 'missing')

    def check_model(self, scene):
        client = self.create_client(GetModelList, '/get_model_list')
        if not client.wait_for_service(timeout_sec=5.0):
            self.record('Gazebo model spawn', False, '/get_model_list unavailable')
            return
        future = client.call_async(GetModelList.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        response = future.result() if future.done() else None
        names = list(response.model_names) if response else []
        self.record('Gazebo model spawn', response is not None and 'scout' in names,
                    f'scout present among {len(names)} models')
        if scene == 'none' or response is None:
            return
        world_file = Path(get_package_share_directory('scout_gazebo')) / 'worlds' / (scene + '.world')
        world = ET.parse(world_file).getroot().find('world')
        expected = {model.get('name') for model in world.findall('model')}
        actual = set(names) - {'scout'}
        self.record('course scene model inventory', actual == expected,
                    f'{scene}: {len(actual)} models, missing={sorted(expected - actual)}, '
                    f'extra={sorted(actual - expected)}')
        reference_counts = {
            'mapping': (189, 62, 14), 'localization': (203, 76, 14),
            'mapping_s': (41, 9, 8), 'localization_s': (42, 10, 8),
        }
        counts = (len(actual), sum(n.startswith(('hatchback', 'prius')) for n in actual),
                  sum(n.startswith('Apriltag') for n in actual))
        self.metrics['scene'] = {'name': scene, 'models': counts[0], 'cars': counts[1],
                                 'apriltags': counts[2]}
        self.record('course scene size and vehicle/tag counts', counts == reference_counts[scene],
                    self.metrics['scene'])
        state = world.find('state')
        saved_models = {} if state is None else {
            model.get('name'): model for model in state.findall('model')}
        self.record('course saved model state retained', expected <= saved_models.keys(),
                    f'{len(saved_models)} saved poses for {len(expected)} scene models')
        if state is None:
            return
        if not scene.endswith('_s'):
            initial_mismatches = []
            for model in world.findall('model'):
                initial = np.fromstring(model.findtext('pose') or '0 0 0 0 0 0', sep=' ')
                saved = np.fromstring(saved_models[model.get('name')].findtext('pose'), sep=' ')
                if not np.allclose(initial, saved, atol=1e-6):
                    initial_mismatches.append(model.get('name'))
            self.record('initial model poses match saved scene layout', not initial_mismatches,
                        initial_mismatches or 'all declarations use the saved final poses')
        # These representatives catch missing state transforms and boundary/parking misalignment.
        anchors = (['carport1', 'ground_yellow_clone_0', 'wall_94m',
                    'wall_94m_clone', 'wall_51.5m_clone', 'hatchback',
                    'Apriltag36_11_00000_clone_0_clone_clone_0']
                   if not scene.endswith('_s') else
                   ['walls_16*30', 'carport1', 'ground_gray', 'hatchback_red',
                    'Apriltag36_11_00000'])
        anchors = [name for name in anchors if name in saved_models]
        pose_results = {}
        for name in anchors:
            expected_pose = np.fromstring(saved_models[name].findtext('pose'), sep=' ')
            try:
                # Native Gazebo transport discovery can lag ROS services during scene startup.
                # Retry discovery timeouts once; an actual pose mismatch still fails immediately.
                for attempt in range(2):
                    try:
                        result = subprocess.run(['gz', 'model', '-m', name, '-p'],
                                                capture_output=True, text=True,
                                                timeout=15, check=True)
                        break
                    except subprocess.TimeoutExpired:
                        if attempt == 1:
                            raise
                        self.spin_for(0.5)
                actual_pose = np.fromstring(result.stdout.strip(), sep=' ')
                if actual_pose.shape != (6,):
                    raise ValueError('Unexpected Gazebo pose response: ' + result.stdout.strip())
                position_error = float(np.linalg.norm(actual_pose[:3] - expected_pose[:3]))
                # Compare rotations rather than Euler components, which are ambiguous at gimbal lock.
                def rotation(angles):
                    roll, pitch, heading = angles
                    cr, sr = math.cos(roll), math.sin(roll)
                    cp, sp = math.cos(pitch), math.sin(pitch)
                    ch, sh = math.cos(heading), math.sin(heading)
                    return np.array([
                        [ch * cp, ch * sp * sr - sh * cr, ch * sp * cr + sh * sr],
                        [sh * cp, sh * sp * sr + ch * cr, sh * sp * cr - ch * sr],
                        [-sp, cp * sr, cp * cr]])
                rotation_error = float(np.linalg.norm(
                    rotation(actual_pose[3:]) - rotation(expected_pose[3:])))
                pose_results[name] = {'position_error_m': position_error,
                                      'rotation_matrix_error': rotation_error}
                self.record('saved scene pose ' + name,
                            position_error < 0.02 and rotation_error < 0.02,
                            f'position error={position_error:.6f} m, rotation error={rotation_error:.6f}')
            except (subprocess.SubprocessError, ValueError) as error:
                self.record('saved scene pose ' + name, False, str(error))
            self.spin_for(0.1)
        self.metrics['scene_anchor_poses'] = pose_results

    def drive(self, linear, angular, duration):
        start = stamp_seconds(self.messages['/clock'].clock)
        deadline = time.monotonic() + max(20, duration * 10)
        command = Twist()
        command.linear.x, command.angular.z = linear, angular
        while stamp_seconds(self.messages['/clock'].clock) - start < duration:
            if time.monotonic() > deadline:
                raise TimeoutError('Simulation clock stalled during motion check')
            self.command.publish(command)
            self.spin_for(0.05)

    def stop(self):
        for _ in range(12):
            self.command.publish(Twist())
            self.spin_for(0.05)

    def check_motion(self):
        print('Motion check: forward 0.2 m/s, then turn 0.4 rad/s; zero commands follow.', flush=True)
        self.drive(0.0, 0.0, 1.0)
        start = self.messages['/ground_truth/odom'].pose.pose
        odom_start = self.messages['/odom'].pose.pose
        self.drive(0.2, 0.0, 2.5)
        self.drive(0.0, 0.0, 1.0)
        end = self.messages['/ground_truth/odom'].pose.pose
        odom_end = self.messages['/odom'].pose.pose
        dx, dy = end.position.x - start.position.x, end.position.y - start.position.y
        heading = yaw(start)
        forward = dx * math.cos(heading) + dy * math.sin(heading)
        lateral = -dx * math.sin(heading) + dy * math.cos(heading)
        encoder_distance = math.hypot(odom_end.position.x - odom_start.position.x,
                                      odom_end.position.y - odom_start.position.y)
        self.metrics['forward_motion'] = {
            'ground_truth_forward_m': forward, 'ground_truth_lateral_m': lateral,
            'encoder_distance_m': encoder_distance,
        }
        self.record('positive cmd_vel moves physical robot forward', forward > 0.15
                    and abs(lateral) < 0.15 and encoder_distance > 0.15,
                    self.metrics['forward_motion'])
        initial_yaw = yaw(end)
        self.drive(0.0, 0.4, 2.5)
        self.drive(0.0, 0.0, 1.0)
        actual = self.messages['/ground_truth/odom']
        delta = yaw(actual.pose.pose) - initial_yaw
        delta = math.atan2(math.sin(delta), math.cos(delta))
        self.metrics['turn_radians'] = delta
        self.record('positive angular cmd_vel turns physical robot left', delta > 0.15,
                    f'{delta:.3f} rad')
        twist = actual.twist.twist
        speed = math.hypot(twist.linear.x, twist.linear.y)
        self.record('zero cmd_vel stops physical robot', speed < 0.04
                    and abs(twist.angular.z) < 0.08,
                    f'linear={speed:.4f} m/s angular={twist.angular.z:.4f} rad/s')


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--motion', action='store_true', help='Drive forward and turn in simulation.')
    parser.add_argument('--timeout', type=float, default=35.0)
    parser.add_argument('--scene', default='mapping',
                        choices=['mapping', 'localization', 'mapping_s', 'localization_s', 'none'],
                        help='Expected course scene; use none for a custom world.')
    parser.add_argument('--report', type=Path, help='Write a JSON result file.')
    args, ros_args = parser.parse_known_args()
    rclpy.init(args=ros_args)
    node = SimulationCheck(args.timeout)
    try:
        if node.check_topics():
            node.check_data()
            node.check_tf()
            node.check_model(args.scene)
            if args.motion and all(result['passed'] for result in node.results):
                node.check_motion()
            elif args.motion:
                node.record('motion check', False, 'Skipped because prerequisite checks failed')
    except (Exception, KeyboardInterrupt) as error:
        node.record('check execution', False, repr(error))
    finally:
        if args.motion and rclpy.ok():
            node.stop()
        passed = bool(node.results) and all(result['passed'] for result in node.results)
        report = {'passed': passed, 'motion_requested': args.motion,
                  'checks': node.results, 'metrics': node.metrics}
        if args.report:
            args.report.parent.mkdir(parents=True, exist_ok=True)
            args.report.write_text(json.dumps(report, indent=2, ensure_ascii=False) + '\n')
        print(f"RESULT: {'PASS' if passed else 'FAIL'} ({len(node.results)} checks)", flush=True)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0 if passed else 1


if __name__ == '__main__':
    raise SystemExit(main())
