#!/usr/bin/env python3
"""Project a horizontal slice of /velodyne_points to /scan in the same frame."""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py.point_cloud2 import read_points_numpy


class PointCloudToScan(Node):
    def __init__(self):
        super().__init__('pointcloud_to_scan')
        defaults = {
            'min_height': -0.1, 'max_height': 0.1,
            'range_min': 0.2, 'range_max': 20.0,
            'scan_bins': 720, 'scan_time': 0.1,
        }
        for name, value in defaults.items():
            self.declare_parameter(name, value)
        self.values = {name: self.get_parameter(name).value for name in defaults}
        p = self.values
        if not (p['scan_bins'] >= 2 and p['min_height'] < p['max_height']
                and 0 < p['range_min'] < p['range_max'] and p['scan_time'] > 0):
            raise ValueError('Invalid height, range, scan_bins or scan_time parameters')
        self.publisher = self.create_publisher(LaserScan, 'scan', qos_profile_sensor_data)
        self.subscription = self.create_subscription(
            PointCloud2, 'velodyne_points', self.project, qos_profile_sensor_data)

    def project(self, cloud):
        points = read_points_numpy(cloud, field_names=('x', 'y', 'z'), skip_nans=True)
        p = self.values
        points = points.reshape(-1, 3)
        valid = np.isfinite(points).all(axis=1)
        valid &= (points[:, 2] >= p['min_height']) & (points[:, 2] <= p['max_height'])
        xy = points[valid, :2]
        distances = np.hypot(xy[:, 0], xy[:, 1])
        valid = (distances >= p['range_min']) & (distances <= p['range_max'])
        xy, distances = xy[valid], distances[valid]
        bins = p['scan_bins']
        increment = 2.0 * math.pi / bins
        # Wrap +pi into the first bin, consistently with a full 360-degree scan.
        indices = np.floor((np.arctan2(xy[:, 1], xy[:, 0]) + math.pi) / increment)
        indices = indices.astype(np.int64) % bins
        ranges = np.full(bins, np.inf, dtype=np.float32)
        np.minimum.at(ranges, indices, distances)
        scan = LaserScan()
        scan.header = cloud.header
        scan.angle_min = -math.pi
        scan.angle_increment = increment
        scan.angle_max = scan.angle_min + (bins - 1) * increment
        scan.range_min = p['range_min']
        scan.range_max = p['range_max']
        scan.scan_time = p['scan_time']
        # Gazebo emits one simultaneous cloud; no per-point time offsets exist.
        scan.time_increment = 0.0
        scan.ranges = ranges.tolist()
        self.publisher.publish(scan)


def main():
    rclpy.init()
    node = PointCloudToScan()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
