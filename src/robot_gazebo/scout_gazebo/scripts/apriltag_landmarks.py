#!/usr/bin/env python3
"""Convert course tag detections to native Cartographer landmark observations."""
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.executors import ExternalShutdownException
from apriltag_ros.msg import AprilTagDetectionArray
from cartographer_ros_msgs.msg import LandmarkEntry, LandmarkList
from tf2_geometry_msgs import do_transform_pose
from tf2_ros import Buffer, TransformException, TransformListener


class LandmarkBridge(Node):
    def __init__(self):
        super().__init__('apriltag_landmarks')
        self.declare_parameter('tracking_frame', 'imu_base')
        self.declare_parameter('translation_weight', 1.0)
        self.declare_parameter('rotation_weight', 1.0)
        self.frame = self.get_parameter('tracking_frame').value
        self.translation_weight = float(self.get_parameter('translation_weight').value)
        self.rotation_weight = float(self.get_parameter('rotation_weight').value)
        if not all(math.isfinite(v) and v > 0 for v in (
                self.translation_weight, self.rotation_weight)):
            raise ValueError('Landmark weights must be finite and positive')
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.publisher = self.create_publisher(LandmarkList, 'landmark', 10)
        self.subscription = self.create_subscription(
            AprilTagDetectionArray, 'tag_detections', self.convert, 10)

    def convert(self, detections):
        result = LandmarkList()
        result.header.stamp = detections.header.stamp
        result.header.frame_id = self.frame
        # Empty messages keep the landmark stream advancing without inventing
        # observations when no tag is visible.
        if detections.detections:
            try:
                transform = self.buffer.lookup_transform(
                    self.frame, detections.header.frame_id,
                    Time.from_msg(detections.header.stamp))
            except TransformException as exc:
                self.get_logger().warning(f'Skipping landmark without camera TF: {exc}',
                                          throttle_duration_sec=5.)
                return
            seen = set()
            for tag in detections.detections:
                if len(tag.id) != 1 or tag.id[0] in seen or tag.pose.header != detections.header:
                    continue
                p = tag.pose.pose.pose
                values = [p.position.x, p.position.y, p.position.z,
                          p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w]
                if not all(math.isfinite(v) for v in values) or p.position.z <= 0:
                    continue
                entry = LandmarkEntry()
                entry.id = str(tag.id[0])
                entry.tracking_from_landmark_transform = do_transform_pose(p, transform)
                entry.translation_weight = self.translation_weight
                entry.rotation_weight = self.rotation_weight
                result.landmarks.append(entry)
                seen.add(tag.id[0])
        self.publisher.publish(result)


def main():
    rclpy.init()
    node = LandmarkBridge()
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
