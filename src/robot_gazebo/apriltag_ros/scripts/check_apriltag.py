#!/usr/bin/env python3
"""Read-only check of live tag poses, image overlays and timestamped TF."""
import argparse
import json
import math
from pathlib import Path
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformListener
from apriltag_ros.msg import AprilTagDetectionArray


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--expected-id', type=int, default=None)
    parser.add_argument('--timeout', type=float, default=30.)
    parser.add_argument('--report-dir', type=Path)
    args = parser.parse_args()
    rclpy.init()
    node = Node('check_apriltag')
    buffer = Buffer()
    listener = TransformListener(buffer, node)
    messages, overlays = [], {}

    def stamp_key(stamp):
        return stamp.sec, stamp.nanosec

    def on_detection(msg):
        if msg.detections:
            messages.append(msg)
            del messages[:-20]

    def on_image(msg):
        overlays[stamp_key(msg.header.stamp)] = msg
        while len(overlays) > 50:
            del overlays[next(iter(overlays))]

    subscriptions = [
        node.create_subscription(AprilTagDetectionArray, '/tag_detections', on_detection, 10),
        node.create_subscription(Image, '/tag_detections_image', on_image, 10),
    ]
    result = {'passed': False}
    snapshot = None
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            for msg in reversed(messages):
                image = overlays.get(stamp_key(msg.header.stamp))
                if image is None or image.header.frame_id != msg.header.frame_id:
                    continue
                for detection in msg.detections:
                    if len(detection.id) != 1 or len(detection.size) != 1:
                        continue
                    tag_id = detection.id[0]
                    if args.expected_id is not None and tag_id != args.expected_id:
                        continue
                    pose = detection.pose.pose.pose
                    p, q = pose.position, pose.orientation
                    values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
                    if not all(math.isfinite(v) for v in values) or p.z <= 0:
                        continue
                    if abs(sum(v*v for v in values[3:]) - 1.) > 1e-6:
                        continue
                    if detection.pose.header != msg.header or detection.size[0] <= 0:
                        continue
                    try:
                        tf = buffer.lookup_transform(msg.header.frame_id, f'tag_{tag_id}',
                                                     Time.from_msg(msg.header.stamp))
                    except Exception:
                        continue
                    translation = tf.transform.translation
                    if max(abs(a-b) for a, b in zip(
                            (p.x, p.y, p.z), (translation.x, translation.y, translation.z))) > 1e-6:
                        continue
                    snapshot = CvBridge().imgmsg_to_cv2(image, 'bgr8')
                    green = ((snapshot[:, :, 1] > 180) & (snapshot[:, :, 0] < 80)
                             & (snapshot[:, :, 2] < 80)).sum()
                    red = ((snapshot[:, :, 2] > 180) & (snapshot[:, :, 0] < 80)
                           & (snapshot[:, :, 1] < 80)).sum()
                    if green < 20 or red < 10:
                        continue
                    result = {
                        'passed': True, 'id': tag_id, 'size_m': detection.size[0],
                        'frame': msg.header.frame_id, 'stamp': stamp_key(msg.header.stamp),
                        'position_m': [p.x, p.y, p.z],
                        'quaternion_xyzw': values[3:], 'tf_matches_pose': True,
                        'image_matches_stamp': True, 'overlay_green_pixels': int(green),
                        'overlay_red_pixels': int(red),
                    }
                    break
                if result['passed']:
                    break
            if result['passed']:
                break
        if args.report_dir:
            args.report_dir.mkdir(parents=True, exist_ok=True)
            (args.report_dir / 'check.json').write_text(json.dumps(result, indent=2) + '\n')
            if result['passed']:
                cv2.imwrite(str(args.report_dir / 'detection.png'), snapshot)
        print(json.dumps(result, indent=2))
        print('RESULT: PASS' if result['passed'] else 'RESULT: FAIL (aim camera at a visible tag)')
    finally:
        del subscriptions, listener
        node.destroy_node()
        rclpy.shutdown()
    return 0 if result['passed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
