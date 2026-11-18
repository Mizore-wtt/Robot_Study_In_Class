#!/usr/bin/env python3
"""Apply RViz2 /initialpose through Cartographer's trajectory restart services."""
import math
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from cartographer_ros_msgs.srv import FinishTrajectory, GetTrajectoryStates, StartTrajectory
from geometry_msgs.msg import PoseWithCovarianceStamped


class InitialPose(Node):
    def __init__(self):
        super().__init__('cartographer_initial_pose')
        self.states = self.create_client(GetTrajectoryStates, '/get_trajectory_states')
        self.finish = self.create_client(FinishTrajectory, '/finish_trajectory')
        self.start = self.create_client(StartTrajectory, '/start_trajectory')
        self.sub = self.create_subscription(PoseWithCovarianceStamped, '/initialpose', self.receive, 1)
        self.pending = None
        self.busy = False

    def receive(self, msg):
        if self.busy:
            self.get_logger().warning('A trajectory restart is in progress; retry after completion')
            return
        if msg.header.frame_id != 'map':
            self.get_logger().error('Initial pose must use the map frame')
            return
        p, q = msg.pose.pose.position, msg.pose.pose.orientation
        values = [p.x, p.y, p.z, q.x, q.y, q.z, q.w]
        if not all(math.isfinite(x) for x in values) or abs(sum(x*x for x in values[3:]) - 1) > 1e-3:
            self.get_logger().error('Invalid initial pose or quaternion')
            return
        if not all(c.service_is_ready() for c in (self.states, self.finish, self.start)):
            self.get_logger().error('Cartographer services are not ready')
            return
        self.busy = True
        self.pending = msg.pose.pose
        self.states.call_async(GetTrajectoryStates.Request()).add_done_callback(self.on_states)

    def fail(self, message):
        self.get_logger().error(message)
        self.busy = False

    def on_states(self, future):
        result = future.result()
        entries = list(zip(result.trajectory_states.trajectory_id, result.trajectory_states.trajectory_state))
        frozen = [i for i, state in entries if state == 2]
        active = [i for i, state in entries if state == 0]
        if result.status.code or not frozen or len(active) != 1:
            self.fail('Initial pose requires one active localization trajectory and a frozen map')
            return
        self.reference = frozen[0]
        req = FinishTrajectory.Request()
        req.trajectory_id = active[0]
        self.finish.call_async(req).add_done_callback(self.on_finished)

    def on_finished(self, future):
        result = future.result()
        if result.status.code:
            self.fail(result.status.message)
            return
        req = StartTrajectory.Request()
        req.configuration_directory = str(Path(get_package_share_directory('scout_gazebo')) / 'config')
        req.configuration_basename = 'localization.lua'
        req.use_initial_pose = True
        req.initial_pose = self.pending
        req.relative_to_trajectory_id = self.reference
        self.start.call_async(req).add_done_callback(self.on_started)

    def on_started(self, future):
        result = future.result()
        if result.status.code:
            self.fail(result.status.message)
            return
        self.get_logger().info(f'Started localization trajectory {result.trajectory_id} with initial pose')
        self.busy = False


def main():
    rclpy.init()
    node = InitialPose()
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
