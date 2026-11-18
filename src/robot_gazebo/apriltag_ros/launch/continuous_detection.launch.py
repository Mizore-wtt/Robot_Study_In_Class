"""AprilTag detection for the course's ideal (zero-distortion) Gazebo camera."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/camera/color/image_raw'),
        DeclareLaunchArgument('camera_info_topic', default_value='/camera/color/camera_info'),
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            get_package_share_directory('apriltag_ros'), 'config', 'tags.yaml')),
        Node(package='apriltag_ros', executable='apriltag_node', name='apriltag',
             parameters=[LaunchConfiguration('params_file'), {
                 'use_sim_time': ParameterValue(LaunchConfiguration('use_sim_time'), value_type=bool)}],
             remappings=[('image', LaunchConfiguration('image_topic')),
                         ('camera_info', LaunchConfiguration('camera_info_topic'))],
             output='screen'),
    ])
