"""Course Cartographer 2D mapping. Start the simulation separately."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory('scout_gazebo')
    tags = get_package_share_directory('apriltag_ros')
    return LaunchDescription([
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('apriltag', default_value='true',
                             description='Start detector; false if it is already running.'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(tags, 'launch', 'continuous_detection.launch.py')),
            condition=IfCondition(LaunchConfiguration('apriltag'))),
        Node(package='scout_gazebo', executable='apriltag_landmarks.py',
             parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='cartographer_ros', executable='cartographer_node',
             name='cartographer_node', parameters=[{'use_sim_time': True}],
             arguments=['-configuration_directory', os.path.join(share, 'config'),
                        '-configuration_basename', 'mapping.lua'],
             remappings=[('scan', '/scan'), ('imu', '/imu/data'), ('landmark', '/landmark')],
             output='screen'),
        Node(package='cartographer_ros', executable='cartographer_occupancy_grid_node',
             name='cartographer_occupancy_grid_node', parameters=[{'use_sim_time': True}],
             arguments=['-resolution', '0.05', '-publish_period_sec', '1.0'], output='screen'),
        Node(package='rviz2', executable='rviz2', name='rviz_mapping',
             parameters=[{'use_sim_time': True}],
             arguments=['-d', os.path.join(share, 'config', 'cartographer.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
    ])
