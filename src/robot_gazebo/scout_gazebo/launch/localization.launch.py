"""Localize against a frozen pbstream and publish the saved occupancy map."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def start(context):
    share = get_package_share_directory('scout_gazebo')
    stem = os.path.abspath(os.path.expanduser(LaunchConfiguration('map').perform(context)))
    for suffix in ('.pbstream', '.yaml'):
        if not os.path.isfile(stem + suffix):
            raise RuntimeError(f'Missing saved map: {stem + suffix}; run save_map.py first.')
    return [
        Node(package='cartographer_ros', executable='cartographer_node',
             name='cartographer_node', parameters=[{'use_sim_time': True}],
             arguments=['-configuration_directory', os.path.join(share, 'config'),
                        '-configuration_basename', 'localization.lua',
                        '-load_state_filename', stem + '.pbstream', '-load_frozen_state', 'true'],
             remappings=[('scan', '/scan'), ('imu', '/imu/data'), ('landmark', '/landmark')],
             output='screen'),
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             parameters=[{'yaml_filename': stem + '.yaml', 'use_sim_time': True}], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='map_lifecycle_manager', parameters=[{'use_sim_time': True, 'autostart': True,
                                                       'node_names': ['map_server']}], output='screen'),
    ]


def generate_launch_description():
    share = get_package_share_directory('scout_gazebo')
    tags = get_package_share_directory('apriltag_ros')
    workspace = os.path.abspath(os.path.join(share, '../../../..'))
    return LaunchDescription([
        DeclareLaunchArgument('map', default_value=os.path.join(workspace, 'maps', 'parking'),
                             description='Absolute map stem without .pbstream/.yaml suffix.'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument('apriltag', default_value='true'),
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(tags, 'launch', 'continuous_detection.launch.py')),
            condition=IfCondition(LaunchConfiguration('apriltag'))),
        Node(package='scout_gazebo', executable='apriltag_landmarks.py',
             parameters=[{'use_sim_time': True}], output='screen'),
        Node(package='scout_gazebo', executable='initial_pose.py',
             parameters=[{'use_sim_time': True}], output='screen'),
        OpaqueFunction(function=start),
        Node(package='rviz2', executable='rviz2', name='rviz_localization',
             parameters=[{'use_sim_time': True}],
             arguments=['-d', os.path.join(share, 'config', 'cartographer.rviz')],
             condition=IfCondition(LaunchConfiguration('rviz')), output='screen'),
    ])
