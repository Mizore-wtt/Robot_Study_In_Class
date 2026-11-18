"""Launch the experiment's Scout, ROS 2 sensors, cloud projection and RViz2."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    scout_share = get_package_share_directory('scout_gazebo')
    gazebo_share = get_package_share_directory('gazebo_ros')
    model_paths = [os.path.join(scout_share, 'models')]
    if os.environ.get('GAZEBO_MODEL_PATH'):
        model_paths.append(os.environ['GAZEBO_MODEL_PATH'])

    description = ParameterValue(Command([
        FindExecutable(name='xacro'), ' ', os.path.join(scout_share, 'urdf', 'base.xacro'),
        ' gpu:=', LaunchConfiguration('gpu'),
        ' lidar_samples:=', LaunchConfiguration('lidar_samples'),
    ]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', description='Open the Gazebo GUI.'),
        DeclareLaunchArgument('rviz', default_value='true', description='Open RViz2.'),
        DeclareLaunchArgument('gpu', default_value='false', description='Use the GPU ray sensor.'),
        DeclareLaunchArgument('lidar_samples', default_value='720'),
        DeclareLaunchArgument(
            'scene', default_value='mapping',
            choices=['mapping', 'localization', 'mapping_s', 'localization_s'],
            description='Full course scenes by default; *_s selects a simplified scene.'),
        DeclareLaunchArgument(
            'world', default_value=[scout_share, '/worlds/', LaunchConfiguration('scene'), '.world'],
            description='Optional absolute world path overriding the selected scene.'),
        DeclareLaunchArgument('x', default_value='0.0'),
        DeclareLaunchArgument('y', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        SetEnvironmentVariable('GAZEBO_MODEL_PATH', os.pathsep.join(dict.fromkeys(model_paths))),
        # All scene dependencies are bundled; startup needs no model downloads.
        SetEnvironmentVariable('GAZEBO_MODEL_DATABASE_URI', ''),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gazebo_share, 'launch', 'gazebo.launch.py')),
            launch_arguments={
                'world': LaunchConfiguration('world'),
                'gui': LaunchConfiguration('gui'),
                'verbose': 'true',
            }.items(),
        ),
        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            parameters=[{'robot_description': description, 'use_sim_time': True}],
            output='screen',
        ),
        Node(
            package='gazebo_ros', executable='spawn_entity.py',
            arguments=[
                '-entity', 'scout', '-topic', 'robot_description', '-timeout', '90',
                '-x', LaunchConfiguration('x'), '-y', LaunchConfiguration('y'),
                '-z', '0.05', '-Y', LaunchConfiguration('yaw'),
            ],
            output='screen',
        ),
        Node(
            package='scout_gazebo', executable='pointcloud_to_scan.py',
            name='pointcloud_to_scan',
            parameters=[os.path.join(scout_share, 'config', 'sensors.yaml'), {'use_sim_time': True}],
            output='screen',
        ),
        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', os.path.join(scout_share, 'config', 'simulation.rviz')],
            parameters=[{'use_sim_time': True}],
            condition=IfCondition(LaunchConfiguration('rviz')),
            output='screen',
        ),
    ])
