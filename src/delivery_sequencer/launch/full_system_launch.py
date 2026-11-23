import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    rosbot_dir = get_package_share_directory('rosbot')
    nav_dir = get_package_share_directory('rosbot_nav')
    arm_dir = get_package_share_directory('rosbot_arm')

    return LaunchDescription([
        DeclareLaunchArgument(
            'map', default_value=os.path.join(nav_dir, 'maps', 'Robo_Lab.yaml')),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(nav_dir, 'params', 'nav2_params.yaml')),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(rosbot_dir, 'launch', 'sensors_launch.py')),
            launch_arguments={'launch_rviz': 'false'}.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav_dir, 'launch', 'bringup_launch.py')),
            launch_arguments={
                'map': LaunchConfiguration('map'),
                'params_file': LaunchConfiguration('params_file'),
                'use_sim_time': 'false',
            }.items()),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(arm_dir, 'launch', 'arm_launch.py'))),

        Node(
            package='block_detector', executable='block_detector',
            name='block_detector', output='screen'),
        Node(
            package='nav_manager', executable='nav_manager',
            name='nav_manager', output='screen'),
        Node(
            package='delivery_sequencer', executable='sequencer',
            name='sequencer', output='screen'),
    ])
