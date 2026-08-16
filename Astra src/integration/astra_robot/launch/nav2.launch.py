"""
Nav2 + AMCL localisation against the saved map, plus RViz.

Run AFTER spacecraft.launch.py is fully up:

    ros2 launch astra_robot nav2.launch.py

Then in RViz: "2D Pose Estimate" near Command Module, then "Nav2 Goal".

NOTE: no static map->odom publisher here on purpose. AMCL owns that
transform. Publishing a second frozen one on top makes navigation fail as
soon as the robot leaves the origin.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('astra_robot')
    nav2_bringup = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml = LaunchConfiguration(
        'map', default=os.path.join(pkg, 'maps', 'spacecraft_map.yaml'))
    params = LaunchConfiguration(
        'params_file', default=os.path.join(pkg, 'config', 'nav2_params.yaml'))

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, 'launch', 'bringup_launch.py')),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': 'true',
            'map': map_yaml,
            'params_file': params,
        }.items(),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', os.path.join(pkg, 'rviz', 'nav2_default_view.rviz')],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        DeclareLaunchArgument('map', default_value=os.path.join(
            pkg, 'maps', 'spacecraft_map.yaml')),
        DeclareLaunchArgument('params_file', default_value=os.path.join(
            pkg, 'config', 'nav2_params.yaml')),
        nav2,
        rviz,
    ])
