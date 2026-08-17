"""
ASTRA bringup: Gazebo world + robot + ROS-GZ bridge.

This is the ONLY launch file you need for the simulation itself.
There are no controller spawners, no controller_manager, and no
ros2_control - the drivetrain lives inside the URDF's DiffDrive plugin.

    ros2 launch astra_robot spacecraft.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    pkg = get_package_share_directory('astra_robot')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world = os.path.join(pkg, 'worlds', 'spacecraft_clean.sdf')
    xacro_file = os.path.join(pkg, 'urdf', 'astra_robot.urdf.xacro')
    bridge_cfg = os.path.join(pkg, 'config', 'ros_gz_bridge.yaml')

    robot_description = xacro.process_file(xacro_file).toxml()

    # Spawn inside Command Module (top-left room, centre -7,6), matching
    # locations.yaml's command_module entry.
    x = LaunchConfiguration('x', default='-7.0')
    y = LaunchConfiguration('y', default='6.5')
    z = LaunchConfiguration('z', default='0.08')
    yaw = LaunchConfiguration('yaw', default='0.0')

    # Lets Gazebo resolve package:// mesh URIs (the astronaut glb)
    resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.dirname(pkg),
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world}'}.items(),
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
    )

    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'config_file': bridge_cfg, 'use_sim_time': True}],
    )

    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'astra_robot',
                   '-x', x, '-y', y, '-z', z, '-Y', yaw],
    )

    # Give Gazebo a few seconds to finish loading the world before spawning,
    # otherwise `create` can fire at an empty world and silently do nothing.
    delayed_spawn = TimerAction(period=5.0, actions=[spawn])

    return LaunchDescription([
        DeclareLaunchArgument('x', default_value='-7.0'),
        DeclareLaunchArgument('y', default_value='6.5'),
        DeclareLaunchArgument('z', default_value='0.08'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        resource_path,
        gazebo,
        rsp,
        bridge,
        delayed_spawn,
    ])
