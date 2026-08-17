"""
SLAM mapping run - use this to generate the real slam_toolbox map your
rubric requires ("Map generated using slam_toolbox and saved correctly").

    # Terminal 1
    ros2 launch astra_robot spacecraft.launch.py
    # Terminal 2
    ros2 launch astra_robot slam.launch.py
    # Terminal 3 - drive it around ALL SIX rooms slowly
    ros2 run teleop_twist_keyboard teleop_twist_keyboard

    # Terminal 4 - when the map looks complete in RViz:
    ros2 run nav2_map_server map_saver_cli -f ~/spacecraft_map
    # then copy spacecraft_map.pgm/.yaml over src/astra_robot/maps/ and rebuild

teleop_twist_keyboard publishes plain geometry_msgs/msg/Twist on /cmd_vel,
which is exactly what our bridge and DiffDrive plugin expect - no extra
flags needed.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('astra_robot')
    slam_params = os.path.join(pkg, 'config', 'slam_toolbox.yaml')

    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_params, {'use_sim_time': True}],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=['-d', os.path.join(pkg, 'rviz', 'nav2_default_view.rviz')],
    )

    return LaunchDescription([slam, rviz])
