"""One-command demo: Gazebo + Nav2 + the ASTRA LLM navigation agent.

This is the "LLM integration" launch file listed in the project deliverables.
It only *includes* the teammates' existing launch files - it does not modify or
override their configuration:

    rosbot_description/launch/gazebo.launch.py   (world + robot + ros_gz bridge)
    rosbot_nav2_bringup/launch/nav2.launch.py    (map server, AMCL, Nav2, RViz)

The agent starts after a delay so Nav2 has time to come up, the same idea as
the professor's start_turtlebot3_nav2_agent.py in the Week 13 lab.

    ros2 launch astra_llm_agent astra_llm_demo.launch.py

Then, in a second terminal:

    ros2 topic pub --once /prompt std_msgs/msg/String "{data: 'Go to the medical bay'}"
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg_share = get_package_share_directory('astra_llm_agent')

    declare_agent_delay = DeclareLaunchArgument(
        'agent_delay', default_value='15.0',
        description='Seconds to wait for Gazebo and Nav2 before starting the agent')
    declare_model = DeclareLaunchArgument(
        'model', default_value='mistral-large-latest',
        description='Mistral model id; use mistral-small-latest if rate limited')

    # --- teammates' stack, included as-is -------------------------------------
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('rosbot_description'), 'launch', 'gazebo.launch.py',
        ]))
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('rosbot_nav2_bringup'), 'launch', 'nav2.launch.py',
        ]))
    )

    # --- this part of the project ---------------------------------------------
    llm_agent_node = Node(
        package='astra_llm_agent',
        executable='llm_nav_agent',
        name='astra_llm_nav_agent',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'rooms_file': os.path.join(pkg_share, 'config', 'rooms.yaml'),
            'env_file': os.path.join(pkg_share, 'config', 'openai.env'),
            'model': LaunchConfiguration('model'),
            'use_sim_time': True,
        }],
    )

    delayed_agent = TimerAction(
        period=LaunchConfiguration('agent_delay'),
        actions=[llm_agent_node],
    )

    return LaunchDescription([
        declare_agent_delay,
        declare_model,
        gazebo,
        nav2,
        delayed_agent,
    ])
