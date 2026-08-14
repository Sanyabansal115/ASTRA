"""
ASTRA full-stack demo: Gazebo + Nav2 + the LLM navigation agent.

    ros2 launch astra_llm_agent astra_full_demo.launch.py

This REPLACES the old astra_llm_demo.launch.py, which included
`rosbot_description/gazebo.launch.py` and `rosbot_nav2_bringup/nav2.launch.py`.
Those packages are gone - the simulation, world, map, and Nav2 config now all
live in the single `astra_robot` package.

Staging matters and is why the TimerActions are here:
  t=0s   Gazebo + robot + ros_gz bridge come up
  t=12s  Nav2 + AMCL + RViz (needs /clock, /scan and TF to already exist)
  t=25s  LLM agent (needs the navigate_to_pose action server to exist)

If your machine is slow, raise nav2_delay / agent_delay rather than debugging
phantom "server unavailable" errors.

Then, in another terminal:
    ros2 run astra_llm_agent prompt_cli
    > go check the cargo bay
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    llm_share = get_package_share_directory('astra_llm_agent')
    robot_share = get_package_share_directory('astra_robot')

    default_rooms = os.path.join(llm_share, 'config', 'rooms.yaml')
    default_env = os.path.join(llm_share, 'config', 'openai.env')

    nav2_delay = LaunchConfiguration('nav2_delay', default='12.0')
    agent_delay = LaunchConfiguration('agent_delay', default='25.0')

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_share, 'launch', 'spacecraft.launch.py')),
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(robot_share, 'launch', 'nav2.launch.py')),
    )

    llm_agent = Node(
        package='astra_llm_agent',
        executable='llm_nav_agent',
        name='astra_llm_nav_agent',
        output='screen',
        emulate_tty=True,               # keeps LangChain's verbose output readable
        parameters=[{
            'rooms_file': LaunchConfiguration('rooms_file'),
            'env_file': LaunchConfiguration('env_file'),
            'model': LaunchConfiguration('model'),
            # Must be true: the agent stamps goal poses with the clock, and
            # Nav2/Gazebo run on simulated time. Wall-clock stamps make the
            # TF lookup for that stamp fail and Nav2 rejects the goal.
            'use_sim_time': True,
            # astra_robot's URDF and nav2_params both use base_footprint as the
            # robot base frame; the agent's get_robot_pose tool looks up
            # map -> this frame.
            'robot_base_frame': 'base_footprint',
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument('rooms_file', default_value=default_rooms),
        DeclareLaunchArgument('env_file', default_value=default_env),
        DeclareLaunchArgument('model', default_value='mistral-large-latest'),
        DeclareLaunchArgument('nav2_delay', default_value='12.0'),
        DeclareLaunchArgument('agent_delay', default_value='25.0'),
        simulation,
        TimerAction(period=nav2_delay, actions=[nav2]),
        TimerAction(period=agent_delay, actions=[llm_agent]),
    ])
