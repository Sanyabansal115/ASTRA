"""Launch only the ASTRA LLM navigation agent.

Assumes the team's Gazebo simulation and Nav2 stack are ALREADY running:

    ros2 launch rosbot_description gazebo.launch.py
    ros2 launch rosbot_nav2_bringup nav2.launch.py
    ros2 launch astra_llm_agent llm_agent.launch.py     <- this file

Nothing in the teammates' packages is modified or re-configured here.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('astra_llm_agent')

    default_rooms_file = os.path.join(pkg_share, 'config', 'rooms.yaml')
    default_env_file = os.path.join(pkg_share, 'config', 'openai.env')

    declare_rooms_file = DeclareLaunchArgument(
        'rooms_file', default_value=default_rooms_file,
        description='YAML file mapping room names to goal poses in the map frame')
    declare_env_file = DeclareLaunchArgument(
        'env_file', default_value=default_env_file,
        description='Fallback .env file holding MISTRAL_API_KEY (the environment '
                    'variable and ~/.astra_llm/openai.env are checked first)')
    declare_model = DeclareLaunchArgument(
        'model', default_value='mistral-large-latest',
        description='Mistral model id; use mistral-small-latest if rate limited')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='True',
        description='Use the Gazebo /clock, matching the rest of the stack')

    llm_agent_node = Node(
        package='astra_llm_agent',
        executable='llm_nav_agent',
        name='astra_llm_nav_agent',
        output='screen',
        emulate_tty=True,          # keeps LangChain's verbose output readable
        parameters=[{
            'rooms_file': LaunchConfiguration('rooms_file'),
            'env_file': LaunchConfiguration('env_file'),
            'model': LaunchConfiguration('model'),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }],
    )

    return LaunchDescription([
        declare_rooms_file,
        declare_env_file,
        declare_model,
        declare_use_sim_time,
        llm_agent_node,
    ])
