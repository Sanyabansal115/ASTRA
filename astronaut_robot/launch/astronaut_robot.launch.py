from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import xacro
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('astronaut_robot')
    xacro_file = os.path.join(pkg_share, 'urdf', 'astronaut_robot.urdf.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # Spawns the robot into whatever Gazebo world your teammate's launch file
    # has already brought up — this launch file assumes the world is running.
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'astronaut_robot', '-topic', 'robot_description'],
        output='screen',
    )

    joint_state_broadcaster_spawner = ExecuteProcess(
        cmd=['ros2', 'run', 'controller_manager', 'spawner', 'joint_state_broadcaster'],
        output='screen',
    )
    diff_drive_spawner = ExecuteProcess(
        cmd=['ros2', 'run', 'controller_manager', 'spawner', 'diff_drive_controller'],
        output='screen',
    )
    arm_controller_spawner = ExecuteProcess(
        cmd=['ros2', 'run', 'controller_manager', 'spawner', 'arm_controller'],
        output='screen',
    )

    task_server = Node(
        package='astronaut_robot',
        executable='robot_task_server.py',
        output='screen',
    )

    # Bridges the simulated LiDAR (Gazebo transport) into a real ROS 2 /scan
    # topic. Nav2/AMCL need this — without it there's no obstacle data at all.
    bridge_config = os.path.join(pkg_share, 'config', 'ros_gz_bridge.yaml')
    lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['--ros-args', '-p', f'config_file:={bridge_config}'],
        output='screen',
    )

    return LaunchDescription([
        robot_state_publisher,
        spawn_entity,
        joint_state_broadcaster_spawner,
        diff_drive_spawner,
        arm_controller_spawner,
        task_server,
        lidar_bridge,
    ])
