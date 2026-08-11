cmake_minimum_required(VERSION 3.8)
project(astronaut_robot)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/RobotTask.action"
  DEPENDENCIES action_msgs
)

# Python executables (nodes)
install(PROGRAMS
  scripts/robot_task_server.py
  scripts/mission_control_demo.py
  DESTINATION lib/${PROJECT_NAME}
)

# Everything else the launch file / robot_state_publisher / gz need at runtime
install(DIRECTORY urdf launch config meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
