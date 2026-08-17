# Gazebo
ros2 launch astra_robot spacecraft.launch.py

# Slam tool kit
ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file src/astra_robot/config/slam_toolbox.yaml -p use_sim_time:=true

# Nav2
ros2 launch astra_robot nav2.launch.py
