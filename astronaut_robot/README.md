 # astronaut_robot

The robot piece of the COMP219 spacecraft project: mobile base + a deliberately
basic 2-joint arm + 1-joint gripper, with the "Cute Astronaut" mesh as a
visual-only skin on top (real mechanics underneath, astronaut look on top —
same trick as the walking-animation idea in the original plan).

## Layout
- `urdf/astronaut_robot.urdf.xacro` — the robot model (base, wheels, arm, gripper, mesh)
- `config/controllers.yaml` — diff-drive controller for the base, joint trajectory controller for the arm
- `action/RobotTask.action` — the one action everything is triggered through
- `scripts/robot_task_server.py` — handles navigate / grab / fix tasks (**run this**)
- `scripts/mission_control_demo.py` — example client, sequences a full cargo-bay-to-engine-room mission
- `launch/astronaut_robot.launch.py` — spawns the robot + controllers + task server

## Build
Drop this folder into your workspace's `src/`, then:
```
cd ~/ros2_ws
colcon build --packages-select astronaut_robot
source install/setup.bash
```

## Run (after the Gazebo world + Nav2 are already up)
```
ros2 launch astronaut_robot astronaut_robot.launch.py
```
Test it standalone with:
```
ros2 run astronaut_robot mission_control_demo.py
```

## What's  covered

Added since the last pass: a LiDAR sensor on the URDF (`lidar_link`, front-mounted)
and a `ros_gz_bridge` config + launch node so it reaches ROS 2 as a real `/scan`
topic — that was a real gap before (the assignment requires LiDAR for obstacle
detection, and Nav2/AMCL can't function without a `/scan` topic at all).
