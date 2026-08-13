 # astronaut_robot — ASTRA Robot Branch

**ASTRA** (Autonomous Spacecraft Task & Repair Agent) — the robot package for the COMP219 group project.
This branch owns everything about the physical robot: its body, sensors, controllers, and the action interface that the LLM teammate calls to make it do things.

---

## What's in this branch and why

```
astronaut_robot/
├── action/
│   └── RobotTask.action          # The single action definition everything talks through
├── config/
│   ├── controllers.yaml          # ROS 2 Control config for base + arm
│   └── ros_gz_bridge.yaml        # Bridges Gazebo topics (LiDAR /scan) into ROS 2
├── launch/
│   └── astronaut_robot.launch.py # Spawns robot + controllers + task server
├── meshes/
│   └── cute_astronaut.glb        # Visual-only skin (no collision role)
├── scripts/
│   ├── robot_task_server.py      # The action server — LLM calls this
│   └── mission_control_demo.py   # Demo client that sequences a full mission
└── urdf/
    └── astronaut_robot.urdf.xacro  # Full robot model: base, wheels, arm, gripper, LiDAR
```

### `action/RobotTask.action`
Single action interface for all three things the robot can do:
| `task_type` | What happens |
|---|---|
| `"navigate"` | Forwards the goal to Nav2's `NavigateToPose` action |
| `"grab"` | Runs a scripted arm trajectory → reach → close gripper → lift |
| `"fix"` | Runs a scripted arm trajectory → reach → touch panel → trigger state change |

All three share the same goal/result/feedback shape so the LLM teammate only needs to know one interface.

### `urdf/astronaut_robot.urdf.xacro`
Defines the full robot:
- **Differential-drive base** with four wheels (ROS 2 Control `diff_drive_controller`)
- **2-joint arm** (shoulder + elbow) + **1-joint gripper** (`joint_trajectory_controller`)
- **LiDAR** (`lidar_link`, front-mounted) — required by Nav2/AMCL for `/scan`; the sensor itself is defined here, but Gazebo's Sensors system plugin must be loaded at the *world* level by the Gazebo teammate
- **Cute Astronaut mesh** as a visual-only `<visual>` on the base — real collision geometry is the box underneath

### `config/controllers.yaml`
Tells `ros2_control` what controllers to load:
- `diff_drive_controller` — drives the base from `/cmd_vel`
- `joint_trajectory_controller` — moves the arm from `robot_task_server.py`
- `joint_state_broadcaster` — publishes `/joint_states` for RViz and AMCL

### `config/ros_gz_bridge.yaml`
Bridges the Gazebo LiDAR topic (`/lidar`) into ROS 2 as `/scan` (`sensor_msgs/LaserScan`). Without this Nav2 has no sensor data at all.

### `scripts/robot_task_server.py`
The node the LLM teammate calls. Accepts `RobotTask` goals and handles them:
- **navigate** → calls Nav2 `NavigateToPose`
- **grab** → sends `FollowJointTrajectory` goals for reach → grasp → lift, then calls `/manipulator/attach_object` (Gazebo DetachableJoint bridge — confirm name with Gazebo teammate)
- **fix** → sends reach pose, calls `/manipulator/set_part_fixed` to change panel state

Arm poses (`ARM_REACH`, `ARM_GRASP`, etc.) are hand-picked placeholders — tune them by dragging the arm in RViz and reading `/joint_states`.

### `scripts/mission_control_demo.py`
Example client that sequences a complete mission:
`navigate → Cargo Bay` → `grab → toolkit` → `navigate → Engine Room` → `fix → engine_panel`

Module coordinates are placeholders — swap in real map coords once the Gazebo world exists. This is also the exact call pattern the LLM node should follow.

### `launch/astronaut_robot.launch.py`
Brings up: `robot_state_publisher` → Gazebo spawn → controller spawners → `robot_task_server`. Assumes the Gazebo world and Nav2 are **already running** (started by the Gazebo teammate's launch file).

---

## Build

Drop this folder into your workspace's `src/`, then:
```bash
cd ~/ros2_ws
colcon build --packages-select astronaut_robot
source install/setup.bash
```

---

## Run

> **Prerequisite:** the Gazebo world + Nav2 must already be up before running this.

```bash
ros2 launch astronaut_robot astronaut_robot.launch.py
```

Test the full mission sequence standalone:
```bash
ros2 run astronaut_robot mission_control_demo.py
```

Send a single task manually:
```bash
ros2 action send_goal /robot_task astronaut_robot/action/RobotTask \
  "{task_type: 'navigate', target_name: 'engine_room', x: 5.0, y: -2.0}"
```

---

## Integration checklist (confirm with teammates)

1. **Gazebo teammate** — add the Sensors system plugin to the world `.sdf` file, otherwise the LiDAR publishes nothing:
   ```xml
   <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
     <render_engine>ogre2</render_engine>
   </plugin>
   ```
2. **Gazebo teammate** — confirm the service names `/manipulator/attach_object` and `/manipulator/set_part_fixed` match what the DetachableJoint bridge actually exposes.
3. **LLM teammate** — use `mission_control_demo.py` as the call pattern. Goal fields: `task_type`, `target_name`, `x`, `y`. Check `result.success` after each goal.
4. **Everyone** — swap placeholder arm joint values and module map coordinates once the world is available.
