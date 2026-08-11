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

## What's now covered vs. what's still your Gazebo teammate's world

Added since the last pass: a LiDAR sensor on the URDF (`lidar_link`, front-mounted)
and a `ros_gz_bridge` config + launch node so it reaches ROS 2 as a real `/scan`
topic — that was a real gap before (the assignment requires LiDAR for obstacle
detection, and Nav2/AMCL can't function without a `/scan` topic at all).

**One thing this can't fully self-contain:** for the LiDAR to actually produce
data, Gazebo's Sensors system plugin has to be loaded — this is normally done at
the *world* level (in the `.sdf`/world file), not per-robot. Ask your Gazebo
teammate to include this in their world file's plugin list:
```xml
<plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
  <render_engine>ogre2</render_engine>
</plugin>
```
Everything else — the sensor definition itself, the bridge, the topic — is handled
in this package already.

## Integration checklist — confirm these with teammates today
1. **Gazebo teammate:** the mesh path assumes `meshes/cute_astronaut.glb` is in this
   package (it's already copied in). Confirm your version of Gazebo (gz-sim, ROS 2
   Jazzy) loads `.glb` directly via `<mesh filename="...">` — if not, we convert to
   `.dae` and swap one line in the xacro.
2. **Gazebo teammate:** load the Sensors system plugin in the world file (see above),
   or the LiDAR won't publish anything.
3. **Gazebo teammate:** `robot_task_server.py` calls two placeholder services —
   `/manipulator/attach_object` and `/manipulator/set_part_fixed` (both `std_srvs/Trigger`
   right now). These need to actually be wired to Gazebo's DetachableJoint system and to
   whatever mechanism changes the panel's look. Agree on real names/types together.
4. **LLM teammate:** call the `robot_task` action with `task_type` = `"navigate"`,
   `"grab"`, or `"fix"`. `mission_control_demo.py` shows the exact call pattern —
   send the goal, wait for the result, check `.success`.
5. **Everyone:** the `ARM_REACH`/`ARM_GRASP`/`ARM_LIFT`/`ARM_TOUCH` joint values in
   `robot_task_server.py` are placeholders. Once the arm spawns in Gazebo, drag it
   into good poses in RViz (same as the MoveIt lab), read the joint values off
   `/joint_states`, and swap them in.
6. **Module coordinates** in `mission_control_demo.py` are placeholders — swap in
   the real map coordinates once the world/map exists.

## Note on the assignment spec vs. the LLM-only design
The requirements PDF describes goal selection through *"the team's C++ navigation
menu program,"* with the LLM agent as an addition on top of it — not a full
replacement (step 5 vs. step 8 in the Navigation Methodology section). This
project uses the LLM as the only command interface, no menu program. Worth a
one-line confirmation with your professor, or a sentence in the report explicitly
justifying the substitution as a deliberate design choice.
